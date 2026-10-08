#include "QuantumNative.h"
#include "NativeViewport.hpp"
#include <quantum/editor/ApplicationService.hpp>
#include <quantum/coaster/RideHardwareSerialization.hpp>
#include <nlohmann/json.hpp>
#include <quantum/editor/RegionSummary.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace
{
    struct Failure : std::runtime_error
    {
        QnResult result;
        Failure(QnResult code, const std::string& message) : std::runtime_error(message), result(code) {}
    };
    struct Session
    {
        std::mutex mutex;
        quantum::editor::ApplicationService application;
        std::unique_ptr<NativeViewport> viewport;
        std::uint64_t routeRevision = 0;
        // Only the viewport owner thread reads/writes this publication marker.
        std::uint64_t viewportRevision = 0;
        std::uint64_t viewportGeometryGeneration = 0;
        std::uint64_t documentEpoch = 1, viewportFramedEpoch = 0;
        std::vector<QnRegion> regions;
        std::uint64_t authoringRevision = 0;
        std::vector<QnProfileSegment> profiles;
        std::vector<QnDevice> devices;
        nlohmann::json hardwareConfiguration = nlohmann::json::array();
        QnProfileCommand dragCommand{};
        std::uint64_t dragToken = 0;
        std::uint64_t selectedDevice = 0, selectedInfrastructure = 0;
        std::uint64_t supportRevision = 0, supportSelectionRevision = 1;
        std::vector<QnSupportStructure> supportStructures;
        std::vector<QnSupportNode> supportNodes;
        std::vector<QnSupportMember> supportMembers;
        std::optional<quantum::editor::SupportSelection> selectedSupport;
        bool supportWorkspace = false;
        uint64_t placementSelectionRevision = 1;
        bool selectedTrackStart = false;
        quantum::editor::StartPoseTransformMode transformMode = quantum::editor::StartPoseTransformMode::Select;
        bool closed = false;
        Session()
        {
            application.preview().setPhysicsDiagnosticsVisible(false);
            static_cast<void>(application.rebuildPreview());
        }
    };
    std::mutex handlesMutex;
    std::map<QnSession, std::shared_ptr<Session>> sessions;
    QnSession nextHandle = 1;
    // SDL event queue and the ImGui Vulkan presenter are process resources.
    // Phase 1 has one visible editor viewport, and any number of CPU sessions.
    std::mutex viewportMutex;
    bool viewportActive = false;

    void errorMessage(QnError* error, const char* message) noexcept
    {
        if (!error) return;
        const auto length = std::min(std::strlen(message), sizeof(error->message) - 1);
        std::memcpy(error->message, message, length);
        error->message[length] = '\0';
    }
    template<class Operation>
    QnResult boundary(QnError* error, Operation operation) noexcept
    {
        errorMessage(error, "");
        try { operation(); return QN_OK; }
        catch (const Failure& failure) { errorMessage(error, failure.what()); return failure.result; }
        catch (const std::invalid_argument& failure) { errorMessage(error, failure.what()); return QN_INVALID_ARGUMENT; }
        catch (const std::out_of_range& failure) { errorMessage(error, failure.what()); return QN_INVALID_ARGUMENT; }
        catch (const std::exception& failure) { errorMessage(error, failure.what()); return QN_INTERNAL_ERROR; }
        catch (...) { errorMessage(error, "Unknown native failure."); return QN_INTERNAL_ERROR; }
    }
    std::shared_ptr<Session> session(QnSession handle)
    {
        std::lock_guard lock(handlesMutex);
        const auto found = sessions.find(handle);
        if (found == sessions.end()) throw Failure(QN_INVALID_HANDLE, "Invalid or stale QUANTUM session.");
        return found->second;
    }
    void requireOpen(const Session& state)
    {
        if (state.closed) throw Failure(QN_INVALID_HANDLE, "QUANTUM session has been destroyed.");
    }
    std::filesystem::path utf8Path(const char* path)
    {
        if (!path || !*path) throw std::invalid_argument("A nonempty UTF-8 path is required.");
        return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(path)));
    }
    void refreshRegions(Session& state)
    {
        auto& app = state.application;
        if (state.routeRevision == app.revision()) return;
        std::vector<QnRegion> regions;
        const auto& track = app.track();
        const auto& geometry = app.centerline().visualization();
        const auto units = track.physicalSettings().metersPerCoordinateUnit;
        double station = 0;
        regions.reserve(track.sectionCount());
        for (std::size_t i = 0; i < track.sectionCount(); ++i)
        {
            const auto& section = track.section(i);
            quantum::editor::RegionNetRotation rotation;
            if (section.kind == quantum::coaster::RegionKind::RateProfiles)
                rotation = quantum::editor::computeNetRotationDegrees(section.rateProfileRegion().rateProfiles);
            const auto& slice = geometry.sectionSlices.at(i);
            const double length = quantum::coaster::sectionLength(section) * units;
            regions.push_back({static_cast<uint32_t>(i), static_cast<uint32_t>(section.kind),
                quantum::coaster::isForceDrivenSection(section) ? 2u : section.kind == quantum::coaster::RegionKind::Geometry ? 1u : 0u,
                0, station, length, station + length, rotation.rollDegrees, rotation.pitchDegrees, rotation.yawDegrees,
                slice.startPosition.x * units, slice.startPosition.y * units, slice.startPosition.z * units,
                slice.endPosition.x * units, slice.endPosition.y * units, slice.endPosition.z * units});
            station += length;
        }
        state.regions = std::move(regions);
        state.routeRevision = app.revision();
    }

    void copyLabel(char* output, std::size_t capacity, std::string_view value)
    {
        auto count = std::min(capacity - 1, value.size());
        // Do not split a UTF-8 code point when a long authored name is bounded.
        while (count < value.size() && count && (static_cast<unsigned char>(value[count]) & 0xc0) == 0x80) --count;
        std::memcpy(output, value.data(), count); output[count] = 0;
    }
    void appendProfileSamples(std::vector<QnProfileSegment>& profiles,
        const quantum::coaster::ChannelProfile& profile,
        uint32_t owner, uint64_t id, uint32_t channel, uint32_t unit, double domainScale, double valueScale)
    {
        for (const auto& segment : profile.segments)
        {
            const auto& transition = segment.transition;
            QnProfileSegment row{};
            row.owner_kind = owner; row.owner_id = id; row.channel = channel;
            row.segment_id = segment.id; row.transition_type = static_cast<uint32_t>(transition.transitionType);
            row.unit = unit; row.begin_meters = transition.domainBegin * domainScale;
            row.end_meters = transition.domainEnd * domainScale;
            row.value_begin = transition.valueBegin * valueScale; row.value_end = transition.valueEnd * valueScale;
            for (unsigned sample = 0; sample < 33; ++sample)
                row.samples[sample] = quantum::math::evaluateScalarTransition(transition,
                    std::lerp(transition.domainBegin, transition.domainEnd, sample / 32.0)) * valueScale;
            profiles.push_back(row);
        }
    }
    void refreshAuthoring(Session& state)
    {
        auto& app = state.application;
        if (state.authoringRevision == app.revision()) return;
        std::vector<QnProfileSegment> profiles;
        std::vector<QnDevice> devices;
        nlohmann::json hardwareConfiguration = nlohmann::json::array();
        const auto& track = app.track();
        const auto scale = track.physicalSettings().metersPerCoordinateUnit;
        for (std::size_t region = 0; region < track.sectionCount(); ++region)
        {
            const auto& section = track.section(region);
            const bool force = quantum::coaster::isForceDrivenSection(section);
            if (section.kind != quantum::coaster::RegionKind::RateProfiles && !force) continue;
            const auto styles = force ? quantum::editor::forceDrivenRowStyles(scale) : quantum::editor::rateProfileRowStyles(scale);
            for (uint32_t channel = 0; channel < 3; ++channel)
                appendProfileSamples(profiles, quantum::editor::sectionProfileChannel(section, static_cast<quantum::editor::ProfileChannel>(channel)),
                    0, region, channel, force && channel > 0 ? 1 : 0, scale, styles[channel].authoredToDisplay);
        }
        for (const auto& device : track.trackDevices().devices)
        {
            hardwareConfiguration.push_back({{"id", device.id}, {"configuration",
                nlohmann::json::parse(quantum::coaster::serializeRideHardwareConfiguration(device))}});
            QnDevice row{}; row.id = device.id; row.kind = static_cast<uint32_t>(device.kind);
            row.enabled = device.enabled; row.custom = device.accelerationProfile.has_value();
            row.start_station = device.startStationMeters; row.end_station = device.endStationMeters;
            row.acceleration = device.targetAccelerationMetersPerSecondSquared; row.maximum_force = device.maximumForceNewtons;
            copyLabel(row.name, sizeof(row.name), device.name); devices.push_back(row);
            if (device.accelerationProfile) appendProfileSamples(profiles, *device.accelerationProfile, 1, device.id, 3, 2, 1, 1);
        }
        state.profiles = std::move(profiles); state.devices = std::move(devices);
        state.hardwareConfiguration = std::move(hardwareConfiguration);
        state.authoringRevision = app.revision();
    }
    void requireRevision(const Session& state, uint64_t revision)
    {
        if (state.application.revision() != revision)
            throw Failure(QN_CONFLICT, "Document changed. Refresh the workspace and try again.");
    }
    void selectSupport(Session& state, std::optional<quantum::editor::SupportSelection> selection)
    {
        if (selection == state.selectedSupport) return;
        state.selectedSupport = selection;
        ++state.supportSelectionRevision;
    }
    void selectTrackStart(Session& state, bool selected, quantum::editor::StartPoseTransformMode mode)
    {
        if (state.selectedTrackStart == selected && state.transformMode == mode) return;
        state.application.cancelTransform();
        state.selectedTrackStart = selected; state.transformMode = mode;
        ++state.placementSelectionRevision;
    }
    void refreshSupports(Session& state)
    {
        auto& app = state.application;
        if (state.selectedSupport && !quantum::editor::supportSelectionExists(app.track().supports(), *state.selectedSupport))
        {
            const quantum::editor::SupportSelection parent{state.selectedSupport->structureId,
                quantum::editor::SupportSelectionKind::Structure, 0};
            selectSupport(state, quantum::editor::supportSelectionExists(app.track().supports(), parent)
                ? std::optional{parent} : std::nullopt);
        }
        if (state.supportRevision == app.revision()) return;
        std::vector<QnSupportStructure> structures;
        std::vector<QnSupportNode> nodes;
        std::vector<QnSupportMember> members;
        const auto scale = app.track().physicalSettings().metersPerCoordinateUnit;
        const auto& visualization = app.supports();
        std::size_t nodeIndex = 0;
        const auto copyConnection = [](const auto& source) -> QnSupportConnection
        {
            QnSupportConnection result{};
            if (!source) return result;
            result.treatment = static_cast<uint32_t>(source->treatment) + 1;
            result.flags = (source->localPlacement ? 1u : 0u) | (source->mounting ? 2u : 0u);
            if (source->asset)
            {
                copyLabel(result.asset, sizeof(result.asset), source->asset->path);
                if (source->asset->path.size() >= sizeof(result.asset)) result.flags |= 4;
            }
            return result;
        };
        for (const auto& structure : app.track().supports().structures)
        {
            QnSupportStructure row{structure.id, static_cast<uint32_t>(structure.nodes.size()),
                static_cast<uint32_t>(structure.members.size()), structure.generatedWoodenRun.has_value() || structure.generatedFrame.has_value()};
            copyLabel(row.name, sizeof(row.name), structure.name); structures.push_back(row);
            for (const auto& node : structure.nodes)
            {
                const auto position = node.position * scale;
                // Visualization preserves authored node order and owns resolved
                // track-frame evaluation. Managed code receives only its result.
                const auto resolved = visualization.nodes.at(nodeIndex++).position * scale;
                const auto attachment = node.trackAttachment.value_or(quantum::coaster::TrackAttachment{});
                nodes.push_back({structure.id, node.id, node.trackAttachment ? 1u : node.foundation ? 2u : 0u, 0,
                    position.x, position.y, position.z, resolved.x, resolved.y, resolved.z,
                    attachment.station * scale, attachment.lateralOffset * scale, attachment.verticalOffset * scale});
            }
            for (const auto& member : structure.members)
            {
                const auto profile = quantum::coaster::resolveSupportProfile(structure, member);
                members.push_back({structure.id, member.id, member.startNodeId, member.endNodeId,
                    static_cast<uint32_t>(profile.shape), static_cast<uint32_t>(member.role),
                    static_cast<uint32_t>(member.orientation), member.orientationReference.has_value(),
                    profile.outerDimensions.x * scale, profile.outerDimensions.y * scale,
                    profile.wallThickness * scale, copyConnection(member.startConnection), copyConnection(member.endConnection)});
            }
        }
        state.supportStructures = std::move(structures); state.supportNodes = std::move(nodes);
        state.supportMembers = std::move(members); state.supportRevision = app.revision();
    }
    void requireProfileCommand(const QnProfileCommand* command)
    {
        if (!command || command->owner_kind > 1 || command->operation > 6
            || (command->owner_kind == 0 ? command->channel > 2 : command->channel != 3))
            throw std::invalid_argument("Invalid profile command target or operation.");
    }
    void requireDrag(Session& state, uint64_t token)
    {
        if (!token || token != state.dragToken)
            throw std::invalid_argument("This Timeline drag is no longer active.");
        if (state.dragCommand.expected_revision != state.application.revision())
        {
            state.application.cancelProfileDrag(token);
            state.dragToken = 0;
            throw Failure(QN_CONFLICT, "Document changed. The Timeline drag was canceled.");
        }
        static_cast<void>(state.application.profileDragRevision(token));
    }
}

static_assert(sizeof(QnSnapshot) == 104);
static_assert(sizeof(QnRegion) == 112);
static_assert(sizeof(QnCommand) == 24);
static_assert(sizeof(QnProfileSegment) == 328 && sizeof(QnDevice) == 184);
static_assert(sizeof(QnAuthoringSnapshot) == 24 && sizeof(QnProfileCommand) == 48);
static_assert(sizeof(QnSupportStructure) == 272 && sizeof(QnSupportNode) == 88);
static_assert(sizeof(QnSupportConnection) == 520 && sizeof(QnSupportMember) == 1096);
static_assert(sizeof(QnSupportSnapshot) == 64 && sizeof(QnSupportCommand) == 88);
static_assert(sizeof(QnPose) == 56 && sizeof(QnPlacementSnapshot) == 184);
static_assert(sizeof(QnTransformCommand) == 32 && sizeof(QnTransformIntent) == 48);
uint32_t QN_CALL qn_abi_version() { return 5; }
QnResult QN_CALL qn_create(QnSession* out, QnError* error)
{
    if (out) *out = 0;
    return boundary(error, [&] {
        if (!out) throw std::invalid_argument("Missing session output.");
        auto state = std::make_shared<Session>();
        std::lock_guard lock(handlesMutex);
        if (nextHandle >= static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
            throw Failure(QN_UNAVAILABLE, "Session handle space exhausted.");
        const auto handle = nextHandle++;
        sessions.emplace(handle, std::move(state));
        *out = handle;
    });
}
QnResult QN_CALL qn_destroy(QnSession handle, QnError* error)
{
    return boundary(error, [&] {
        auto state = session(handle);
        std::lock_guard lock(state->mutex);
        requireOpen(*state);
        if (state->viewport) throw Failure(QN_BUSY, "Close the viewport on its UI thread before destroying the session.");
        state->closed = true;
        std::lock_guard handlesLock(handlesMutex);
        sessions.erase(handle);
    });
}
QnResult QN_CALL qn_new(QnSession handle, QnError* error)
{
    return boundary(error, [&] {
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        state->application.newDocument();
        state->selectedInfrastructure = 0;
        ++state->documentEpoch;
        selectSupport(*state, std::nullopt);
        selectTrackStart(*state, false, quantum::editor::StartPoseTransformMode::Select);
    });
}
QnResult QN_CALL qn_open(QnSession handle, const char* path, QnError* error)
{
    return boundary(error, [&] {
        const auto input = utf8Path(path);
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        state->application.open(input);
        state->selectedInfrastructure = 0;
        ++state->documentEpoch;
        selectSupport(*state, std::nullopt);
        selectTrackStart(*state, false, quantum::editor::StartPoseTransformMode::Select);
    });
}
QnResult QN_CALL qn_save(QnSession handle, const char* path, QnError* error)
{
    return boundary(error, [&] {
        const auto output = utf8Path(path);
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        try { state->application.save(output); }
        catch (const std::runtime_error& failure) { throw Failure(QN_IO_ERROR, failure.what()); }
    });
}
QnResult QN_CALL qn_snapshot(QnSession handle, QnSnapshot* out,
    QnRegion* regions, uint32_t capacity, uint64_t knownRevision, QnError* error)
{
    return boundary(error, [&] {
        if (!out || out->size != sizeof(QnSnapshot) || out->version != 1 || (!regions && capacity))
            throw std::invalid_argument("Invalid snapshot size/version or Region buffer.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        refreshRegions(*state);
        auto& app = state->application;
        const auto& preview = app.preview();
        *out = {};
        out->size = sizeof(*out); out->version = 1; out->revision = app.revision();
        out->region_count = static_cast<uint32_t>(state->regions.size());
        out->sample_count = static_cast<uint32_t>(app.centerline().visualization().samples.size());
        out->car_count = static_cast<uint32_t>(preview.trainDefinition().cars.size());
        out->playback = static_cast<uint32_t>(preview.playbackState());
        out->available = preview.isAvailable(); out->dirty = app.history().isDirty();
        out->can_undo = app.history().canUndo(); out->can_redo = app.history().canRedo();
        out->speed = preview.speedMetersPerSecond(); out->initial_speed = app.track().physicalSettings().initialSpeed;
        out->total_length = state->regions.empty() ? 0 : state->regions.back().end_station;
        out->physics_ms = preview.frameTelemetry().physicsMilliseconds;
        for (const auto& car : preview.trainDefinition().cars)
            out->train_mass += quantum::physics::totalCarMassKilograms(car.car, car.loadout);
        if (const auto* dynamics = preview.dynamicsState())
        {
            out->tick = dynamics->tick;
            out->station = dynamics->generalizedReferenceLocation.stationMeters;
        }
        if (knownRevision == out->revision) return;
        if (capacity < out->region_count) throw Failure(QN_BUFFER_TOO_SMALL, "Region buffer is too small.");
        std::copy(state->regions.begin(), state->regions.end(), regions);
    });
}
QnResult QN_CALL qn_command(QnSession handle, const QnCommand* command, QnError* error)
{
    return boundary(error, [&] {
        if (!command || command->type < QN_PLAY || command->type > QN_SET_REGION_LENGTH || !std::isfinite(command->value))
            throw std::invalid_argument("Invalid QUANTUM command.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        auto& app = state->application;
        if (command->type == QN_SET_REGION_LENGTH || command->type == QN_SET_INITIAL_SPEED)
            if (command->expected_revision != app.revision())
                throw Failure(QN_CONFLICT, "Document changed. Refresh the Inspector and try again.");
        switch (command->type)
        {
        case QN_PLAY: app.play(); break;
        case QN_PAUSE: app.pause(); break;
        case QN_RESET: app.reset(); break;
        case QN_SET_INITIAL_SPEED: app.setInitialSpeed(command->value); break;
        case QN_UNDO: app.restoreHistory(true); break;
        case QN_REDO: app.restoreHistory(false); break;
        case QN_SET_REGION_LENGTH: app.setRegionLength(command->region_index, command->value); break;
        }
    });
}
QnResult QN_CALL qn_authoring_snapshot(QnSession handle, QnAuthoringSnapshot* out,
    QnProfileSegment* profiles, uint32_t profileCapacity, QnDevice* devices, uint32_t deviceCapacity,
    uint64_t knownRevision, QnError* error)
{
    return boundary(error, [&] {
        if (!out || out->size != sizeof(*out) || out->version != 1
            || (!profiles && profileCapacity) || (!devices && deviceCapacity))
            throw std::invalid_argument("Invalid authoring snapshot size/version or buffer.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        refreshAuthoring(*state);
        *out = {sizeof(*out), 1, state->authoringRevision,
            static_cast<uint32_t>(state->profiles.size()), static_cast<uint32_t>(state->devices.size())};
        if (knownRevision == out->revision) return;
        if (profileCapacity < out->profile_count || deviceCapacity < out->device_count)
            throw Failure(QN_BUFFER_TOO_SMALL, "Authoring snapshot buffer is too small.");
        std::copy(state->profiles.begin(), state->profiles.end(), profiles);
        std::copy(state->devices.begin(), state->devices.end(), devices);
    });
}
QnResult QN_CALL qn_transition_types(QnTransitionType* types, uint32_t capacity, uint32_t* count, QnError* error)
{
    return boundary(error, [&] {
        if (!count || (!types && capacity)) throw std::invalid_argument("Invalid transition catalog output.");
        *count = static_cast<uint32_t>(quantum::editor::transitionTypePresets.size());
        if (capacity < *count) throw Failure(QN_BUFFER_TOO_SMALL, "Transition catalog buffer is too small.");
        for (uint32_t i = 0; i < *count; ++i)
        {
            types[i] = {}; const auto& preset = quantum::editor::transitionTypePresets[i];
            types[i].type = static_cast<uint32_t>(preset.type);
            copyLabel(types[i].name, sizeof(types[i].name), preset.displayName);
        }
    });
}
QnResult QN_CALL qn_profile_command(QnSession handle, const QnProfileCommand* command, QnError* error)
{
    return boundary(error, [&] {
        requireProfileCommand(command);
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        auto& app = state->application;
        if (app.revision() != command->expected_revision) throw Failure(QN_CONFLICT, "Document changed. Refresh the Timeline.");
        const quantum::editor::ProfileEdit edit{command->segment_id,
            static_cast<quantum::editor::ProfileEditOperation>(command->operation), command->value,
            static_cast<quantum::math::TransitionType>(command->transition_type)};
        if (command->owner_kind == 0)
            app.editRegionProfile(command->owner_id, static_cast<quantum::editor::ProfileChannel>(command->channel), edit);
        else app.editDeviceProfile(command->owner_id, edit);
    });
}
QnResult QN_CALL qn_device_acceleration(QnSession handle, uint64_t device, uint64_t revision,
    uint32_t custom, double acceleration, QnError* error)
{
    return boundary(error, [&] {
        if (custom > 1) throw std::invalid_argument("Unknown acceleration mode.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (state->application.revision() != revision) throw Failure(QN_CONFLICT, "Document changed. Refresh Track Devices.");
        state->application.setDeviceAcceleration(device, custom != 0, acceleration);
    });
}
QnResult QN_CALL qn_profile_drag_begin(QnSession handle, const QnProfileCommand* command, uint64_t* token, QnError* error)
{
    if (token) *token = 0;
    return boundary(error, [&] {
        requireProfileCommand(command);
        if (!token) throw std::invalid_argument("Missing drag token output.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        requireRevision(*state, command->expected_revision);
        if (state->application.transformPreview()) throw Failure(QN_BUSY, "Finish the authored transform before starting a Timeline gesture.");
        const quantum::editor::ProfileTarget target{command->owner_kind == 0
            ? quantum::editor::ProfileOwner::Region : quantum::editor::ProfileOwner::Device,
            command->owner_id, static_cast<quantum::editor::ProfileChannel>(command->channel)};
        const quantum::editor::ProfileEdit edit{command->segment_id,
            static_cast<quantum::editor::ProfileEditOperation>(command->operation)};
        state->dragToken = 0;
        state->dragToken = state->application.beginProfileDrag(target, edit);
        state->dragCommand = *command;
        *token = state->dragToken;
    });
}
QnResult QN_CALL qn_profile_drag_preview(QnSession handle, uint64_t token, double value, double snap,
    QnProfileSegment* profiles, uint32_t capacity, uint32_t* count, QnError* error)
{
    if (count) *count = 0;
    return boundary(error, [&] {
        if (!count || (!profiles && capacity)) throw std::invalid_argument("Invalid drag preview buffer.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        requireDrag(*state, token);
        const auto& profile = state->application.previewProfileDrag(token, value, snap);
        const auto& command = state->dragCommand;
        double domainScale = 1, valueScale = 1;
        uint32_t unit = 2;
        if (command.owner_kind == 0)
        {
            const auto& track = state->application.track();
            const bool force = quantum::coaster::isForceDrivenSection(track.section(command.owner_id));
            domainScale = track.physicalSettings().metersPerCoordinateUnit;
            const auto styles = force ? quantum::editor::forceDrivenRowStyles(domainScale) : quantum::editor::rateProfileRowStyles(domainScale);
            valueScale = styles[command.channel].authoredToDisplay;
            unit = force && command.channel > 0 ? 1 : 0;
        }
        std::vector<QnProfileSegment> sampled;
        appendProfileSamples(sampled, profile, command.owner_kind, command.owner_id, command.channel, unit, domainScale, valueScale);
        *count = static_cast<uint32_t>(sampled.size());
        if (capacity < *count) throw Failure(QN_BUFFER_TOO_SMALL, "Drag preview buffer is too small.");
        std::copy(sampled.begin(), sampled.end(), profiles);
    });
}
QnResult QN_CALL qn_profile_drag_finish(QnSession handle, uint64_t token, uint32_t commit, QnError* error)
{
    return boundary(error, [&] {
        if (commit > 1) throw std::invalid_argument("Unknown drag completion operation.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (commit)
        {
            requireDrag(*state, token);
            state->dragToken = 0;
            state->application.commitProfileDrag(token);
        }
        else
        {
            state->application.cancelProfileDrag(token);
            if (state->dragToken == token) state->dragToken = 0;
        }
    });
}
QnResult QN_CALL qn_device_create(QnSession handle, uint64_t revision, uint32_t kind, uint64_t* device, QnError* error)
{
    if (device) *device = 0;
    return boundary(error, [&] {
        if (!device || kind > 8) throw std::invalid_argument("Invalid device kind/output.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        *device = state->application.createDevice(static_cast<quantum::coaster::TrackDeviceKind>(kind));
    });
}
QnResult QN_CALL qn_device_delete(QnSession handle, uint64_t revision, uint64_t device, QnError* error)
{
    return boundary(error, [&] {
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        state->application.deleteDevice(device);
    });
}
QnResult QN_CALL qn_device_properties(QnSession handle, uint64_t revision, uint64_t device,
    uint32_t enabled, double start, double end, double force, QnError* error)
{
    return boundary(error, [&] {
        if (enabled > 1) throw std::invalid_argument("Unknown device enabled state.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        state->application.setDeviceProperties(device, enabled != 0, start, end, force);
    });
}
QnResult QN_CALL qn_select_device(QnSession handle, uint64_t revision, uint64_t device, QnError* error)
{
    return boundary(error, [&] {
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        if (device && std::none_of(state->application.track().trackDevices().devices.begin(),
                state->application.track().trackDevices().devices.end(), [device](const auto& d) { return d.id == device; }))
            throw std::invalid_argument("Unknown Track Device selection.");
        state->selectedDevice = device;
    });
}
QnResult QN_CALL qn_hardware_snapshot(QnSession handle, uint64_t revision,
    char* buffer, uint32_t capacity, uint32_t* required, QnError* error)
{
    return boundary(error, [&] {
        if (!required || (!buffer && capacity)) throw std::invalid_argument("Invalid hardware snapshot buffer.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        refreshAuthoring(*state);
        using nlohmann::json;
        if (std::ranges::none_of(state->application.track().trackDevices().infrastructure,
            [&](const auto& i) { return i.id == state->selectedInfrastructure; })) state->selectedInfrastructure = 0;
        json result = {{"revision", revision}, {"hardware", state->hardwareConfiguration},
            {"selectedInfrastructure",state->selectedInfrastructure}, {"selectedAccess",state->selectedDevice},
            {"nextInfrastructureId", state->application.track().trackDevices().nextInfrastructureId},
            {"infrastructure", json::parse(quantum::coaster::serializeTrackInfrastructure(state->application.track().trackDevices().infrastructure))}};
        for (auto& item : result["hardware"])
        {
            const auto id = item["id"].get<uint64_t>();
            const auto& runtime = state->application.preview().operations().hardware();
            const auto h = std::find_if(runtime.begin(), runtime.end(), [id](const auto& value) { return value.id == id; });
            if (h == runtime.end()) { item["state"] = "Preview unavailable"; continue; }
            item["state"] = h->state; item["interlockReason"] = h->interlockReason;
            item["runtimeEnabled"] = h->enabled;
            item["occupiedBogieCount"] = h->occupancy.bogies.size();
            std::size_t carCount = 0;
            json trainIds = json::array();
            for (const auto& train : h->occupancy.trains) { carCount += train.carCount; trainIds.push_back(train.id); }
            item["occupiedCarCount"] = carCount; item["trainIds"] = std::move(trainIds);
            item["entered"] = h->occupancy.entered; item["exited"] = h->occupancy.exited;
            item["commandedSpeed"] = h->commandedSpeedMetersPerSecond;
            item["commandedAcceleration"] = h->commandedAccelerationMetersPerSecondSquared;
            item["profileAcceleration"] = h->profileAccelerationMetersPerSecondSquared;
            item["targetSpeedLimited"] = h->targetSpeedLimited;
            item["direction"] = h->actualSpeedMetersPerSecond > 0 ? 1 : h->actualSpeedMetersPerSecond < 0 ? -1 : 0;
            item["actualSpeed"] = h->actualSpeedMetersPerSecond; item["appliedForce"] = h->appliedForceNewtons;
            item["aligned"] = h->aligned; item["stopped"] = h->stopped; item["movementPermitted"] = h->movementPermitted;
            if (const auto* station = std::get_if<quantum::physics::StationRuntime>(&h->mechanism))
            { item["dispatchRequested"] = station->dispatchRequested; item["dispatchPermitted"] = station->dispatchPermitted; }
            if (const auto* transfer = std::get_if<quantum::physics::TransferRuntime>(&h->mechanism))
            { item["alignment"] = transfer->alignment; item["targetAlignment"] = transfer->target; item["position"] = transfer->positionMeters;
              item["locked"] = transfer->phase == quantum::physics::TransferRuntime::Phase::Locked; }
            if (const auto* gate = std::get_if<quantum::physics::AccessRuntime>(&h->mechanism))
            { item["position"] = gate->fraction; item["locked"] = gate->locked; item["interlocked"] = gate->interlocked || gate->protectedByOperation; }
        }
        const auto text = result.dump();
        if (text.size() >= std::numeric_limits<uint32_t>::max()) throw std::invalid_argument("Hardware snapshot is too large.");
        *required = static_cast<uint32_t>(text.size() + 1);
        if (capacity < *required) throw Failure(QN_BUFFER_TOO_SMALL, "Hardware snapshot buffer is too small.");
        std::memcpy(buffer, text.c_str(), *required);
    });
}
QnResult QN_CALL qn_hardware_configuration(QnSession handle, uint64_t revision, uint64_t device, const char* text, QnError* error)
{
    return boundary(error, [&] {
        if (!text) throw std::invalid_argument("Hardware configuration is required.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        const auto& devices = state->application.track().trackDevices().devices;
        auto found = std::find_if(devices.begin(), devices.end(), [device](const auto& d) { return d.id == device; });
        if (found == devices.end()) throw std::invalid_argument("Unknown hardware ID.");
        quantum::coaster::HardwareConfiguration config;
        try { config = quantum::coaster::deserializeRideHardwareConfiguration(found->kind, text); }
        catch (const std::exception& e) { throw std::invalid_argument(e.what()); }
        state->application.setHardwareConfiguration(device, config);
    });
}
QnResult QN_CALL qn_hardware_command(QnSession handle, uint64_t revision, uint64_t device, uint32_t command, double value, uint64_t target, QnError* error)
{
    return boundary(error, [&] {
        if (command > static_cast<uint32_t>(quantum::physics::HardwareCommand::SetAccessInterlocked))
            throw std::invalid_argument("Unknown runtime hardware command.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        state->application.preview().hardwareCommand(device, static_cast<quantum::physics::HardwareCommand>(command), value, target);
    });
}
QnResult QN_CALL qn_infrastructure_configuration(QnSession handle, uint64_t revision, const char* text, uint64_t nextId, QnError* error)
{
    return boundary(error, [&] {
        if (!text) throw std::invalid_argument("Infrastructure configuration is required.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        std::vector<quantum::coaster::TrackInfrastructure> items;
        try { items = quantum::coaster::deserializeTrackInfrastructure(text); }
        catch (const std::exception& e) { throw std::invalid_argument(e.what()); }
        state->application.setInfrastructure(items, nextId);
    });
}
QnResult QN_CALL qn_update(QnSession handle, double seconds, QnError* error)
{
    return boundary(error, [&] {
        if (!std::isfinite(seconds) || seconds < 0 || seconds > .25)
            throw std::invalid_argument("Update must be between zero and 0.25 seconds.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        state->application.preview().beginFrameTelemetry();
        state->application.update(seconds);
    });
}
QnResult QN_CALL qn_support_snapshot(QnSession handle, QnSupportSnapshot* out,
    QnSupportStructure* structures, uint32_t structureCapacity,
    QnSupportNode* nodes, uint32_t nodeCapacity, QnSupportMember* members,
    uint32_t memberCapacity, uint64_t knownRevision, QnError* error)
{
    return boundary(error, [&] {
        if (!out || out->size != sizeof(*out) || out->version != 1
            || (!structures && structureCapacity) || (!nodes && nodeCapacity) || (!members && memberCapacity))
            throw std::invalid_argument("Invalid support snapshot size/version or buffers.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        refreshSupports(*state);
        *out = {sizeof(*out), 1, state->application.revision(), state->supportSelectionRevision,
            state->supportRevision, static_cast<uint32_t>(state->supportStructures.size()),
            static_cast<uint32_t>(state->supportNodes.size()), static_cast<uint32_t>(state->supportMembers.size()),
            state->supportWorkspace};
        if (state->selectedSupport)
        {
            out->selection_kind = static_cast<uint32_t>(state->selectedSupport->kind) + 1;
            out->selection_structure = state->selectedSupport->structureId;
            out->selection_element = state->selectedSupport->elementId;
        }
        if (knownRevision == out->revision) return;
        if (structureCapacity < out->structure_count || nodeCapacity < out->node_count || memberCapacity < out->member_count)
            throw Failure(QN_BUFFER_TOO_SMALL, "Support snapshot buffer is too small.");
        std::copy(state->supportStructures.begin(), state->supportStructures.end(), structures);
        std::copy(state->supportNodes.begin(), state->supportNodes.end(), nodes);
        std::copy(state->supportMembers.begin(), state->supportMembers.end(), members);
    });
}
QnResult QN_CALL qn_support_command(QnSession handle, const QnSupportCommand* command,
    const char* text, uint32_t* created, QnError* error)
{
    if (created) *created = 0;
    return boundary(error, [&] {
        if (!command || !created || command->operation > QN_SUPPORT_FOUNDATION_SHAPE
            || command->shape > (command->operation == QN_SUPPORT_SET_PROFILE || command->operation == QN_SUPPORT_CREATE_MEMBER ? 3u : 5u) || command->member_end > 1 || command->treatment > 8)
            throw std::invalid_argument("Invalid support command or enum value.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        requireRevision(*state, command->expected_revision);
        using namespace quantum::editor;
        SupportEdit request{static_cast<SupportOperation>(command->operation)};
        request.structureId = command->structure_id; request.elementId = command->element_id;
        request.startNodeId = command->start_node; request.endNodeId = command->end_node;
        const auto scale = state->application.track().physicalSettings().metersPerCoordinateUnit;
        request.position = glm::dvec3{command->x, command->y, command->z} / scale;
        request.attachment = {command->x / scale, command->y / scale, command->z / scale};
        request.profile = {static_cast<quantum::coaster::SupportMemberProfileShape>(command->shape),
            {command->width / scale, command->height / scale}, command->wall / scale};
        request.memberEnd = static_cast<quantum::coaster::SupportMemberEnd>(command->member_end);
        request.treatment = static_cast<quantum::coaster::SupportMemberEndTreatment>(command->treatment);
        request.text = text ? text : "";
        request.style = static_cast<quantum::coaster::SupportStyleFamily>(command->shape);
        try
        {
            if (request.operation == SupportOperation::CreateMember && !request.text.empty())
            {
                const auto j = nlohmann::json::parse(request.text);
                if (!j.is_object() || j.size() != 1) throw std::invalid_argument("Member options require useStyleProfile only.");
                request.useStyleProfile = j.at("useStyleProfile").get<bool>();
            }
            if (request.operation == SupportOperation::SetStructureSurface || request.operation == SupportOperation::SetMemberSurface)
            {
                if (!request.text.empty()) request.surface = quantum::coaster::deserializeSupportSurface(request.text);
            }
            if (request.operation == SupportOperation::SetFoundationShape)
                request.foundation = {static_cast<quantum::coaster::SupportFoundationKind>(command->shape),
                    {command->width/scale,command->height/scale,command->wall/scale}};
            if (request.operation == SupportOperation::GenerateFrame)
            {
                const auto j = nlohmann::json::parse(request.text);
                request.frame.station = j.at("stationMeters").get<double>()/scale;
                request.frame.width = j.at("widthMeters").get<double>()/scale;
                request.frame.foundationElevation = j.at("foundationElevationMeters").get<double>()/scale;
                request.frame.verticalOffset = j.at("verticalOffsetMeters").get<double>()/scale;
                const auto profile = [&](const char* key)
                {
                    const auto& p = j.at(key);
                    const auto shape = p.at("shape").get<unsigned>();
                    if (shape > 3) throw std::invalid_argument("Unknown frame profile shape.");
                    return quantum::coaster::SupportMemberProfile{static_cast<quantum::coaster::SupportMemberProfileShape>(shape),
                        {p.at("width").get<double>()/scale,p.at("height").get<double>()/scale},p.at("wall").get<double>()/scale};
                };
                request.frame.post = profile("post"); request.frame.beam = profile("beam"); request.frame.brace = profile("brace");
                const auto& a = j.at("foundationSizeMeters");
                if (!a.is_array() || a.size()!=3) throw std::invalid_argument("Foundation size needs three dimensions.");
                request.frame.foundationSize = glm::dvec3{a[0].get<double>(),a[1].get<double>(),a[2].get<double>()}/scale;
            }
        }
        catch (const nlohmann::json::exception& failure)
        {
            throw std::invalid_argument(std::string("Invalid support command JSON: ") + failure.what());
        }

        *created = state->application.editSupport(request);
    });
}
QnResult QN_CALL qn_select_support(QnSession handle, uint64_t revision,
    uint32_t kind, uint32_t structure, uint32_t element, QnError* error)
{
    return boundary(error, [&] {
        if (kind > 3 || (!kind && (structure || element)) || (kind == 1 && element))
            throw std::invalid_argument("Invalid support selection kind/identity.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        std::optional<quantum::editor::SupportSelection> selection;
        if (kind)
        {
            selection = {structure, static_cast<quantum::editor::SupportSelectionKind>(kind - 1), element};
            if (!quantum::editor::supportSelectionExists(state->application.track().supports(), *selection))
                throw std::invalid_argument("Unknown support selection.");
        }
        selectSupport(*state, selection);
    });
}
QnResult QN_CALL qn_select_infrastructure(QnSession handle, uint64_t revision, uint64_t id, QnError* error)
{
    return boundary(error,[&] {
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state,revision);
        const auto& items = state->application.track().trackDevices().infrastructure;
        if (id && std::ranges::none_of(items,[id](const auto& item) { return item.id == id; }))
            throw std::invalid_argument("Unknown infrastructure ID.");
        state->selectedInfrastructure = id;
    });
}
QnResult QN_CALL qn_support_style_snapshot(QnSession handle, uint64_t revision,
    char* buffer, uint32_t capacity, uint32_t* required, QnError* error)
{
    return boundary(error, [&] {
        if (!required || (!buffer && capacity)) throw std::invalid_argument("Invalid support style buffer.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        using namespace quantum::coaster;
        using nlohmann::json;
        json structures = json::array();
        const double scale = state->application.track().physicalSettings().metersPerCoordinateUnit;
        const auto profile = [scale](const SupportMemberProfile& p) -> json {
            return {{"shape",static_cast<unsigned>(p.shape)}, {"width",p.outerDimensions.x*scale},
                {"height",p.outerDimensions.y*scale}, {"wall",p.wallThickness*scale}}; };
        for (const auto& s : state->application.track().supports().structures)
        {
            json members = json::array(), foundations = json::array();
            for (const auto& m : s.members)
                members.push_back({{"id",m.id}, {"useStyleProfile",m.useStyleProfile},
                    {"useFrameProfile",m.useFrameProfile},
                    {"surface",m.surface ? json::parse(serializeSupportSurface(*m.surface)) : json(nullptr)},
                    {"resolved",json::parse(serializeSupportSurface(resolveSupportSurface(s,m,scale)))}});
            for (const auto& n : s.nodes) if (n.foundation)
                foundations.push_back({{"id",n.id}, {"kind",static_cast<unsigned>(n.foundation->kind)},
                    {"dimensions",{n.foundation->dimensions.x*scale,n.foundation->dimensions.y*scale,n.foundation->dimensions.z*scale}}});
            json recipe = nullptr;
            if (s.generatedFrame)
            {
                const auto& r = *s.generatedFrame;
                recipe = {{"stationMeters",r.station*scale}, {"widthMeters",r.width*scale},
                    {"foundationElevationMeters",r.foundationElevation*scale}, {"verticalOffsetMeters",r.verticalOffset*scale},
                    {"post",profile(r.post)}, {"beam",profile(r.beam)}, {"brace",profile(r.brace)},
                    {"foundationSizeMeters",{r.foundationSize.x*scale,r.foundationSize.y*scale,r.foundationSize.z*scale}}};
            }
            structures.push_back({{"id",s.id}, {"family",static_cast<unsigned>(s.style)},
                {"surface",s.surface ? json::parse(serializeSupportSurface(*s.surface)) : json(nullptr)},
                {"resolved",json::parse(serializeSupportSurface(resolveSupportSurface(s,SupportMember{},scale)))},
                {"frame",recipe}, {"members",members}, {"foundations",foundations}});
        }
        const auto text = structures.dump();
        if (text.size() >= std::numeric_limits<uint32_t>::max()) throw std::invalid_argument("Support style snapshot is too large.");
        *required = static_cast<uint32_t>(text.size()+1);
        if (capacity < *required) throw Failure(QN_BUFFER_TOO_SMALL,"Support style buffer is too small.");
        std::memcpy(buffer,text.c_str(),*required);
    });
}
QnResult QN_CALL qn_support_workspace(QnSession handle, uint32_t active, QnError* error)
{
    return boundary(error, [&] {
        if (active > 1) throw std::invalid_argument("Invalid support workspace state.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (state->supportWorkspace != (active != 0))
        {
            state->application.cancelTransform();
            ++state->placementSelectionRevision;
            state->supportWorkspace = active != 0;
        }
    });
}
QnResult QN_CALL qn_viewport_open(QnSession handle, uint32_t platform, uint64_t window, QnError* error)
{
    return boundary(error, [&] {
        if (platform > 2 || ((platform == 0) != (window == 0))) throw std::invalid_argument("Unsupported viewport platform/window.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        std::lock_guard viewLock(viewportMutex);
        if (viewportActive) throw Failure(QN_BUSY, "One editor viewport is supported per process.");
        auto viewport = std::make_unique<NativeViewport>();
        viewport->open(platform, window, state->application.centerline().visualization());
        state->viewport = std::move(viewport);
        state->viewportRevision = 0;
        state->viewportGeometryGeneration = 0;
        state->viewportFramedEpoch = 0;
        viewportActive = true;
    });
}
QnResult QN_CALL qn_viewport_frame(QnSession handle, uint32_t width, uint32_t height, QnError* error)
{
    bool busy = false;
    const auto result = boundary(error, [&] {
        if (width > 16384 || height > 16384) throw std::invalid_argument("Invalid viewport physical size.");
        auto state = session(handle);
        std::optional<quantum::editor::CenterlineVisualization> geometry;
        std::optional<quantum::editor::SupportVisualization> supports;
        std::optional<quantum::coaster::GroundAppearance> ground;
        NativeViewport* viewport;
        uint64_t revision;
        uint64_t geometryGeneration;
        uint64_t documentEpoch;
        bool frameTrack;
        const auto acceptPick = [&](Session& state, NativeViewport& viewport)
        {
            if (const auto pick = viewport.takeInfrastructurePick(); pick && pick->revision == state.application.revision())
            {
                if (pick->access) state.selectedDevice = pick->id;
                else state.selectedInfrastructure = pick->id;
            }
            if (const auto pick = viewport.takeSupportPick(); pick && state.supportWorkspace
                && pick->revision == state.application.revision() && pick->selectionRevision == state.supportSelectionRevision
                && (!pick->selection || quantum::editor::supportSelectionExists(state.application.track().supports(), *pick->selection)))
                selectSupport(state, pick->selection);
            if (const auto pick = viewport.takeTrackStartPick(); pick && !state.supportWorkspace
                && pick->revision == state.application.revision() && pick->selectionRevision == state.placementSelectionRevision)
                selectTrackStart(state, true, state.transformMode);
        };
        {
            // A document rebuild/file operation may take longer than a frame.
            // Retain the last presented image instead of blocking UI input.
            std::unique_lock lock(state->mutex, std::try_to_lock);
            if (!lock.owns_lock()) { busy = true; return; }
            requireOpen(*state);
            if (!state->viewport) throw Failure(QN_UNAVAILABLE, "Viewport is not open.");
            viewport = state->viewport.get(); viewport->requireOwnerThread();
            acceptPick(*state, *viewport);
            auto& app = state->application;
            refreshSupports(*state);
            revision = app.revision();
            geometryGeneration = app.centerline().generation();
            documentEpoch = state->documentEpoch;
            // Spatial authoring edits and Undo/Redo preserve the user's view.
            // File replacement frames once; other framing is explicit.
            frameTrack = state->viewportFramedEpoch != documentEpoch;
            viewport->capturePlacement(revision, state->placementSelectionRevision, app.transformPresentationRevision(),
                !state->supportWorkspace, state->selectedTrackStart, state->transformMode,
                app.track().startPose(), app.transformPreview(), app.track().physicalSettings().metersPerCoordinateUnit,
                app.centerline().visualization());
            viewport->captureTrain(revision, app.preview().vertexGeneration(), app.preview().vertices(), app.preview().meshInstances());
            viewport->captureDevice(revision, state->supportWorkspace ? 0 : state->selectedDevice, app.track(), app.centerline().visualization(), app.preview().operations(), state->selectedInfrastructure);
            viewport->captureSupports(revision, state->supportSelectionRevision, state->supportWorkspace,
                state->selectedSupport, app.track().supports(), app.supports());
            if (state->viewportGeometryGeneration != geometryGeneration)
            {
                geometry = app.centerline().visualization();
                ground = app.track().ground();
            }
            if (state->viewportRevision != revision)
            {
                supports = app.supports();
            }
        }
        // A session reference keeps the state alive. Destroy rejects an open
        // viewport; viewport close/frame can only occur on this same thread.
        const bool presented = viewport->frame(width, height, geometry ? &*geometry : nullptr,
            supports ? &*supports : nullptr, ground ? &*ground : nullptr, frameTrack);
        if (presented)
        {
            state->viewportRevision = revision;
            state->viewportGeometryGeneration = geometryGeneration;
            state->viewportFramedEpoch = documentEpoch;
            // A click is transient selection only. A long worker edit must not
            // block presentation; pending input can be consumed next frame.
            std::unique_lock lock(state->mutex, std::try_to_lock);
            if (lock.owns_lock()) acceptPick(*state, *viewport);
        }
    });
    return busy ? QN_BUSY : result;
}
QnResult QN_CALL qn_viewport_shortcuts(QnSession handle, uint32_t* shortcuts, QnError* error)
{
    bool busy = false;
    const auto result = boundary(error, [&] {
        if (!shortcuts) throw std::invalid_argument("Missing shortcut output.");
        *shortcuts = 0;
        auto state = session(handle);
        std::unique_lock lock(state->mutex, std::try_to_lock);
        if (!lock.owns_lock()) { busy = true; return; }
        requireOpen(*state);
        if (!state->viewport) throw Failure(QN_UNAVAILABLE, "Viewport is not open.");
        state->viewport->requireOwnerThread();
        *shortcuts = state->viewport->takeShortcuts();
    });
    return busy ? QN_BUSY : result;
}
QnResult QN_CALL qn_viewport_close(QnSession handle, QnError* error)
{
    return boundary(error, [&] {
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (!state->viewport) return;
        state->viewport->requireOwnerThread();
        state->application.cancelTransform();
        std::lock_guard viewLock(viewportMutex);
        state->viewport.reset(); viewportActive = false;
    });
}
QnResult QN_CALL qn_viewport_input(QnSession handle, uint32_t action, double x, double y, double z, double seconds, QnError* error)
{
    return boundary(error, [&] {
        if (action < 1 || action > 13 || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(seconds))
            throw std::invalid_argument("Invalid viewport camera input.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (!state->viewport) throw Failure(QN_UNAVAILABLE, "Viewport is not open.");
        state->viewport->requireOwnerThread();
        if (action == 9)
        {
            auto& app = state->application;
            state->viewport->capturePlacement(app.revision(), state->placementSelectionRevision, app.transformPresentationRevision(),
                !state->supportWorkspace, state->selectedTrackStart, state->transformMode,
                app.track().startPose(), app.transformPreview(), app.track().physicalSettings().metersPerCoordinateUnit,
                app.centerline().visualization());
        }
        state->viewport->input(action, x, y, z, seconds);
    });
}
QnResult QN_CALL qn_viewport_presentation(QnSession handle, uint32_t mode, QnError* error)
{
    return boundary(error, [&] {
        if (mode > 3) throw std::invalid_argument("Unknown viewport presentation mode.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (!state->viewport) throw Failure(QN_UNAVAILABLE, "Viewport is not open.");
        state->viewport->setPresentation(mode);
    });
}

QnResult QN_CALL qn_placement_snapshot(QnSession handle, QnPlacementSnapshot* out, QnError* error)
{
    return boundary(error, [&] {
        if (!out || out->size != sizeof(*out) || out->version != 1)
            throw std::invalid_argument("Placement snapshot size/version mismatch.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        auto& app = state->application;
        const auto scale = app.track().physicalSettings().metersPerCoordinateUnit;
        const auto copyPose = [scale](const quantum::coaster::AuthoredStartPose& pose) -> QnPose {
            return {pose.position.x * scale, pose.position.y * scale, pose.position.z * scale,
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
        };
        const auto preview = app.transformPreview();
        const auto angles = glm::degrees(quantum::editor::startPoseAngles(app.track().startPose()));
        *out = {sizeof(*out), 1, app.revision(), state->placementSelectionRevision,
            app.revision() + app.transformPresentationRevision() + state->placementSelectionRevision,
            state->selectedTrackStart, static_cast<uint32_t>(state->transformMode), !state->supportWorkspace,
            preview.has_value(), copyPose(app.track().startPose()), copyPose(preview.value_or(app.track().startPose())),
            angles.x, angles.y, angles.z};
    });
}
QnResult QN_CALL qn_placement_edit(QnSession handle, uint64_t revision,
    uint32_t operation, double x, double y, double z, QnError* error)
{
    return boundary(error, [&] {
        if (operation > 1) throw std::invalid_argument("Unknown Track Start numeric edit.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        if (operation == 0) state->application.setTrackStartPosition({x, y, z});
        else state->application.setTrackStartAngles({x, y, z});
    });
}
QnResult QN_CALL qn_track_start_select(QnSession handle, uint64_t revision, uint32_t selected, uint32_t mode, QnError* error)
{
    return boundary(error, [&] {
        if (selected > 1 || mode > 2) throw std::invalid_argument("Unknown Track Start selection or tool.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, revision);
        selectTrackStart(*state, selected != 0, static_cast<quantum::editor::StartPoseTransformMode>(mode));
    });
}
QnResult QN_CALL qn_transform_begin(QnSession handle, const QnTransformCommand* command, uint64_t* token, QnError* error)
{
    if (token) *token = 0;
    return boundary(error, [&] {
        if (!command || !token || command->target != 0 || command->mode < 1 || command->mode > 2 || command->axis > 2)
            throw std::invalid_argument("Invalid authored transform request.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state); requireRevision(*state, command->expected_revision);
        if (command->selection_revision != state->placementSelectionRevision || state->supportWorkspace || !state->selectedTrackStart)
            throw Failure(QN_CONFLICT, "Selection changed. Transform canceled.");
        if (state->dragToken) throw Failure(QN_BUSY, "Finish the Timeline gesture before starting a transform.");
        *token = state->application.beginTransform(quantum::editor::TransformTarget::TrackStart,
            static_cast<quantum::editor::StartPoseTransformMode>(command->mode),
            static_cast<quantum::editor::StartPoseTransformAxis>(command->axis));
    });
}
QnResult QN_CALL qn_transform_preview(QnSession handle, uint64_t token, double amount, QnError* error)
{
    return boundary(error, [&] {
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        try { requireRevision(*state, state->application.transformRevision(token)); }
        catch (const Failure&) { state->application.cancelTransform(); throw; }
        state->application.previewTransform(token, amount);
    });
}
QnResult QN_CALL qn_transform_finish(QnSession handle, uint64_t token, uint32_t commit, QnError* error)
{
    return boundary(error, [&] {
        if (commit > 1) throw std::invalid_argument("Unknown transform finish operation.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (commit)
        {
            try { requireRevision(*state, state->application.transformRevision(token)); }
            catch (const Failure&) { state->application.cancelTransform(); throw; }
        }
        state->application.finishTransform(token, commit != 0);
    });
}
QnResult QN_CALL qn_viewport_transform_intent(QnSession handle, QnTransformIntent* intent, QnError* error)
{
    bool busy = false;
    const auto result = boundary(error, [&] {
        if (!intent) throw std::invalid_argument("Missing transform intent output.");
        *intent = {};
        auto state = session(handle); std::unique_lock lock(state->mutex, std::try_to_lock);
        if (!lock.owns_lock()) { busy = true; return; }
        requireOpen(*state);
        if (!state->viewport) throw Failure(QN_UNAVAILABLE, "Viewport is not open.");
        state->viewport->requireOwnerThread(); *intent = state->viewport->takeTransformIntent();
    });
    return busy ? QN_BUSY : result;
}
QnResult QN_CALL qn_viewport_cancel_transform(QnSession handle, QnError* error)
{
    return boundary(error, [&] {
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (!state->viewport) return;
        state->viewport->requireOwnerThread(); state->viewport->cancelTransformInput();
    });
}
