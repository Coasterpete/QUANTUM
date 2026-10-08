#include "QuantumNative.h"
#include "NativeViewport.hpp"
#include <quantum/editor/ApplicationService.hpp>
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
        std::vector<QnRegion> regions;
        std::uint64_t authoringRevision = 0;
        std::vector<QnProfileSegment> profiles;
        std::vector<QnDevice> devices;
        QnProfileCommand dragCommand{};
        std::uint64_t dragToken = 0;
        std::uint64_t selectedDevice = 0;
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
            QnDevice row{}; row.id = device.id; row.kind = static_cast<uint32_t>(device.kind);
            row.enabled = device.enabled; row.custom = device.accelerationProfile.has_value();
            row.start_station = device.startStationMeters; row.end_station = device.endStationMeters;
            row.acceleration = device.targetAccelerationMetersPerSecondSquared; row.maximum_force = device.maximumForceNewtons;
            copyLabel(row.name, sizeof(row.name), device.name); devices.push_back(row);
            if (device.accelerationProfile) appendProfileSamples(profiles, *device.accelerationProfile, 1, device.id, 3, 2, 1, 1);
        }
        state.profiles = std::move(profiles); state.devices = std::move(devices);
        state.authoringRevision = app.revision();
    }
    void requireRevision(const Session& state, uint64_t revision)
    {
        if (state.application.revision() != revision)
            throw Failure(QN_CONFLICT, "Document changed. Refresh the Timeline and try again.");
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
uint32_t QN_CALL qn_abi_version() { return 2; }
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
    });
}
QnResult QN_CALL qn_open(QnSession handle, const char* path, QnError* error)
{
    return boundary(error, [&] {
        const auto input = utf8Path(path);
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        state->application.open(input);
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
        if (!device || kind > 1) throw std::invalid_argument("Invalid device kind/output.");
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
        {
            // A document rebuild/file operation may take longer than a frame.
            // Retain the last presented image instead of blocking UI input.
            std::unique_lock lock(state->mutex, std::try_to_lock);
            if (!lock.owns_lock()) { busy = true; return; }
            requireOpen(*state);
            if (!state->viewport) throw Failure(QN_UNAVAILABLE, "Viewport is not open.");
            viewport = state->viewport.get(); viewport->requireOwnerThread();
            auto& app = state->application;
            revision = app.revision();
            viewport->captureTrain(revision, app.preview().vertexGeneration(), app.preview().vertices(), app.preview().meshInstances());
            viewport->captureDevice(revision, state->selectedDevice, app.track(), app.centerline().visualization());
            if (state->viewportRevision != revision)
            {
                geometry = app.centerline().visualization();
                supports = app.supports();
                ground = app.track().ground();
            }
        }
        // A session reference keeps the state alive. Destroy rejects an open
        // viewport; viewport close/frame can only occur on this same thread.
        const bool presented = viewport->frame(width, height, geometry ? &*geometry : nullptr,
            supports ? &*supports : nullptr, ground ? &*ground : nullptr);
        if (geometry && presented)
        {
            state->viewportRevision = revision;
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
        std::lock_guard viewLock(viewportMutex);
        state->viewport.reset(); viewportActive = false;
    });
}
QnResult QN_CALL qn_viewport_input(QnSession handle, uint32_t action, double x, double y, double z, double seconds, QnError* error)
{
    return boundary(error, [&] {
        if (action < 1 || action > 6 || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(seconds))
            throw std::invalid_argument("Invalid viewport camera input.");
        auto state = session(handle); std::lock_guard lock(state->mutex); requireOpen(*state);
        if (!state->viewport) throw Failure(QN_UNAVAILABLE, "Viewport is not open.");
        state->viewport->requireOwnerThread(); state->viewport->input(action, x, y, z, seconds);
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
