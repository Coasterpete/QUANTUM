#include <quantum/editor/ApplicationService.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/RiderLoadDiagnostics.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <atomic>
#include <chrono>

namespace quantum::editor
{
    namespace
    {
        std::string displayPath(const std::filesystem::path& path)
        {
            const auto utf8 = path.u8string();
            return {utf8.begin(), utf8.end()};
        }

        void applyProfileEdit(coaster::ChannelProfile& profile, ProfileEdit edit,
            double coordinateScale, double displayToAuthored)
        {
            if (!std::isfinite(edit.value)) throw std::invalid_argument("Profile value must be finite.");
            using Operation = ProfileEditOperation;
            using Boundary = coaster::ProfileBoundary;
            switch (edit.operation)
            {
            case Operation::BeginValue: case Operation::EndValue:
                coaster::setChannelSegmentValue(profile, edit.segmentId,
                    edit.operation == Operation::BeginValue ? Boundary::Begin : Boundary::End,
                    edit.value * displayToAuthored); break;
            case Operation::TransitionType:
            {
                auto* transition = coaster::findChannelSegmentTransition(profile, edit.segmentId);
                if (!transition || !trySetTransitionTypePreset(*transition, edit.transitionType))
                    throw std::invalid_argument("Unknown profile segment or transition type.");
                break;
            }
            case Operation::Split:
                static_cast<void>(coaster::splitChannelSegment(profile, edit.segmentId, edit.value / coordinateScale)); break;
            case Operation::Remove:
                static_cast<void>(coaster::removeChannelSegment(profile, edit.segmentId)); break;
            case Operation::MoveBegin: case Operation::MoveEnd:
                coaster::moveChannelSegmentBoundary(profile, edit.segmentId,
                    edit.operation == Operation::MoveBegin ? Boundary::Begin : Boundary::End,
                    edit.value / coordinateScale); break;
            default: throw std::invalid_argument("Unknown profile edit operation.");
            }
        }

        coaster::TrackDevice deviceById(const coaster::AuthoredTrack& track, coaster::TrackDeviceId id)
        {
            for (const auto& device : track.trackDevices().devices)
                if (device.id == id) return device;
            throw std::invalid_argument("Unknown Track Device.");
        }
    }
#ifdef _WIN32
    bool replaceDocumentFile(const std::filesystem::path&, const std::filesystem::path&, std::string&);
#endif
    ApplicationService::ApplicationService(coaster::AuthoredTrack track)
        : ApplicationService(prepare(std::move(track)))
    {
    }

    ApplicationService::ApplicationService(PreparedDocument prepared)
        : track_(std::move(prepared.track))
    {
        history_.reset(track_);
        centerline_.setTrackStyle(track_.trackStyle());
        centerline_.replace(std::move(prepared.centerline));
        supports_ = std::move(prepared.supports);
    }

    PreparedDocument ApplicationService::prepare(coaster::AuthoredTrack track,
        DocumentPreparationTelemetry* telemetry)
    {
        using Clock = std::chrono::steady_clock;
        auto start = telemetry ? Clock::now() : Clock::time_point{};
        auto centerline = createCenterlineVisualization(track, track.trackStyle());
        const auto finish = [&](double& milliseconds)
        {
            const auto now = Clock::now();
            milliseconds = std::chrono::duration<double, std::milli>(now - start).count();
            start = now;
        };
        if (telemetry) finish(telemetry->centerlineMilliseconds);
        auto loads = evaluateRiderLoadDiagnostics(track);
        if (telemetry) finish(telemetry->riderLoadsMilliseconds);
        auto supports = createSupportVisualization(track, centerline.samples);
        if (telemetry) finish(telemetry->supportsMilliseconds);
        AuthoredTrackEditTransaction transaction{track};
        transaction.requireAcceptableRiderLoads(loads);
        return {std::move(track), std::move(centerline), std::move(supports), std::move(loads)};
    }

    std::expected<PreparedDocument, std::string>
    ApplicationService::prepareFile(const std::filesystem::path& path)
    {
        try
        {
            std::ifstream input(path, std::ios::binary);
            if (!input) return std::unexpected("Could not open document: " + displayPath(path));
            const std::string json{std::istreambuf_iterator<char>(input), {}};
            if (input.bad()) return std::unexpected("Could not read document: " + displayPath(path));
            auto track = coaster::deserializeCoasterDocument(json);
            if (!track) return std::unexpected(track.error());
            return prepare(std::move(*track));
        }
        catch (const std::exception& error) { return std::unexpected(std::string{error.what()}); }
    }

    void ApplicationService::resizeRegion(coaster::AuthoredTrack& candidate,
        std::size_t index, double coordinateLength)
    {
        coaster::setSectionLength(candidate.section(index), coordinateLength);
    }

    void ApplicationService::replace(PreparedDocument prepared)
    {
        centerline_.setTrackStyle(prepared.track.trackStyle());
        track_ = std::move(prepared.track);
        centerline_.replace(std::move(prepared.centerline));
        supports_ = std::move(prepared.supports);
        ++revision_;
        static_cast<void>(rebuildPreview());
    }

    void ApplicationService::newDocument(const Publish& publish)
    {
        auto prepared = prepare(coaster::createNewDocument());
        DocumentHistory baseline;
        baseline.reset(prepared.track);
        if (publish) publish(prepared);
        replace(std::move(prepared));
        history_ = std::move(baseline);
        fileState_.newDocument();
    }

    void ApplicationService::open(const std::filesystem::path& path, const Publish& publish)
    {
        auto prepared = prepareFile(path);
        if (!prepared) throw std::invalid_argument(prepared.error());
        openPrepared(std::move(*prepared), path, publish);
    }

    void ApplicationService::openPrepared(PreparedDocument prepared,
        const std::filesystem::path& path, const Publish& publish)
    {
        DocumentHistory baseline;
        baseline.reset(prepared.track);
        DocumentState fileState;
        fileState.setOpenDocument(path);
        if (publish) publish(prepared);
        replace(std::move(prepared));
        history_ = std::move(baseline);
        fileState_ = std::move(fileState);
    }

    void ApplicationService::save(const std::filesystem::path& path)
    {
        if (path.empty()) throw std::invalid_argument("A document destination is required.");
        const auto json = coaster::serializeCoasterDocument(track_);
        // Stage beside the destination so failed writes preserve an existing
        // valid file. The rename is on the same filesystem.
        auto temporary = path;
        static std::atomic<std::uint64_t> sequence{0};
        temporary += ".saving-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count())
            + "-" + std::to_string(sequence.fetch_add(1));
        if (std::filesystem::exists(temporary))
            throw std::runtime_error("Temporary document path already exists: " + displayPath(temporary));
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) throw std::runtime_error("Could not create document: " + displayPath(temporary));
        output.write(json.data(), static_cast<std::streamsize>(json.size()));
        output.close();
        if (!output)
        {
            std::error_code ignored;
            std::filesystem::remove(temporary, ignored);
            throw std::runtime_error("Could not write document: " + displayPath(path));
        }
        std::error_code error;
#ifdef _WIN32
        // std::filesystem::rename cannot replace an existing file on Windows.
        // The platform replacement lives in the implementation, never the ABI.
        std::string message;
        if (!replaceDocumentFile(temporary, path, message))
        {
            std::filesystem::remove(temporary, error);
            throw std::runtime_error(message);
        }
#else
        std::filesystem::rename(temporary, path, error);
        if (error)
        {
            const auto message = error.message();
            std::filesystem::remove(temporary, error);
            throw std::runtime_error("Could not replace document: " + message);
        }
#endif
        fileState_.setCurrentPath(path);
        history_.markSaved();
        synchronizeDirtyState();
    }

    void ApplicationService::edit(AuthoredTrackEditTransaction transaction, const Publish& publish)
    {
        auto prepared = prepare(transaction.candidate());
        if (publish) publish(prepared);
        commitPreparedEdit(transaction, std::move(prepared));
        static_cast<void>(rebuildPreview());
    }

    void ApplicationService::commitPreparedEdit(AuthoredTrackEditTransaction& transaction,
        PreparedDocument prepared, bool continuous)
    {
        auto history = history_;
        history.record(prepared.track, continuous);
        centerline_.setTrackStyle(prepared.track.trackStyle());
        transaction.commit(track_);
        centerline_.replace(std::move(prepared.centerline));
        supports_ = std::move(prepared.supports);
        history_ = std::move(history);
        ++revision_;
        synchronizeDirtyState();
        // Geometry generation invalidates the preview. The native frontend
        // rebuilds at its existing frame publication point; managed commands
        // rebuild immediately before publishing their next snapshot.
    }

    void ApplicationService::setRegionLength(std::size_t index, double meters, const Publish& publish)
    {
        if (!std::isfinite(meters) || meters <= 0) throw std::invalid_argument("Region length must be finite and positive.");
        const double coordinateLength = meters / track_.physicalSettings().metersPerCoordinateUnit;
        if (track_.section(index).length == coordinateLength) return;
        AuthoredTrackEditTransaction transaction{track_};
        resizeRegion(transaction.candidate(), index, coordinateLength);
        edit(std::move(transaction), publish);
    }

    void ApplicationService::setInitialSpeed(double speed, const Publish& publish)
    {
        if (!std::isfinite(speed) || speed < 0) throw std::invalid_argument("Initial speed must be finite and nonnegative.");
        if (speed == track_.physicalSettings().initialSpeed) return;
        AuthoredTrackEditTransaction transaction{track_};
        auto settings = track_.physicalSettings();
        settings.initialSpeed = speed;
        transaction.candidate().setPhysicalSettings(settings);
        edit(std::move(transaction), publish);
    }

    void ApplicationService::editRegionProfile(std::size_t region, ProfileChannel channel,
        ProfileEdit request, const Publish& publish)
    {
        if (static_cast<unsigned>(channel) >= profileChannelCount)
            throw std::invalid_argument("Unknown profile channel.");
        AuthoredTrackEditTransaction transaction{track_};
        auto& section = transaction.candidate().section(region);
        const auto units = track_.physicalSettings().metersPerCoordinateUnit;
        const auto styles = coaster::isForceDrivenSection(section)
            ? forceDrivenRowStyles(units) : rateProfileRowStyles(units);
        applyProfileEdit(sectionProfileChannel(section, channel), request, units,
            styles[static_cast<unsigned>(channel)].displayToAuthored);
        edit(std::move(transaction), publish);
    }

    void ApplicationService::editDeviceProfile(coaster::TrackDeviceId id,
        ProfileEdit request, const Publish& publish)
    {
        AuthoredTrackEditTransaction transaction{track_};
        auto device = deviceById(track_, id);
        if (!device.accelerationProfile) throw std::invalid_argument("Track Device uses Constant acceleration.");
        applyProfileEdit(*device.accelerationProfile, request, 1, 1);
        transaction.candidate().updateTrackDevice(device);
        edit(std::move(transaction), publish);
    }

    void ApplicationService::setDeviceAcceleration(coaster::TrackDeviceId id, bool custom,
        double acceleration, const Publish& publish)
    {
        if (!std::isfinite(acceleration) || acceleration < 0)
            throw std::invalid_argument("Acceleration magnitude must be finite and nonnegative.");
        auto device = deviceById(track_, id);
        device.targetAccelerationMetersPerSecondSquared = acceleration;
        if (custom && !device.accelerationProfile)
        {
            const auto length = coaster::trackDeviceLengthMeters(device, track_.trackLengthMeters(),
                track_.layoutMode() == coaster::LayoutMode::Circuit);
            coaster::ChannelProfile profile;
            profile.segments.push_back({profile.nextSegmentId++, {0, length, acceleration, acceleration,
                math::TransitionType::Linear}});
            device.accelerationProfile = std::move(profile);
        }
        if (!custom) device.accelerationProfile.reset();
        if (device == deviceById(track_, id)) return;
        AuthoredTrackEditTransaction transaction{track_};
        transaction.candidate().updateTrackDevice(device);
        edit(std::move(transaction), publish);
    }

    void ApplicationService::restoreHistory(bool undo, const Publish& publish)
    {
        auto history = history_;
        auto track = undo ? history.undo() : history.redo();
        if (!track) throw std::invalid_argument(undo ? "Nothing to undo." : "Nothing to redo.");
        auto prepared = prepare(std::move(*track));
        if (publish) publish(prepared);
        replace(std::move(prepared));
        history_ = std::move(history);
        synchronizeDirtyState();
    }

    void ApplicationService::restoreHistoryWithPublication(bool undo, const HistoryPublish& publish)
    {
        auto history = history_;
        auto restored = undo ? history.undo() : history.redo();
        if (!restored) return;
        // Specialized publication may reuse existing style/device products.
        // Keep the real cursor unchanged until that publication succeeds.
        publish(*restored, history.lastRestoreTrackStylePresentationImpact());
        history_ = std::move(history);
        ++revision_;
        synchronizeDirtyState();
    }

    void ApplicationService::recordAcceptedEdit(bool continuous,
        std::optional<TrackStylePresentationImpact> impact)
    {
        history_.record(track_, continuous, impact);
        ++revision_;
        synchronizeDirtyState();
    }
    void ApplicationService::synchronizeDirtyState()
    {
        if (history_.isDirty()) fileState_.markDirty();
        else fileState_.clearDirty();
    }
    bool ApplicationService::rebuildPreview() noexcept { return preview_.rebuild(track_); }
    void ApplicationService::play()
    {
        if (!preview_.isAvailable()) throw std::invalid_argument(preview_.error());
        preview_.play();
    }
    void ApplicationService::pause() noexcept { preview_.pause(); }
    void ApplicationService::reset() noexcept { preview_.reset(); }
    void ApplicationService::update(double seconds, bool discontinuity) noexcept { preview_.update(seconds, discontinuity); }
}
