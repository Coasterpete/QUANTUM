#pragma once

#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/DocumentHistory.hpp>
#include <quantum/editor/DocumentState.hpp>
#include <quantum/editor/SimulationPreview.hpp>
#include <quantum/editor/SupportVisualization.hpp>
#include <quantum/editor/TransitionEditorModel.hpp>
#include <expected>
#include <filesystem>
#include <functional>

namespace quantum::editor
{
    struct PreparedDocument
    {
        coaster::AuthoredTrack track;
        CenterlineVisualization centerline;
        SupportVisualization supports;
        coaster::RiderLoadHistory riderLoads;
    };
    struct DocumentPreparationTelemetry
    {
        double centerlineMilliseconds = 0;
        double riderLoadsMilliseconds = 0;
        double supportsMilliseconds = 0;
    };

    enum class ProfileEditOperation { BeginValue, EndValue, TransitionType, Split, Remove, MoveBegin, MoveEnd };
    struct ProfileEdit
    {
        coaster::SegmentId segmentId;
        ProfileEditOperation operation;
        double value = 0;
        math::TransitionType transitionType = math::TransitionType::Linear;
    };

    // Native application state shared by the ImGui and managed frontends.
    // The caller serializes access; GPU publication stays on its window thread.
    class ApplicationService
    {
    public:
        explicit ApplicationService(coaster::AuthoredTrack track = coaster::createNewDocument());
        explicit ApplicationService(PreparedDocument prepared);
        ApplicationService(const ApplicationService&) = delete;
        ApplicationService& operator=(const ApplicationService&) = delete;

        [[nodiscard]] static PreparedDocument prepare(coaster::AuthoredTrack track,
            DocumentPreparationTelemetry* telemetry = nullptr);
        [[nodiscard]] static std::expected<PreparedDocument, std::string>
            prepareFile(const std::filesystem::path& path);
        static void resizeRegion(coaster::AuthoredTrack& candidate,
            std::size_t index, double coordinateLength);

        // Optional publication runs before committing CPU state. A rejected
        // upload leaves document/history intact, following the ImGui contract.
        using Publish = std::function<void(const PreparedDocument&)>;
        void newDocument(const Publish& publish = {});
        void open(const std::filesystem::path& path, const Publish& publish = {});
        void openPrepared(PreparedDocument prepared, const std::filesystem::path& path,
            const Publish& publish = {});
        void save(const std::filesystem::path& path);
        void setRegionLength(std::size_t index, double meters, const Publish& publish = {});
        void setInitialSpeed(double metersPerSecond, const Publish& publish = {});
        void editRegionProfile(std::size_t region, ProfileChannel channel,
            ProfileEdit edit, const Publish& publish = {});
        void editDeviceProfile(coaster::TrackDeviceId device, ProfileEdit edit,
            const Publish& publish = {});
        void setDeviceAcceleration(coaster::TrackDeviceId device, bool custom,
            double metersPerSecondSquared, const Publish& publish = {});
        void restoreHistory(bool undo, const Publish& publish = {});
        using HistoryPublish = std::function<void(const coaster::AuthoredTrack&,
            std::optional<TrackStylePresentationImpact>)>;
        void restoreHistoryWithPublication(bool undo, const HistoryPublish& publish);
        void play();
        void pause() noexcept;
        void reset() noexcept;
        void update(double seconds, bool timingDiscontinuity = false) noexcept;
        [[nodiscard]] bool rebuildPreview() noexcept;

        // Existing specialized ImGui transactions already prepare and publish
        // their concrete products. They finish through this single history path.
        void recordAcceptedEdit(bool continuous = false,
            std::optional<TrackStylePresentationImpact> impact = std::nullopt);
        void commitPreparedEdit(AuthoredTrackEditTransaction& transaction,
            PreparedDocument prepared, bool continuous = false);
        void synchronizeDirtyState();
        [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }

        [[nodiscard]] coaster::AuthoredTrack& track() noexcept { return track_; }
        [[nodiscard]] const coaster::AuthoredTrack& track() const noexcept { return track_; }
        [[nodiscard]] DocumentHistory& history() noexcept { return history_; }
        [[nodiscard]] DocumentState& fileState() noexcept { return fileState_; }
        [[nodiscard]] CenterlineVisualizationCache& centerline() noexcept { return centerline_; }
        [[nodiscard]] SupportVisualization& supports() noexcept { return supports_; }
        [[nodiscard]] SimulationPreview& preview() noexcept { return preview_; }

    private:
        void replace(PreparedDocument prepared);
        void edit(AuthoredTrackEditTransaction transaction, const Publish& publish);
        coaster::AuthoredTrack track_;
        DocumentHistory history_;
        DocumentState fileState_;
        CenterlineVisualizationCache centerline_;
        SupportVisualization supports_;
        SimulationPreview preview_;
        std::uint64_t revision_ = 1;
    };
}
