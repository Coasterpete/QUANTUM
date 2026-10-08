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

    enum class ProfileOwner { Region, Device };
    // This small authored-transform seam currently targets Track Start. Add a
    // concrete target only when its workspace migrates; no scene graph needed.
    enum class TransformTarget { TrackStart };
    struct ProfileTarget
    {
        ProfileOwner owner = ProfileOwner::Region;
        std::uint64_t ownerId = 0;
        ProfileChannel channel = ProfileChannel::Roll;
    };

    enum class SupportOperation
    {
        CreateStructure, DeleteStructure, RenameStructure, CreateNode, DeleteNode,
        MoveNode, SetAttachment, ClearAttachment, SetFoundation, ClearFoundation,
        CreateMember, DeleteMember, SetProfile, SetTreatment, ClearConnection,
        SetConnectorAsset, ClearConnectionPlacement
    };
    // Core units throughout; the managed boundary converts its SI form values.
    struct SupportEdit
    {
        SupportOperation operation;
        coaster::SupportStructureId structureId = 0;
        coaster::SupportElementId elementId = 0, startNodeId = 0, endNodeId = 0;
        glm::dvec3 position{0.0};
        coaster::TrackAttachment attachment;
        coaster::SupportMemberProfile profile;
        coaster::SupportMemberEnd memberEnd = coaster::SupportMemberEnd::Start;
        coaster::SupportMemberEndTreatment treatment = coaster::SupportMemberEndTreatment::MiteredCut;
        std::string text;
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
        void setTrackStartPosition(const glm::dvec3& meters, const Publish& publish = {});
        void setTrackStartAngles(const glm::dvec3& pitchYawRollDegrees, const Publish& publish = {});
        [[nodiscard]] std::uint64_t beginTransform(TransformTarget target,
            StartPoseTransformMode mode, StartPoseTransformAxis axis);
        // Cumulative SI metres / radians relative to the gesture's original
        // pose. Invalid proposals preserve the last accepted candidate.
        void previewTransform(std::uint64_t token, double amount);
        void finishTransform(std::uint64_t token, bool commit, const Publish& publish = {});
        void cancelTransform() noexcept;
        [[nodiscard]] std::uint64_t transformRevision(std::uint64_t token) const;
        [[nodiscard]] std::optional<coaster::AuthoredStartPose> transformPreview() const noexcept;
        [[nodiscard]] std::uint64_t transformPresentationRevision() const noexcept { return transformPresentationRevision_; }
        void editRegionProfile(std::size_t region, ProfileChannel channel,
            ProfileEdit edit, const Publish& publish = {});
        void editDeviceProfile(coaster::TrackDeviceId device, ProfileEdit edit,
            const Publish& publish = {});
        void setDeviceAcceleration(coaster::TrackDeviceId device, bool custom,
            double metersPerSecondSquared, const Publish& publish = {});
        [[nodiscard]] coaster::TrackDeviceId createDevice(coaster::TrackDeviceKind kind,
            const Publish& publish = {});
        void deleteDevice(coaster::TrackDeviceId device, const Publish& publish = {});
        void setDeviceProperties(coaster::TrackDeviceId device, bool enabled,
            double startMeters, double endMeters, double maximumForceNewtons,
            const Publish& publish = {});
        using SupportPublish = std::function<void(const SupportVisualization&)>;
        // Returns the allocated ID for creation, zero for other operations.
        // Support-only edits retain solved track geometry and simulation state.
        [[nodiscard]] std::uint32_t editSupport(const SupportEdit& request,
            const SupportPublish& publish = {});

        // A gesture owns only a native analytical candidate. It never changes
        // document/history/simulation until full preparation succeeds at commit.
        [[nodiscard]] std::uint64_t beginProfileDrag(ProfileTarget target, ProfileEdit edit);
        [[nodiscard]] const coaster::ChannelProfile& previewProfileDrag(std::uint64_t token,
            double displayValue, double snapIncrement = 0);
        void commitProfileDrag(std::uint64_t token, const Publish& publish = {});
        void cancelProfileDrag(std::uint64_t token) noexcept;
        [[nodiscard]] std::uint64_t profileDragRevision(std::uint64_t token) const;
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
        void setTrackStartPose(const coaster::AuthoredStartPose& pose, const Publish& publish);
        struct TransformGesture
        {
            std::uint64_t token, revision;
            TransformTarget target;
            StartPoseTransformMode mode;
            StartPoseTransformAxis axis;
            coaster::AuthoredStartPose original, candidate;
        };
        std::optional<TransformGesture> transform_;
        std::uint64_t nextTransformToken_ = 1, transformPresentationRevision_ = 1;
        struct ProfileDrag
        {
            std::uint64_t token, revision;
            ProfileTarget target;
            ProfileEdit edit;
            coaster::ChannelProfile original, candidate;
            double coordinateScale, displayToAuthored, extent;
        };
        std::optional<ProfileDrag> profileDrag_;
        std::uint64_t nextProfileDragToken_ = 1;
        coaster::AuthoredTrack track_;
        DocumentHistory history_;
        DocumentState fileState_;
        CenterlineVisualizationCache centerline_;
        SupportVisualization supports_;
        SimulationPreview preview_;
        std::uint64_t revision_ = 1;
    };
}
