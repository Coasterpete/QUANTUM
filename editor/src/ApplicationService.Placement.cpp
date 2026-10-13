#include <quantum/editor/ApplicationService.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <stdexcept>

namespace quantum::editor
{
    namespace
    {
        bool samePose(const coaster::AuthoredStartPose& a, const coaster::AuthoredStartPose& b)
        {
            // Ignore only roundoff from the Inspector's decimal display and
            // equivalent q/-q. Meaningful proposals still use Core validation.
            return glm::length(a.position - b.position) <= 1.0e-10
                && std::min(glm::length(a.orientation - b.orientation),
                    glm::length(a.orientation + b.orientation)) <= 1.0e-12;
        }
    }
    void ApplicationService::setTrackStartPose(const coaster::AuthoredStartPose& pose, const Publish& publish)
    {
        AuthoredTrackEditTransaction transaction{track_};
        transaction.candidate().setStartPose(pose);
        if (samePose(track_.startPose(), transaction.candidate().startPose())) return;
        edit(std::move(transaction), publish);
    }
    void ApplicationService::setTrackStartPosition(const glm::dvec3& meters, const Publish& publish)
    {
        auto pose = track_.startPose();
        pose.position = meters / track_.physicalSettings().metersPerCoordinateUnit;
        setTrackStartPose(pose, publish);
    }
    void ApplicationService::setTrackStartAngles(const glm::dvec3& degrees, const Publish& publish)
    {
        setTrackStartPose(startPoseWithAngles(track_.startPose(), glm::radians(degrees)), publish);
    }
    std::uint64_t ApplicationService::beginTransform(TransformTarget target,
        StartPoseTransformMode mode, StartPoseTransformAxis axis)
    {
        if (target != TransformTarget::TrackStart || (mode != StartPoseTransformMode::Move
            && mode != StartPoseTransformMode::Rotate) || static_cast<unsigned>(axis) > 2)
            throw std::invalid_argument("Unknown authored transform target, mode or axis.");
        cancelTransform();
        const auto token = nextTransformToken_++;
        transform_ = TransformGesture{token, revision_, target, mode, axis, track_.startPose(), track_.startPose()};
        ++transformPresentationRevision_;
        return token;
    }
    std::uint64_t ApplicationService::transformRevision(std::uint64_t token) const
    {
        if (!transform_ || transform_->token != token)
            throw std::invalid_argument("This authored transform is no longer active.");
        return transform_->revision;
    }
    std::optional<coaster::AuthoredStartPose> ApplicationService::transformPreview() const noexcept
    {
        if (!transform_ || transform_->revision != revision_) return std::nullopt;
        return transform_->candidate;
    }
    void ApplicationService::previewTransform(std::uint64_t token, double amount)
    {
        if (transformRevision(token) != revision_)
        { cancelTransform(); throw std::invalid_argument("Document changed. Transform canceled."); }
        if (!std::isfinite(amount)) throw std::invalid_argument("Transform amount must be finite.");
        auto& gesture = *transform_;
        auto pose = gesture.original;
        if (amount != 0)
            pose = gesture.mode == StartPoseTransformMode::Move
                ? translateStartPose(pose, gesture.axis, amount / track_.physicalSettings().metersPerCoordinateUnit)
                : rotateStartPose(pose, gesture.axis, amount);
        // Core's strong guarantee validates/canonicalizes a temporary pose;
        // no geometry solve, history entry or simulation rebuild during input.
        coaster::AuthoredTrack candidate;
        candidate.setStartPose(pose);
        if (samePose(gesture.original, candidate.startPose())) pose = gesture.original;
        else pose = candidate.startPose();
        if (pose == gesture.candidate) return;
        gesture.candidate = pose;
        ++transformPresentationRevision_;
    }
    void ApplicationService::finishTransform(std::uint64_t token, bool commit, const Publish& publish)
    {
        if (!commit)
        {
            if (transform_ && transform_->token == token) cancelTransform();
            return;
        }
        const auto expected = transformRevision(token);
        const auto pose = transform_->candidate;
        cancelTransform();
        if (expected != revision_) throw std::invalid_argument("Document changed. Transform canceled.");
        setTrackStartPose(pose, publish);
    }
    void ApplicationService::cancelTransform() noexcept
    {
        if (transform_) { transform_.reset(); ++transformPresentationRevision_; }
    }
}
