#include <quantum/editor/ApplicationService.hpp>
#include <quantum/editor/InteractionPreferences.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <glm/geometric.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
    using namespace quantum;
    using namespace quantum::editor;
    using namespace quantum::coaster;
    using namespace quantum::physics;
    void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
    void near(double a, double b, double tolerance, const char* message)
    { require(std::abs(a - b) <= tolerance, message); }
    template<class F> void rejects(F function)
    {
        bool rejected = false;
        try { function(); } catch (const std::exception&) { rejected = true; }
        require(rejected, "Invalid interaction input accepted");
    }
    void cameraResponse()
    {
        near(viewportMouseRotationRadians(120, 1), viewportMouseRotationRadians(240, 1, 2),
            1e-14, "Equivalent motion at double DPI must rotate equally");
        near(viewportMouseRotationRadians(120, .5), .12, 1e-14, "Independent rotation sensitivity");
        near(viewportMouseRotationRadians(120, 1, 1, true), .024, 1e-14, "Shift mouse precision");
        rejects([] { static_cast<void>(viewportMouseRotationRadians(1, 1, 0)); });
        ViewportCamera one, split;
        one.setBounds({-100,-100,-100}, {100,100,100}); one.frame(1.6); split = one;
        one.orbit(viewportMouseRotationRadians(120, 1), 0);
        for (int i = 0; i < 120; ++i) split.orbit(viewportMouseRotationRadians(1, 1), 0);
        near(one.yaw(), split.yaw(), 1e-12, "Mouse event partition cannot change orbit response");
        split = one;
        one.pan(100, 40, 1600, 1000); split.pan(200, 80, 3200, 2000);
        near(glm::length(one.focus() - split.focus()), 0, 1e-10, "Pan is DPI and viewport size consistent");
        for (auto projection : {ViewportProjection::Perspective, ViewportProjection::Orthographic})
        {
            ViewportCamera camera = one; camera.setProjection(projection);
            const auto ray = camera.viewportRay(.8, .3, 1.6);
            const auto normal = glm::normalize(camera.focus() - camera.position());
            const auto point = ray.origin + ray.direction
                * glm::dot(camera.focus() - ray.origin, normal) / glm::dot(ray.direction, normal);
            camera.zoomAt(2, .8, .3, 1.6, .1);
            const auto after = camera.viewportRay(.8, .3, 1.6);
            const auto toPoint = point - after.origin;
            near(glm::length(glm::cross(toPoint, after.direction)), 0, 1e-9,
                "Cursor zoom must preserve the focus-plane point under the cursor in both projections");
        }
    }
    void preferences()
    {
        InteractionPreferences p;
        p.camera.panSensitivity = .3F; p.camera.zoomToCursor = false;
        p.transitions.fineDragGain = .01F; p.transitions.snapEnabled = true;
        const auto encoded = serializeInteractionPreferences(p);
        require(serializeInteractionPreferences(deserializeInteractionPreferences(encoded)) == encoded,
            "Camera and transition preferences survive a restart codec round trip");
        const auto partial = deserializeInteractionPreferences("{\"version\":1,\"camera\":{\"orbit\":0.5}}");
        near(partial.camera.orbitSensitivity, .5, 0, "Missing preference fields use compatible defaults");
        near(partial.camera.panSensitivity, 1, 0, "Missing pan preference default");
        rejects([] { static_cast<void>(deserializeInteractionPreferences("{\"camera\":{\"zoom\":0}}")); });
        rejects([] { static_cast<void>(deserializeInteractionPreferences("{\"version\":2}")); });
        rejects([] { static_cast<void>(deserializeInteractionPreferences("not json")); });
    }
    void transitions()
    {
        double fine = 0, coarse = 0;
        double fineProposal = 0;
        for (int i = 0; i < 100; ++i)
            fineProposal = accumulateMarkerValueDrag(fine, -1, .0001, .1, .001);
        const double coarseProposal = accumulateMarkerValueDrag(coarse, -100, .0001, .1, .001);
        near(fineProposal, coarseProposal, 1e-14, "Sub-grid drag motion must accumulate independently of frames");
        require(fineProposal > 0, "Fine snapping must not stick");
        for (const auto& preset : transitionTypePresets)
            for (double length : {5.0, 100.0})
                for (double target : {-2.0, .35, 3.0})
                {
                    math::ScalarTransition transition{10, 10 + length, .02, -.03, preset.type};
                    const auto before = transition;
                    const auto result = solveTransitionRotationTarget(transition, target);
                    require(result.converged && transition == before, "Every supported analytic target solves without mutation");
                    transition.valueEnd = result.endValue;
                    near(math::integrateScalarTransition(transition, 10, 10 + length), target, 1e-10,
                        "Rotation target residual obeys the analytic integral");
                }
        auto track = createNewDocument(); track.setLayoutMode(LayoutMode::Shuttle);
        track.section(0) = createRateProfileSection(80); track.appendSection();
        track.section(1) = createRateProfileSection(80);
        ApplicationService app(track);
        auto id = sectionProfileChannel(app.track().section(0), ProfileChannel::Pitch).segments.front().id;
        app.editRegionProfile(0, ProfileChannel::Pitch, {id, ProfileEditOperation::Split, 40});
        const auto before = serializeCoasterDocument(app.track());
        const auto transition = sectionProfileChannel(app.track().section(0), ProfileChannel::Pitch).segments.front().transition;
        const auto solved = solveTransitionRotationTarget(transition, .2);
        app.editRegionProfile(0, ProfileChannel::Pitch, {id, ProfileEditOperation::EndValue, solved.endValue * degreesPerRadian});
        const auto& profile = sectionProfileChannel(app.track().section(0), ProfileChannel::Pitch);
        near(profile.segments[0].transition.valueEnd, profile.segments[1].transition.valueBegin, 0,
            "Solved value maintains exact shared-boundary continuity");
        near(math::integrateScalarTransition(profile.segments[0].transition, 0, 40), .2, 1e-10,
            "Target result reaches the real authoring pipeline");
        const auto after = serializeCoasterDocument(app.track());
        app.restoreHistory(true); require(serializeCoasterDocument(app.track()) == before, "Target Undo is exact");
        app.restoreHistory(false); require(serializeCoasterDocument(app.track()) == after, "Target Redo is exact");
        const auto reload = deserializeCoasterDocument(after);
        require(reload && serializeCoasterDocument(*reload) == after, "Target persists through save/reopen");
        require(app.preview().isAvailable(), "Target geometry produces a usable preview");
        const auto& slices = app.centerline().visualization().sectionSlices;
        near(glm::length(slices[0].endPosition - slices[1].startPosition), 0, 1e-9,
            "Target regeneration preserves connected Region positions");
    }
    void deviceResize()
    {
        ApplicationService app;
        const auto id = app.createDevice(TrackDeviceKind::CableLift);
        app.setDeviceProperties(id, true, 0, 50, 100000);
        app.setDeviceAcceleration(id, true, 2);
        app.editDeviceProfile(id, {1, ProfileEditOperation::Split, 40});
        app.editDeviceProfile(id, {1, ProfileEditOperation::EndValue, 4});
        const auto before = serializeCoasterDocument(app.track());
        const auto original = *app.track().trackDevices().devices.back().accelerationProfile;
        app.setDeviceProperties(id, true, 10, 30, 100000);
        const auto& resized = *app.track().trackDevices().devices.back().accelerationProfile;
        near(resized.segments[0].transition.domainEnd, 16, 1e-12,
            "Shrinking past the former split preserves normalized custom-profile timing");
        for (double fraction : {0.0, .2, .8, .95, 1.0})
            near(evaluateChannelProfile(original, 50 * fraction), evaluateChannelProfile(resized, 20 * fraction),
                1e-12, "Interval resize retains custom-profile shape and amplitude");
        require(resized.nextSegmentId == original.nextSegmentId && resized.segments[1].id == original.segments[1].id,
            "Interval resize preserves stable identities");
        const auto after = serializeCoasterDocument(app.track());
        app.restoreHistory(true); require(serializeCoasterDocument(app.track()) == before, "Profile resize Undo exact");
        app.restoreHistory(false); require(serializeCoasterDocument(app.track()) == after, "Profile resize Redo exact");
        const auto reopened = deserializeCoasterDocument(after);
        require(reopened && serializeCoasterDocument(*reopened) == after, "Cable custom profile save/reopen exact");
    }
}
int main()
{
    try { cameraResponse(); preferences(); transitions(); deviceResize();
        std::cout << "Track authoring interaction regression checks passed\n"; return 0; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
