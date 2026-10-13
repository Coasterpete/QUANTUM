#include <quantum/editor/ApplicationService.hpp>
#include <quantum/editor/ViewportTransformHandles.hpp>
#include <quantum/editor/TrackDevicePresentation.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
    using namespace quantum;
    void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
    void near(glm::dvec3 a, glm::dvec3 b, double tolerance, const char* message)
    { require(glm::length(a - b) <= tolerance, message); }
    template<class F> void rejected(F action)
    { try { action(); } catch (const std::exception&) { return; } throw std::runtime_error("Expected rejection"); }
    std::string saved(editor::ApplicationService& app) { return coaster::serializeCoasterDocument(app.track()); }
    nlohmann::json shape(editor::ApplicationService& app)
    { auto data = nlohmann::json::parse(saved(app)); data.erase("startPose"); return data; }
    void anglesAndHandles()
    {
        for (auto angles : {glm::dvec3{.2, -.5, .7}, glm::dvec3{1.5707963267948966, .4, -.8},
            glm::dvec3{-1.5707963267948966, -.9, .2}})
        {
            auto pose = editor::startPoseWithAngles({}, angles);
            const auto roundtrip = editor::startPoseWithAngles(pose, editor::startPoseAngles(pose));
            require(std::min(glm::length(pose.orientation - roundtrip.orientation),
                glm::length(pose.orientation + roundtrip.orientation)) < 1e-12, "Euler display reconstructs quaternion including both singularities");
        }
        const auto pitch = coaster::startPoseRiderFrame(editor::startPoseWithAngles({}, {.3, 0, 0}));
        near(pitch.tangent, {std::cos(.3), 0, -std::sin(.3)}, 1e-14, "Positive pitch follows native downward convention");
        editor::ViewportCamera camera; camera.setBounds({-10, -10, -10}, {10, 10, 10}); camera.frame(1.6);
        for (double scale : {1.0, 1.5, 2.0})
        {
            auto handles = editor::viewportTransformHandles(camera, {0, 0, 0}, 1200, 750, scale);
            require(handles.has_value(), "Native marker is projectable");
            for (unsigned i = 0; i < 3; ++i)
            {
                const auto axis = static_cast<editor::StartPoseTransformAxis>(i);
                require(editor::pickViewportTransformHandle(*handles, editor::StartPoseTransformMode::Move, handles->move[i].end) == axis,
                    "Projected X/Y/Z handles pick their own native axes");
                require(editor::pickViewportTransformHandle(*handles, editor::StartPoseTransformMode::Rotate,
                    handles->origin + glm::dvec2{handles->rotateRadii[i], 0}) == axis, "All native rotation controls pick by radius");
            }
            require(!editor::pickViewportTransformHandle(*handles, editor::StartPoseTransformMode::Move, {-500, -500}), "Empty viewport does not select a handle");
        }
    }
    void gestureContract()
    {
        editor::ApplicationService app;
        const auto before = saved(app); const auto revision = app.revision(); const auto history = app.history().size();
        const auto generation = app.centerline().generation(), trainGeneration = app.preview().vertexGeneration();
        const auto token = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Move, editor::StartPoseTransformAxis::Z);
        auto watch = std::chrono::steady_clock::now();
        for (int i = 0; i < 1000; ++i) app.previewTransform(token, i * .01);
        const auto milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - watch).count();
        require(app.revision() == revision && saved(app) == before && app.history().size() == history
            && app.centerline().generation() == generation && app.preview().vertexGeneration() == trainGeneration,
            "1000 proposals leave authored document, history, geometry and train untouched");
        const auto lastAccepted = *app.transformPreview();
        rejected([&] { app.previewTransform(token, std::numeric_limits<double>::quiet_NaN()); });
        require(app.transformPreview() == lastAccepted, "Invalid proposal keeps last accepted preview");
        app.previewTransform(token, 15);
        app.finishTransform(token, true);
        require(app.track().startPose().position.z == 15 && app.history().size() == history + 1 && app.revision() == revision + 1,
            "One Move gesture is one native history state");
        app.restoreHistory(true); require(saved(app) == before, "Move Undo restores exact original native state");
        app.restoreHistory(false); require(app.track().startPose().position.z == 15, "Move Redo restores placement");
        const auto accepted = saved(app); const auto acceptedRevision = app.revision();
        auto drag = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Rotate, editor::StartPoseTransformAxis::Z);
        app.previewTransform(drag, .2); app.finishTransform(drag, false);
        require(saved(app) == accepted && app.revision() == acceptedRevision && !app.transformPreview(), "Cancel leaves no partial pose or history");
        drag = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Move, editor::StartPoseTransformAxis::X);
        app.previewTransform(drag, 0); app.finishTransform(drag, true);
        require(app.revision() == acceptedRevision, "No-move click has no history state");
        drag = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Move, editor::StartPoseTransformAxis::X);
        app.previewTransform(drag, 3); app.previewTransform(drag, 0); app.finishTransform(drag, true);
        require(app.revision() == acceptedRevision, "Out-and-back drag has no history state");
        drag = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Rotate, editor::StartPoseTransformAxis::Z);
        app.previewTransform(drag, .3);
        rejected([&] { app.finishTransform(drag, true, [](const auto&) { throw std::runtime_error("Publication rejected"); }); });
        require(saved(app) == accepted && app.revision() == acceptedRevision, "Failed publication preserves document/history");
        auto forceTrack = coaster::createNewDocument();
        auto setup = forceTrack.coasterSetup(); setup.heartline.offsetMeters = 0; forceTrack.setCoasterSetup(setup);
        forceTrack.section(0) = coaster::createForceDrivenSection(40);
        auto& force = std::get<coaster::ForceDrivenRegion>(std::get<coaster::GeometryRegion>(forceTrack.section(0).region).construction);
        force.targetNormalG.segments[0].transition.valueBegin = force.targetNormalG.segments[0].transition.valueEnd = 0;
        editor::ApplicationService forceApp{std::move(forceTrack)};
        const auto forceBefore = saved(forceApp); const auto forceRevision = forceApp.revision();
        drag = forceApp.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Rotate, editor::StartPoseTransformAxis::Y);
        forceApp.previewTransform(drag, -std::acos(-1.0) / 2);
        rejected([&] { forceApp.finishTransform(drag, true); });
        require(saved(forceApp) == forceBefore && forceApp.revision() == forceRevision && !forceApp.transformPreview(),
            "Full native validation rejects unreachable force-driven uphill placement without a partial commit");
        const auto old = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Move, editor::StartPoseTransformAxis::Y);
        const auto current = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Move, editor::StartPoseTransformAxis::Z);
        app.finishTransform(old, false); app.previewTransform(current, 2);
        require(app.transformPreview()->position.z == 17, "Late cancellation cannot touch a newer gesture");
        app.setRegionLength(0, app.track().trackLengthMeters() + 1);
        rejected([&] { app.previewTransform(current, 4); }); require(!app.transformPreview(), "Stale transform cancels after authored revision changes");
        std::cout << "1000 native pose previews: " << milliseconds << " ms; geometry solves: 0\n";
    }
    void crossSystem(const char* fixture)
    {
        auto loaded = editor::ApplicationService::prepareFile(fixture); require(loaded.has_value(), "Open real document");
        editor::ApplicationService app{std::move(*loaded)};
        const auto device = app.createDevice(coaster::TrackDeviceKind::Launch);
        app.setDeviceProperties(device, true, 0, app.track().trackLengthMeters(), 1000000);
        app.setDeviceAcceleration(device, true, 2);
        auto profile = *app.track().trackDevices().devices.back().accelerationProfile;
        app.editDeviceProfile(device, {profile.segments.front().id, editor::ProfileEditOperation::EndValue, 3});
        using Op = editor::SupportOperation;
        editor::SupportEdit edit{Op::CreateStructure}; edit.text = "Placement regression";
        const auto sid = app.editSupport(edit);
        edit = {Op::CreateNode, sid}; edit.position = {2, 4, 1}; const auto attached = app.editSupport(edit);
        edit.position = {5, -2, 0}; const auto free = app.editSupport(edit);
        edit.position = {6, -3, -2}; const auto foundation = app.editSupport(edit);
        edit = {Op::SetAttachment, sid, attached}; edit.attachment = {10, 1, -.5}; static_cast<void>(app.editSupport(edit));
        edit = {Op::SetFoundation, sid, foundation}; static_cast<void>(app.editSupport(edit));
        edit = {Op::CreateMember, sid}; edit.startNodeId = foundation; edit.endNodeId = attached; static_cast<void>(app.editSupport(edit));
        const auto authoredShape = shape(app);
        const auto original = app.track().startPose(); const auto originalSamples = app.centerline().visualization().samples;
        editor::ViewportCamera anchorCamera;
        anchorCamera.setBounds(app.centerline().visualization().minimumPosition, app.centerline().visualization().maximumPosition);
        anchorCamera.frame(1.6);
        require(anchorCamera.frameSphere(original.position, 5, 1.6), "Frame Track Start succeeds for real native geometry");
        const auto anchorHandles = editor::viewportTransformHandles(anchorCamera, original.position, 800, 500);
        require(anchorHandles.has_value(), "Framed Track Start marker remains projectable");
        require(glm::length(anchorHandles->origin - glm::dvec2{400, 250}) < .01, "Frame Track Start centers the native marker for picking");
        const std::vector<renderer::LineVertex> originalTrain{app.preview().vertices().begin(), app.preview().vertices().end()};
        const auto originalDevice = editor::createTrackDeviceSelectionVertices(app.track(), app.centerline().visualization(), device);
        const auto originalTrainGeneration = app.preview().vertexGeneration();
        std::vector<std::string> states{saved(app)};
        auto position = original.position * app.track().physicalSettings().metersPerCoordinateUnit;
        for (unsigned axis = 0; axis < 3; ++axis)
        {
            position[axis] += axis == 2 ? 15 : axis == 1 ? -3 : 2;
            app.setTrackStartPosition(position); states.push_back(saved(app));
            require(shape(app) == authoredShape, "Numeric XYZ leaves all analytic profiles, boundaries, devices and authored supports exactly unchanged");
        }
        const auto translated = app.centerline().visualization().samples;
        const auto offset = app.track().startPose().position - original.position;
        require(originalSamples.size() == translated.size(), "Translation preserves native sample topology");
        for (std::size_t i = 0; i < translated.size(); ++i)
            near(translated[i].position, originalSamples[i].position + offset, 1e-8, "Whole real coaster translates including Z raising");
        require(app.preview().vertexGeneration() > originalTrainGeneration && app.preview().vertices().size() == originalTrain.size(),
            "Train presentation rebuilt with preserved topology");
        for (std::size_t i = 0; i < originalTrain.size(); ++i)
        {
            const auto& a = originalTrain[i]; const auto& b = app.preview().vertices()[i];
            near({b.x, b.y, b.z}, glm::dvec3{a.x, a.y, a.z} + offset, 3e-5, "Train preview has no stale previous-world vertices");
        }
        const auto translatedDevice = editor::createTrackDeviceSelectionVertices(app.track(), app.centerline().visualization(), device);
        require(translatedDevice.size() == originalDevice.size() && !translatedDevice.empty(), "Selected device interval remains visible");
        for (std::size_t i = 0; i < originalDevice.size(); ++i)
        {
            const auto& a = originalDevice[i]; const auto& b = translatedDevice[i];
            near({b.x, b.y, b.z}, glm::dvec3{a.x, a.y, a.z} + offset, 3e-5, "Selected interval follows relocated native track");
        }
        auto angles = glm::degrees(editor::startPoseAngles(app.track().startPose())); angles.y += 17;
        app.setTrackStartAngles(angles); states.push_back(saved(app));
        const auto move = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Move, editor::StartPoseTransformAxis::Z);
        app.previewTransform(move, 4); app.finishTransform(move, true); states.push_back(saved(app));
        const auto rotate = app.beginTransform(editor::TransformTarget::TrackStart, editor::StartPoseTransformMode::Rotate, editor::StartPoseTransformAxis::Z);
        app.previewTransform(rotate, -.1); app.finishTransform(rotate, true); states.push_back(saved(app));
        require(shape(app) == authoredShape, "Numeric and gizmo rotation preserve every authored local/profile/device/support field");
        const auto& structure = app.track().supports().structures.back();
        for (const auto& node : structure.nodes)
        {
            const auto found = std::find_if(app.supports().nodes.begin(), app.supports().nodes.end(), [&](const auto& n) { return n.selection.structureId == sid && n.selection.elementId == node.id; });
            require(found != app.supports().nodes.end(), "Relocated support presentation exists");
            const auto expected = node.trackAttachment ? coaster::resolveSupportTrackAttachment(app.centerline().visualization().samples, *node.trackAttachment).position : node.position;
            near(found->position, expected, 1e-12, "Attached follows native evaluation; free/foundation remain in world space");
        }
        require(app.preview().isAvailable() && app.preview().pose(), "Native train rebuilt on relocated track");
        const auto rotation = app.track().startPose().orientation * glm::conjugate(original.orientation);
        for (std::size_t i = 0; i < originalTrain.size(); ++i)
        {
            const auto& a = originalTrain[i]; const auto& b = app.preview().vertices()[i];
            near({b.x, b.y, b.z}, app.track().startPose().position + rotation * (glm::dvec3{a.x, a.y, a.z} - original.position),
                4e-5, "Rotated train presentation follows regenerated native placement");
        }
        app.play(); app.update(1.0 / 60); app.pause();
        require(app.preview().dynamicsState()->tick > 0 && app.preview().playbackState() == editor::SimulationPreview::PlaybackState::Paused, "Relocated train Play/Pause uses existing fixed-step simulation");
        const auto& forces = app.preview().lastDeviceForces().devices;
        require(std::any_of(forces.begin(), forces.end(), [&](auto& f) { return f.id == device && f.occupiedBogieCount > 0 && f.appliedForceNewtons > 0; }), "Native train encounters unchanged custom device interval");
        app.reset(); require(app.preview().dynamicsState()->tick == 0, "Reset clears preview tick after placement");
        for (std::size_t i = states.size() - 1; i > 0; --i)
        { app.restoreHistory(true); require(saved(app) == states[i - 1], "Repeated Undo restores exact sequential placements"); }
        for (std::size_t i = 1; i < states.size(); ++i)
        { app.restoreHistory(false); require(saved(app) == states[i], "Repeated Redo restores exact sequential placements"); }
        const auto path = std::filesystem::temp_directory_path() / ("quantum-placement-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".quantum");
        app.save(path); editor::ApplicationService restored; restored.open(path); std::filesystem::remove(path);
        require(saved(restored) == saved(app) && restored.preview().isAvailable(), "Native save/close/reload restores exact placement and cross-system authoring");
    }
}
int main(int argc, char** argv)
{
    try { require(argc == 2, "Real fixture argument required"); anglesAndHandles(); gestureContract(); crossSystem(argv[1]); std::cout << "Track placement service checks passed\n"; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
