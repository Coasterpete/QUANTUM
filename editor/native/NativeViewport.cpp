#include "NativeViewport.hpp"
#include <quantum/editor/TrackDevicePresentation.hpp>
#include <quantum/editor/SupportPicking.hpp>
#include <quantum/editor/ViewportTrackAnchors.hpp>
#include <imgui_impl_vulkan.h>
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace
{
    void focusViewport(SDL_Window* window)
    {
#ifdef _WIN32
        // Raising a borrowed child does not transfer focus from an Avalonia
        // control. Focus the HWND itself, on the shared window-owner thread.
        const auto hwnd = static_cast<HWND>(SDL_GetPointerProperty(
            SDL_GetWindowProperties(window), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
        if (!hwnd) throw std::runtime_error("Viewport HWND is unavailable.");
        SetFocus(hwnd);
        if (GetFocus() != hwnd) throw std::runtime_error("Could not focus the viewport child.");
#else
        if (!SDL_RaiseWindow(window))
            throw std::runtime_error(std::string("Viewport focus request failed: ") + SDL_GetError());
#endif
    }
}

NativeViewport::~NativeViewport() { close(); }
void NativeViewport::requireOwnerThread() const
{
    if (owner_ != std::this_thread::get_id())
        throw std::invalid_argument("Viewport operations require its creating UI thread.");
}
void NativeViewport::open(uint32_t platform, uint64_t host,
    const quantum::editor::CenterlineVisualization& geometry)
{
    owner_ = std::this_thread::get_id();
    hosted_ = platform != 0;
    try
    {
        if (platform == 2) SDL_SetHint(SDL_HINT_VIDEO_DRIVER, "x11");
        if (!SDL_InitSubSystem(SDL_INIT_VIDEO))
            throw std::runtime_error(std::string("SDL video initialization failed: ") + SDL_GetError());
        video_ = true;
        if (!platform)
            window_ = SDL_CreateWindow("QUANTUM native viewport", 800, 500,
                SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
        else
        {
            const auto props = SDL_CreateProperties();
            if (!props) throw std::runtime_error(SDL_GetError());
            const bool configured = SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_VULKAN_BOOLEAN, true)
                && (platform == 1
                    ? SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_WIN32_HWND_POINTER,
                        reinterpret_cast<void*>(static_cast<uintptr_t>(host)))
                    : SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_X11_WINDOW_NUMBER, static_cast<Sint64>(host)));
            if (configured) window_ = SDL_CreateWindowWithProperties(props);
            SDL_DestroyProperties(props);
        }
        if (!window_) throw std::runtime_error(std::string("SDL native host wrapping failed: ") + SDL_GetError());
        renderer_.initialize(window_, geometry.vertices, geometry.verticesPerCurve, geometry.renderableTrack);
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(renderer_.physicalDevice(), &properties);
        SDL_Log("QUANTUM viewport device: %s (Vulkan device type %u)", properties.deviceName, properties.deviceType);
        renderer_.setViewportElementVisibility(false, 0);
        camera_.setBounds(geometry.minimumPosition, geometry.maximumPosition);
        camera_.frame(static_cast<double>(width_) / height_);
        imgui_ = ImGui::CreateContext();
        auto& io = ImGui::GetIO(); io.IniFilename = nullptr; io.LogFilename = nullptr;
        // ImGui is only the existing GPU texture presenter, with no product UI
        // or SDL input backend. Native child events and managed camera commands
        // both use the existing camera below.
        const auto format = renderer_.swapchainFormat();
        ImGui_ImplVulkan_InitInfo info{};
        info.ApiVersion = VK_API_VERSION_1_4;
        info.Instance = renderer_.instance(); info.PhysicalDevice = renderer_.physicalDevice();
        info.Device = renderer_.device(); info.QueueFamily = renderer_.graphicsQueueFamily();
        info.Queue = renderer_.graphicsQueue();
        info.DescriptorPoolSize = IMGUI_IMPL_VULKAN_MINIMUM_SAMPLED_IMAGE_POOL_SIZE;
        info.MinImageCount = 2; info.ImageCount = renderer_.swapchainImageCount();
        info.UseDynamicRendering = true;
        info.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        auto& rendering = info.PipelineInfoMain.PipelineRenderingCreateInfo;
        rendering.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
        rendering.colorAttachmentCount = 1; rendering.pColorAttachmentFormats = &format;
        info.CheckVkResultFn = [](VkResult result) {
            if (result < 0) throw std::runtime_error("Vulkan texture presenter failed: " + std::to_string(result));
        };
        if (!ImGui_ImplVulkan_Init(&info)) throw std::runtime_error("Vulkan texture presenter initialization failed.");
        backend_ = true;
        swapchainGeneration_ = renderer_.swapchainGeneration();
        renderer_.setFrameRenderCallback([](VkCommandBuffer buffer, void*) {
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), buffer);
        }, this);
    }
    catch (...) { close(); throw; }
}
void NativeViewport::retireTexture(void* object) noexcept
{
    auto& self = *static_cast<NativeViewport*>(object);
    if (self.texture_ != VK_NULL_HANDLE)
    {
        ImGui_ImplVulkan_RemoveTexture(self.texture_);
        self.texture_ = VK_NULL_HANDLE;
    }
}
bool NativeViewport::frame(uint32_t width, uint32_t height,
    const quantum::editor::CenterlineVisualization* geometry,
    const quantum::editor::SupportVisualization* supports,
    const quantum::coaster::GroundAppearance* ground, bool frameTrack)
{
    requireOwnerThread(); ImGui::SetCurrentContext(imgui_);
    if (!width || !height) return false;
    width_ = width; height_ = height;
    int inputWidth = 0, inputHeight = 0;
    if (!SDL_GetWindowSize(window_, &inputWidth, &inputHeight))
        throw std::runtime_error(std::string("SDL host size query failed: ") + SDL_GetError());
    const auto pick = [&](double x, double y) -> std::optional<quantum::editor::SupportSelection>
    {
        if (!supportActive_ || supports || geometry || inputWidth <= 0 || inputHeight <= 0
            || x < 0 || y < 0 || x > inputWidth || y > inputHeight) return std::nullopt;
        const auto scale = static_cast<double>(width) / inputWidth;
        return pickSupportAt(x / inputWidth, y / inputHeight, scale);
    };
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        // The hosted native child receives platform input directly. SDL's
        // existing-window wrapper reduces it to the same portable events the
        // production editor uses; C# toolbar actions use the explicit API.
        if (event.type == SDL_EVENT_MOUSE_MOTION)
        {
            const glm::dvec2 pointer{event.motion.x * static_cast<double>(width) / std::max(1, inputWidth),
                event.motion.y * static_cast<double>(height) / std::max(1, inputHeight)};
            if (pointerTransform_ && !(event.motion.state & SDL_BUTTON_LMASK)) cancelTransformInput();
            else if (pointerTransform_) placementPointer(1, pointer, static_cast<double>(width) / std::max(1, inputWidth));
            else if (event.motion.state & SDL_BUTTON_RMASK)
                camera_.orbit(event.motion.xrel * .005, event.motion.yrel * .005);
            else if (event.motion.state & SDL_BUTTON_MMASK)
                camera_.pan(event.motion.xrel, event.motion.yrel, width_, height_);
            else
            {
                hoveredSupport_ = pick(event.motion.x, event.motion.y);
                placementPointer(1, pointer, static_cast<double>(width) / std::max(1, inputWidth));
            }
        }
        else if (event.type == SDL_EVENT_MOUSE_WHEEL && !pointerTransform_) camera_.zoom(event.wheel.y);
        else if (event.type == SDL_EVENT_KEY_DOWN)
        {
            // Native-child focus bypasses Avalonia routed keys. Forward product
            // shortcuts as intents; document work still runs on the controller.
            if (event.key.key == SDLK_ESCAPE)
            { cancelTransformInput(); shortcuts_ |= 512u; }
            else if (event.key.key == SDLK_TAB)
                shortcuts_ |= (event.key.mod & SDL_KMOD_SHIFT) ? 128u : 64u;
            else if (event.key.mod & SDL_KMOD_CTRL)
            {
                if (event.key.key == SDLK_Z) shortcuts_ |= (event.key.mod & SDL_KMOD_SHIFT) ? 2u : 1u;
                else if (event.key.key == SDLK_Y) shortcuts_ |= 2u;
                else if (event.key.key == SDLK_S) shortcuts_ |= (event.key.mod & SDL_KMOD_SHIFT) ? 32u : 4u;
                else if (event.key.key == SDLK_O) shortcuts_ |= 8u;
                else if (event.key.key == SDLK_N) shortcuts_ |= 16u;
                else if (event.key.key == SDLK_T) shortcuts_ |= 1024u;
            }
            else if (!pointerTransform_ && event.key.key == SDLK_F)
            { if (trainActive_) frameTrain(14); else camera_.frame(static_cast<double>(width_) / height_); }
            else if (!pointerTransform_ && event.key.key == SDLK_W) camera_.moveLocal(1, 0, 0, .05);
            else if (!pointerTransform_ && event.key.key == SDLK_S) camera_.moveLocal(-1, 0, 0, .05);
            else if (!pointerTransform_ && event.key.key == SDLK_A) camera_.moveLocal(0, -1, 0, .05);
            else if (!pointerTransform_ && event.key.key == SDLK_D) camera_.moveLocal(0, 1, 0, .05);
        }
        else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        {
            if (event.button.button != SDL_BUTTON_LEFT) cancelTransformInput();
            focusViewport(window_);
            shortcuts_ |= 256u;
            if (trainActive_ && !supports && !geometry && event.button.button == SDL_BUTTON_LEFT && inputWidth > 0 && inputHeight > 0)
                pickTrainAt(event.button.x / inputWidth, event.button.y / inputHeight);
            if (!trainActive_ && supportActive_ && !supports && !geometry && event.button.button == SDL_BUTTON_LEFT)
            {
                selectedSupport_ = pick(event.button.x, event.button.y);
                supportPick_ = {supportRevision_, supportSelectionRevision_, selectedSupport_};
                if (!selectedSupport_ && inputWidth > 0 && inputHeight > 0) pickInfrastructureAt(event.button.x/inputWidth,event.button.y/inputHeight);
            }
            if (!trainActive_ && placementActive_ && !geometry && !supports && event.button.button == SDL_BUTTON_LEFT)
                placementPointer(0, {event.button.x * static_cast<double>(width) / std::max(1, inputWidth),
                    event.button.y * static_cast<double>(height) / std::max(1, inputHeight)},
                    static_cast<double>(width) / std::max(1, inputWidth));
            if ((event.button.button == SDL_BUTTON_RIGHT || event.button.button == SDL_BUTTON_MIDDLE)
                && !SDL_CaptureMouse(true))
                SDL_LogWarn(SDL_LOG_CATEGORY_INPUT, "Viewport mouse capture failed: %s", SDL_GetError());
        }
        else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP || event.type == SDL_EVENT_WINDOW_FOCUS_LOST)
        {
            if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) cancelTransformInput();
            else if (event.button.button == SDL_BUTTON_LEFT)
                placementPointer(2, {event.button.x * static_cast<double>(width) / std::max(1, inputWidth),
                    event.button.y * static_cast<double>(height) / std::max(1, inputHeight)},
                    static_cast<double>(width) / std::max(1, inputWidth));
            if (!SDL_CaptureMouse(false))
                SDL_LogWarn(SDL_LOG_CATEGORY_INPUT, "Viewport mouse release failed: %s", SDL_GetError());
            if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) hoveredSupport_.reset();
        }
        else if (event.type == SDL_EVENT_WINDOW_HIDDEN) cancelTransformInput();
        else if (event.type == SDL_EVENT_WINDOW_MOUSE_LEAVE)
        { hoveredSupport_.reset(); startHovered_ = false; hoveredAxis_.reset(); }
    }
    width_ = width; height_ = height;
    int logicalWidth = 0, logicalHeight = 0;
    if (!SDL_GetWindowSize(window_, &logicalWidth, &logicalHeight))
        throw std::runtime_error(std::string("SDL host size query failed: ") + SDL_GetError());
    if (!hosted_ && (logicalWidth != static_cast<int>(width) || logicalHeight != static_cast<int>(height))
        && !SDL_SetWindowSize(window_, static_cast<int>(width), static_cast<int>(height)))
        throw std::runtime_error(std::string("SDL host resize failed: ") + SDL_GetError());
    int pixelWidth = 0, pixelHeight = 0;
    if (!SDL_GetWindowSizeInPixels(window_, &pixelWidth, &pixelHeight))
        throw std::runtime_error(std::string("SDL pixel size query failed: ") + SDL_GetError());
    if (pixelWidth <= 0 || pixelHeight <= 0) return false;
    const auto extent = renderer_.viewportExtent();
    if (extent.width != static_cast<uint32_t>(pixelWidth) || extent.height != static_cast<uint32_t>(pixelHeight))
    {
        SDL_Log("QUANTUM viewport extent: managed %ux%u, SDL pixels %dx%d", width, height, pixelWidth, pixelHeight);
        renderer_.resizeViewportTarget(pixelWidth, pixelHeight, retireTexture, this, false);
    }
    if (texture_ == VK_NULL_HANDLE)
    {
        texture_ = ImGui_ImplVulkan_AddTexture(renderer_.viewportImageView(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        if (texture_ == VK_NULL_HANDLE) throw std::runtime_error("Viewport texture registration failed.");
    }
    if (geometry)
    {
        renderer_.updateTrackCurveVertices(geometry->vertices, geometry->verticesPerCurve);
        renderer_.updateRenderableTrack(geometry->renderableTrack);
        camera_.setBounds(geometry->minimumPosition, geometry->maximumPosition);
        if (frameTrack) camera_.frame(static_cast<double>(pixelWidth) / pixelHeight);
    }
    if (supports)
    {
        renderer_.updateSupportSolidPresentation(supports->solidPresentations);
        renderer_.updateSupportVertices(supports->memberVertices);
    }
    if (ground) renderer_.setGroundSurface(*ground);
    renderer_.setViewportViewProjection(camera_.viewProjection(static_cast<double>(pixelWidth) / pixelHeight));
    renderer_.setViewportCameraPosition(glm::vec3(camera_.position()));
    if (trainDirty_)
    {
        lines_.assign(train_.begin(), train_.end());
        lines_.insert(lines_.end(), device_.begin(), device_.end());
        lines_.insert(lines_.end(), placementGhost_.begin(), placementGhost_.end());
        renderer_.updateTrainPreviewVertices(lines_);
        renderer_.updateDynamicMeshInstances(meshes_);
        trainAssetFailed_ = false;
        for (const auto& instance : meshes_)
        {
            const auto status = renderer_.dynamicMeshAssetLoadStatus(instance.assetIdentifier);
            if (status && status->usingDiagnosticFallback) { trainAssetFailed_ = true; break; }
        }
        trainDirty_ = false;
    }
    auto& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(pixelWidth), static_cast<float>(pixelHeight));
    io.DisplayFramebufferScale = ImVec2(1, 1); io.DeltaTime = 1.0f / 60;
    ImGui_ImplVulkan_NewFrame(); ImGui::NewFrame();
    ImGui::GetBackgroundDrawList()->AddImage(ImTextureRef((ImTextureID)texture_), ImVec2(0, 0), io.DisplaySize);
    if (infrastructureDirty_)
    {
        renderer_.updateInfrastructurePresentation(infrastructure_, access_);
        infrastructureDirty_ = false;
    }
    if (supportActive_) drawSupportOverlay();
    drawPlacementOverlay();
    drawTrainOverlay();
    ImGui::Render(); renderer_.drawFrame();
    if (swapchainGeneration_ != renderer_.swapchainGeneration())
    {
        // Format and FIFO image count are stable under the hosted resize
        // path, matching the production editor's minimum-image-count policy.
        ImGui_ImplVulkan_SetMinImageCount(2);
        swapchainGeneration_ = renderer_.swapchainGeneration();
    }
    return true;
}
uint32_t NativeViewport::takeShortcuts() noexcept { return std::exchange(shortcuts_, 0); }
void NativeViewport::setPresentation(uint32_t mode)
{
    requireOwnerThread();
    if (mode > 3) throw std::invalid_argument("Unknown viewport presentation mode.");
    renderer_.setTrackPresentationMode(static_cast<quantum::renderer::TrackPresentationMode>(mode));
    renderer_.setViewportElementVisibility(false, mode == 3 ? quantum::renderer::viewportAllCurvesVisibleMask : 0);
}
void NativeViewport::captureTrain(uint64_t revision, uint64_t generation,
    std::span<const quantum::renderer::LineVertex> train,
    std::span<const quantum::renderer::StaticMeshInstance> meshes)
{
    if (revision == trainRevision_ && generation == trainGeneration_) return;
    // Reuse capacity and preserve the preview's generation-driven publication.
    train_.assign(train.begin(), train.end());
    meshes_.assign(meshes.begin(), meshes.end());
    trainRevision_ = revision; trainGeneration_ = generation; trainDirty_ = true;
}
void NativeViewport::captureDevice(uint64_t revision, uint64_t selected,
    const quantum::coaster::AuthoredTrack& track,
    const quantum::editor::CenterlineVisualization& geometry,
    const quantum::physics::RideOperations& operations, uint64_t infrastructureSelection)
{
    nextAccessFractions_.clear();
    auto& fractions = nextAccessFractions_;
    for (const auto& hardware : operations.hardware())
        if (const auto* gate = std::get_if<quantum::physics::AccessRuntime>(&hardware.mechanism))
            fractions.emplace_back(hardware.id, gate->fraction);
    const bool documentChanged = infrastructureRevision_ != revision;
    if (documentChanged)
    {
        infrastructurePicking_ = quantum::editor::createInfrastructurePresentation(track, geometry.samples);
        infrastructure_.clear();
        for (const auto& object : infrastructurePicking_) infrastructure_.push_back(object.solid);
    }
    if (documentChanged || fractions != accessFractions_)
    {
        accessPicking_ = quantum::editor::createAccessPresentation(track, geometry.samples, operations.hardware());
        access_.clear();
        for (const auto& object : accessPicking_) access_.push_back(object.solid);
        infrastructureDirty_ = true; infrastructureRevision_ = revision;
        accessFractions_.swap(nextAccessFractions_);
    }
    const auto found = std::find_if(operations.hardware().begin(), operations.hardware().end(), [selected](const auto& h) { return h.id == selected; });
    const auto* runtime = found == operations.hardware().end() ? nullptr : &*found;
    double position = 0;
    if (runtime)
    {
        if (const auto* t = std::get_if<quantum::physics::TransferRuntime>(&runtime->mechanism)) position = t->positionMeters;
        if (const auto* g = std::get_if<quantum::physics::AccessRuntime>(&runtime->mechanism)) position = g->fraction;
    }
    const auto label = runtime ? runtime->state : std::string{};
    if (position == deviceMechanismPosition_ && label == deviceMechanismState_ && revision == deviceRevision_ && selected == selectedDevice_ && infrastructureSelection == selectedInfrastructure_) return;
    device_ = quantum::editor::createTrackDeviceSelectionVertices(track, geometry, selected);
    auto mechanism = quantum::editor::createHardwareMechanismVertices(track, geometry, selected, runtime);
    device_.insert(device_.end(), mechanism.begin(), mechanism.end());
    deviceMechanismPosition_ = position; deviceMechanismState_ = label;
    selectedInfrastructure_ = infrastructureSelection;
    // A small bounding outline communicates infrastructure selection without
    // duplicating its geometry in the authoring model.
    if (selectedInfrastructure_)
        for (const auto& object : infrastructurePicking_) if (object.id == selectedInfrastructure_)
        {
            glm::dvec3 lo{std::numeric_limits<double>::infinity()}, hi{-std::numeric_limits<double>::infinity()};
            for (const auto& batch : object.solid.batches) for (const auto& instance : batch.instances)
                for (int corner = 0; corner < 8; ++corner)
                {
                    const auto p = glm::dvec3{glm::dmat4{instance.transform}*glm::dvec4{corner&1 ? .5 : -.5,corner&2 ? .5 : -.5,corner&4 ? .5 : -.5,1}};
                    lo = glm::min(lo,p); hi = glm::max(hi,p);
                }
            if (std::isfinite(lo.x)) for (int corner = 0; corner < 8; ++corner) for (int axis = 0; axis < 3; ++axis)
                if (!(corner & (1<<axis))) for (int end = 0; end < 2; ++end)
                {
                    const int c = corner | (end<<axis);
                    const glm::dvec3 p{c&1 ? hi.x : lo.x,c&2 ? hi.y : lo.y,c&4 ? hi.z : lo.z};
                    device_.push_back({static_cast<float>(p.x),static_cast<float>(p.y),static_cast<float>(p.z),{1,.65F,.15F,1}});
                }
        }
    deviceRevision_ = revision; selectedDevice_ = selected; trainDirty_ = true;
}
void NativeViewport::input(uint32_t action, double x, double y, double z, double value)
{
    switch (action)
    {
    case 1: camera_.orbit(x, y); break;
    case 2: camera_.pan(x, y, width_, height_); break;
    case 3: camera_.zoom(x); break;
    case 4: camera_.frame(static_cast<double>(width_) / height_); break;
    case 5: camera_.moveLocal(x, y, z, std::clamp(value, 0.0, 0.1)); break;
    case 6: focusViewport(window_); break;
    case 7: frameSupportSelection(); break;
    case 8:
        if (!supportActive_) throw std::invalid_argument("Open Supports before selecting support geometry.");
        selectedSupport_ = pickSupportAt(x, y);
        supportPick_ = {supportRevision_, supportSelectionRevision_, selectedSupport_};
        if (!selectedSupport_) pickInfrastructureAt(x,y);
        break;
    case 9:
        camera_.frameSphere(displayedStartPose_.position, 5 / coordinateScale_, static_cast<double>(width_) / height_);
        break;
    case 10: case 11: case 12:
        placementPointer(action - 10, {x * width_, y * height_}, pixelScale_); break;
    case 13: cancelTransformInput(); break;
    case 14: case 15: case 16: case 17: frameTrain(action); break;
    case 18: pickTrainAt(x, y); break;
    }
}
void NativeViewport::pickInfrastructureAt(double x, double y)
{
    const auto ray = camera_.viewportRay(x,y,static_cast<double>(width_)/height_);
    auto hit = quantum::editor::pickInfrastructureSolid(infrastructurePicking_,ray.origin,ray.direction);
    auto gate = quantum::editor::pickInfrastructureSolid(accessPicking_,ray.origin,ray.direction);
    if (gate && (!hit || gate->distance < hit->distance)) hit = gate;
    if (hit) infrastructurePick_ = {infrastructureRevision_,hit->id,hit->access};
}
std::optional<NativeViewport::InfrastructurePickIntent> NativeViewport::takeInfrastructurePick() noexcept
{
    return std::exchange(infrastructurePick_,std::nullopt);
}
std::optional<quantum::editor::SupportSelection> NativeViewport::pickSupportAt(
    double x, double y, double pixelScale) const
{
    const auto hit = quantum::editor::pickSupport(supportPicking_, camera_, {x, y}, width_, height_,
        quantum::editor::supportNodeHitRadiusPixels * pixelScale,
        quantum::editor::supportMemberHitTolerancePixels * pixelScale);
    return hit ? std::optional{hit->selection} : std::nullopt;
}
void NativeViewport::captureSupports(uint64_t revision, uint64_t selectionRevision, bool active,
    std::optional<quantum::editor::SupportSelection> selection,
    const quantum::coaster::SupportCollection& supports,
    const quantum::editor::SupportVisualization& visualization)
{
    if (revision != supportRevision_)
    {
        // Picking needs the native double metadata, not another copy of solid
        // batches or GPU resources. All arrays are revision-cached.
        supportPicking_.nodes = visualization.nodes;
        supportPicking_.members = visualization.members;
        supportAnchors_.clear(); supportConnections_.clear();
        std::size_t memberIndex = 0;
        for (const auto& structure : supports.structures)
        {
            for (const auto& node : structure.nodes)
                supportAnchors_.push_back(node.trackAttachment ? 1u : node.foundation ? 2u : 0u);
            for (const auto& member : structure.members)
            {
                const auto& visual = visualization.members.at(memberIndex++);
                if (member.startConnection) supportConnections_.emplace_back(visual.startPosition, visual.selection);
                if (member.endConnection) supportConnections_.emplace_back(visual.endPosition, visual.selection);
            }
        }
        supportRevision_ = revision;
        hoveredSupport_.reset();
    }
    if (supportActive_ != active) { hoveredSupport_.reset(); supportPick_.reset(); }
    supportActive_ = active; selectedSupport_ = selection; supportSelectionRevision_ = selectionRevision;
}
std::optional<NativeViewport::SupportPickIntent> NativeViewport::takeSupportPick() noexcept
{
    return std::exchange(supportPick_, std::nullopt);
}
void NativeViewport::drawSupportOverlay()
{
    using namespace quantum::editor;
    auto* draw = ImGui::GetBackgroundDrawList();
    const auto& display = ImGui::GetIO().DisplaySize;
    draw->PushClipRect({0, 0}, display, true);
    const auto screen = [&](const glm::dvec3& point) -> std::optional<ImVec2>
    {
        const auto projected = projectViewportPoint(camera_, point, static_cast<double>(width_) / height_);
        if (!projected) return std::nullopt;
        return ImVec2{static_cast<float>(projected->normalizedPosition.x * display.x),
            static_cast<float>(projected->normalizedPosition.y * display.y)};
    };
    const auto selected = [&](const SupportSelection& value)
    {
        return selectedSupport_ && (selectedSupport_ == value
            || (selectedSupport_->kind == SupportSelectionKind::Structure && selectedSupport_->structureId == value.structureId));
    };
    constexpr ImU32 normal = IM_COL32(112, 200, 222, 255), highlight = IM_COL32(255, 200, 80, 255);
    constexpr ImU32 hover = IM_COL32(240, 240, 245, 255), outline = IM_COL32(10, 15, 20, 255);
    for (const auto& member : supportPicking_.members)
    {
        if (!selected(member.selection) && hoveredSupport_ != member.selection) continue;
        const auto start = screen(member.startPosition), end = screen(member.endPosition);
        if (!start || !end) continue;
        draw->AddLine(*start, *end, outline, 5);
        draw->AddLine(*start, *end, selected(member.selection) ? highlight : hover, 3);
    }
    for (std::size_t i = 0; i < supportPicking_.nodes.size(); ++i)
    {
        const auto& node = supportPicking_.nodes[i]; const auto center = screen(node.position);
        if (!center) continue;
        const auto color = selected(node.selection) ? highlight : hoveredSupport_ == node.selection ? hover : normal;
        const float radius = selected(node.selection) ? 5.0f : 3.5f;
        draw->AddCircleFilled(*center, radius + 1.5f, outline, 16);
        draw->AddCircleFilled(*center, radius, color, 16);
        if (supportAnchors_[i] == 2) draw->AddRect({center->x - 7, center->y - 7}, {center->x + 7, center->y + 7}, color);
        if (selected(node.selection) && supportAnchors_[i])
            draw->AddText({center->x + 9, center->y - 7}, color, supportAnchors_[i] == 1 ? "TRACK" : "FOUNDATION");
    }
    for (const auto& [position, selection] : supportConnections_)
        if (selected(selection))
            if (const auto point = screen(position)) draw->AddCircle(*point, 8, highlight, 16, 2);
    draw->PopClipRect();
}
void NativeViewport::frameSupportSelection()
{
    if (!selectedSupport_) return;
    auto view = supportPicking_;
    const auto keep = [&](const auto& value)
    {
        return selectedSupport_->kind == quantum::editor::SupportSelectionKind::Structure
            ? value.selection.structureId == selectedSupport_->structureId : value.selection == selectedSupport_;
    };
    std::erase_if(view.nodes, [&](const auto& n) { return !keep(n); });
    std::erase_if(view.members, [&](const auto& m) { return !keep(m); });
    if (const auto bounds = quantum::editor::supportVisualizationBounds(view))
    {
        const auto padding = glm::dvec3{0.5};
        camera_.frameBounds(bounds->first - padding, bounds->second + padding, static_cast<double>(width_) / height_);
    }
}
void NativeViewport::close() noexcept
{
    if (imgui_) ImGui::SetCurrentContext(imgui_);
    if (renderer_.device() != VK_NULL_HANDLE) vkDeviceWaitIdle(renderer_.device());
    renderer_.setFrameRenderCallback(nullptr, nullptr);
    if (backend_) { retireTexture(this); ImGui_ImplVulkan_Shutdown(); backend_ = false; }
    if (imgui_) { ImGui::DestroyContext(imgui_); imgui_ = nullptr; }
    renderer_.shutdown();
    if (window_) { SDL_DestroyWindow(window_); window_ = nullptr; }
    if (video_) { SDL_QuitSubSystem(SDL_INIT_VIDEO); video_ = false; }
}
