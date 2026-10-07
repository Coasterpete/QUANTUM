#include "NativeViewport.hpp"
#include <imgui_impl_vulkan.h>
#include <SDL3/SDL.h>
#include <algorithm>
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
    const quantum::coaster::GroundAppearance* ground)
{
    requireOwnerThread(); ImGui::SetCurrentContext(imgui_);
    if (!width || !height) return false;
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        // The hosted native child receives platform input directly. SDL's
        // existing-window wrapper reduces it to the same portable events the
        // production editor uses; C# toolbar actions use the explicit API.
        if (event.type == SDL_EVENT_MOUSE_MOTION)
        {
            if (event.motion.state & SDL_BUTTON_RMASK)
                camera_.orbit(event.motion.xrel * .005, event.motion.yrel * .005);
            else if (event.motion.state & SDL_BUTTON_MMASK)
                camera_.pan(event.motion.xrel, event.motion.yrel, width_, height_);
        }
        else if (event.type == SDL_EVENT_MOUSE_WHEEL) camera_.zoom(event.wheel.y);
        else if (event.type == SDL_EVENT_KEY_DOWN)
        {
            // Native-child focus bypasses Avalonia routed keys. Forward product
            // shortcuts as intents; document work still runs on the controller.
            if (event.key.key == SDLK_TAB)
                shortcuts_ |= (event.key.mod & SDL_KMOD_SHIFT) ? 128u : 64u;
            else if (event.key.mod & SDL_KMOD_CTRL)
            {
                if (event.key.key == SDLK_Z) shortcuts_ |= (event.key.mod & SDL_KMOD_SHIFT) ? 2u : 1u;
                else if (event.key.key == SDLK_Y) shortcuts_ |= 2u;
                else if (event.key.key == SDLK_S) shortcuts_ |= (event.key.mod & SDL_KMOD_SHIFT) ? 32u : 4u;
                else if (event.key.key == SDLK_O) shortcuts_ |= 8u;
                else if (event.key.key == SDLK_N) shortcuts_ |= 16u;
            }
            else if (event.key.key == SDLK_F) camera_.frame(static_cast<double>(width_) / height_);
            else if (event.key.key == SDLK_W) camera_.moveLocal(1, 0, 0, .05);
            else if (event.key.key == SDLK_S) camera_.moveLocal(-1, 0, 0, .05);
            else if (event.key.key == SDLK_A) camera_.moveLocal(0, -1, 0, .05);
            else if (event.key.key == SDLK_D) camera_.moveLocal(0, 1, 0, .05);
        }
        else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        {
            focusViewport(window_);
            shortcuts_ |= 256u;
            if ((event.button.button == SDL_BUTTON_RIGHT || event.button.button == SDL_BUTTON_MIDDLE)
                && !SDL_CaptureMouse(true))
                SDL_LogWarn(SDL_LOG_CATEGORY_INPUT, "Viewport mouse capture failed: %s", SDL_GetError());
        }
        else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP || event.type == SDL_EVENT_WINDOW_FOCUS_LOST)
        {
            if (!SDL_CaptureMouse(false))
                SDL_LogWarn(SDL_LOG_CATEGORY_INPUT, "Viewport mouse release failed: %s", SDL_GetError());
        }
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
        camera_.frame(static_cast<double>(pixelWidth) / pixelHeight);
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
        renderer_.updateTrainPreviewVertices(train_);
        renderer_.updateDynamicMeshInstances(meshes_);
        trainDirty_ = false;
    }
    auto& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(pixelWidth), static_cast<float>(pixelHeight));
    io.DisplayFramebufferScale = ImVec2(1, 1); io.DeltaTime = 1.0f / 60;
    ImGui_ImplVulkan_NewFrame(); ImGui::NewFrame();
    ImGui::GetBackgroundDrawList()->AddImage(ImTextureRef((ImTextureID)texture_), ImVec2(0, 0), io.DisplaySize);
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
