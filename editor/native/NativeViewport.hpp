#pragma once
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/ViewportCamera.hpp>
#include <quantum/editor/SupportVisualization.hpp>
#include <quantum/renderer/VulkanContext.hpp>
#include <imgui.h>
#include <thread>

// One concrete hosted presenter; the existing renderer still owns every GPU
// resource. This object owns its SDL wrapper and ImGui texture registration.
class NativeViewport
{
public:
    NativeViewport() = default;
    ~NativeViewport();
    NativeViewport(const NativeViewport&) = delete;
    NativeViewport& operator=(const NativeViewport&) = delete;
    void open(uint32_t platform, uint64_t window,
        const quantum::editor::CenterlineVisualization& geometry);
    bool frame(uint32_t width, uint32_t height,
        const quantum::editor::CenterlineVisualization* geometry,
        const quantum::editor::SupportVisualization* supports,
        const quantum::coaster::GroundAppearance* ground, bool frameTrack);
    void captureTrain(uint64_t revision, uint64_t generation,
        std::span<const quantum::renderer::LineVertex> train,
        std::span<const quantum::renderer::StaticMeshInstance> meshes);
    void captureDevice(uint64_t revision, uint64_t selected,
        const quantum::coaster::AuthoredTrack& track,
        const quantum::editor::CenterlineVisualization& geometry);
    struct SupportPickIntent
    {
        uint64_t revision, selectionRevision;
        std::optional<quantum::editor::SupportSelection> selection;
    };
    void captureSupports(uint64_t revision, uint64_t selectionRevision, bool active,
        std::optional<quantum::editor::SupportSelection> selection,
        const quantum::coaster::SupportCollection& supports,
        const quantum::editor::SupportVisualization& visualization);
    [[nodiscard]] std::optional<SupportPickIntent> takeSupportPick() noexcept;
    void input(uint32_t action, double x, double y, double z, double value);
    void requireOwnerThread() const;
    uint32_t takeShortcuts() noexcept;
    void setPresentation(uint32_t mode);
private:
    void close() noexcept;
    static void retireTexture(void* self) noexcept;
    void drawSupportOverlay();
    void frameSupportSelection();
    [[nodiscard]] std::optional<quantum::editor::SupportSelection> pickSupportAt(
        double normalizedX, double normalizedY, double pixelScale = 1) const;
    SDL_Window* window_ = nullptr;
    ImGuiContext* imgui_ = nullptr;
    bool video_ = false;
    bool backend_ = false;
    bool hosted_ = false;
    uint32_t shortcuts_ = 0;
    VkDescriptorSet texture_ = VK_NULL_HANDLE;
    quantum::renderer::VulkanContext renderer_;
    quantum::editor::ViewportCamera camera_;
    std::thread::id owner_;
    uint32_t width_ = 800, height_ = 500;
    std::uint64_t swapchainGeneration_ = 0;
    uint64_t trainRevision_ = 0, trainGeneration_ = 0;
    bool trainDirty_ = false;
    std::vector<quantum::renderer::LineVertex> train_;
    std::vector<quantum::renderer::LineVertex> device_, lines_;
    uint64_t deviceRevision_ = 0, selectedDevice_ = 0;
    std::vector<quantum::renderer::StaticMeshInstance> meshes_;
    quantum::editor::SupportVisualization supportPicking_;
    std::vector<uint32_t> supportAnchors_;
    std::vector<std::pair<glm::dvec3, quantum::editor::SupportSelection>> supportConnections_;
    std::optional<quantum::editor::SupportSelection> selectedSupport_, hoveredSupport_;
    std::optional<SupportPickIntent> supportPick_;
    uint64_t supportRevision_ = 0, supportSelectionRevision_ = 0;
    bool supportActive_ = false;
};
