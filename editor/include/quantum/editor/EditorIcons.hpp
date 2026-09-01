#pragma once

#include <imgui.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string_view>
#include <vector>

namespace quantum::editor
{
    enum class EditorIcon : std::uint8_t
    {
        Select,
        Move,
        Rotate,
        Orbit,
        Pan,
        Focus,
        Camera,
        Axis,
        Maximize,
        Open,
        Save,
        Undo,
        Redo,
        Eye,
        EyeOff,
        Lock,
        Unlock,
        ZoomIn,
        ZoomOut,
        Play,
        Pause,
        Stop,
        Settings,
        Count
    };

    [[nodiscard]] std::string_view editorIconFileName(EditorIcon icon);

    struct EditorIconRenderStyle
    {
        // Logical size before Dear ImGui's current presentation/DPI scale.
        float iconSize = 19.0F;
        float strokeThicknessMultiplier = 1.2F;
    };

    struct EditorIconMetrics
    {
        float iconExtent = 0.0F;
        float buttonExtent = 0.0F;
        float strokeThickness = 0.0F;
        ImVec2 drawingMinimum{};
        ImVec2 drawingMaximum{};
    };

    // Editor-only Lucide renderer. SVG strokes are flattened once at load
    // time, then submitted through Dear ImGui's existing draw lists.
    class EditorIcons
    {
    public:
        void load(const std::filesystem::path& basePath);
        void clear() noexcept;

        // Uses QUANTUM state colors with the current ImGui sizing/rounding.
        [[nodiscard]] bool button(
            EditorIcon icon,
            const char* id,
            const char* tooltip,
            bool selected = false,
            bool enabled = true,
            const EditorIconRenderStyle& renderStyle = {}
        ) const;

        [[nodiscard]] EditorIconMetrics metrics(
            EditorIcon icon,
            const EditorIconRenderStyle& renderStyle = {}
        ) const;

        [[nodiscard]] bool loaded() const noexcept;

    private:
        struct Stroke
        {
            std::vector<ImVec2> points;
            bool closed = false;
        };

        struct Drawing
        {
            std::vector<Stroke> strokes;
            ImVec2 viewBoxMinimum{};
            ImVec2 viewBoxSize{};
            ImVec2 contentMinimum{};
            ImVec2 contentMaximum{};
            float strokeWidth = 0.0F;
        };

        static constexpr std::size_t iconCount =
            static_cast<std::size_t>(EditorIcon::Count);

        std::array<Drawing, iconCount> drawings_{};
        bool loaded_ = false;
    };
}
