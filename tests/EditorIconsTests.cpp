#include <quantum/editor/EditorIcons.hpp>
#include <quantum/editor/EditorStyle.hpp>

#include <imgui.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
    void require(const bool condition, const char* const message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void requireNear(
        const float actual,
        const float expected,
        const float tolerance,
        const char* const message)
    {
        require(std::abs(actual - expected) <= tolerance, message);
    }

    struct RenderedButton
    {
        ImU32 frameColor = 0;
        ImVec2 size{};
        std::vector<ImU32> vertexColors;
    };

    [[nodiscard]] bool containsColor(
        const RenderedButton& button,
        const ImU32 color)
    {
        for (const ImU32 vertexColor : button.vertexColors)
        {
            if (vertexColor == color)
            {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] RenderedButton renderButton(
        const quantum::editor::EditorIcons& icons,
        const quantum::editor::EditorIcon icon,
        const char* const id,
        const bool selected,
        const bool enabled)
    {
        ImDrawList* const drawList = ImGui::GetWindowDrawList();
        const int firstVertex = drawList->VtxBuffer.Size;
        static_cast<void>(icons.button(
            icon,
            id,
            "Icon tooltip",
            selected,
            enabled
        ));
        const int endVertex = drawList->VtxBuffer.Size;
        require(endVertex > firstVertex,
            "A Lucide button produced no ImGui geometry.");

        const ImVec2 minimum = ImGui::GetItemRectMin();
        const ImVec2 maximum = ImGui::GetItemRectMax();
        RenderedButton rendered;
        rendered.frameColor = drawList->VtxBuffer[firstVertex].col;
        rendered.size = {maximum.x - minimum.x, maximum.y - minimum.y};
        rendered.vertexColors.reserve(
            static_cast<std::size_t>(endVertex - firstVertex));
        for (int vertex = firstVertex; vertex < endVertex; ++vertex)
        {
            rendered.vertexColors.push_back(drawList->VtxBuffer[vertex].col);
        }
        return rendered;
    }

    [[nodiscard]] RenderedButton renderInteractiveButton(
        const quantum::editor::EditorIcons& icons,
        const bool mouseDown)
    {
        ImGuiIO& io = ImGui::GetIO();
        io.AddMousePosEvent(20.0F, 20.0F);
        io.AddMouseButtonEvent(ImGuiMouseButton_Left, mouseDown);
        ImGui::NewFrame();
        ImGui::SetNextWindowPos({0.0F, 0.0F});
        ImGui::SetNextWindowSize({640.0F, 140.0F});
        ImGui::Begin(
            "Lucide toolbar test",
            nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
        );
        const RenderedButton rendered = renderButton(
            icons,
            quantum::editor::EditorIcon::Orbit,
            "##Interactive",
            false,
            true
        );
        ImGui::End();
        ImGui::EndFrame();
        return rendered;
    }

    void verifyIconMappings(const std::filesystem::path& basePath)
    {
        using quantum::editor::EditorIcon;
        constexpr std::array expectedFileNames{
            "mouse-pointer-2.svg",
            "move-3d.svg",
            "rotate-3d.svg",
            "orbit.svg",
            "hand.svg",
            "focus.svg",
            "camera.svg",
            "axis-3d.svg",
            "maximize.svg",
            "folder-open.svg",
            "save.svg",
            "undo-2.svg",
            "redo-2.svg",
            "eye.svg",
            "eye-off.svg",
            "lock.svg",
            "lock-open.svg",
            "zoom-in.svg",
            "zoom-out.svg",
            "play.svg",
            "pause.svg",
            "square-stop.svg",
            "settings.svg"
        };
        require(expectedFileNames.size()
                == static_cast<std::size_t>(EditorIcon::Count),
            "The icon mapping test does not cover every editor icon.");

        for (std::size_t index = 0; index < expectedFileNames.size(); ++index)
        {
            const EditorIcon icon = static_cast<EditorIcon>(index);
            require(quantum::editor::editorIconFileName(icon)
                    == expectedFileNames[index],
                "An editor action maps to the wrong Lucide asset.");
            require(std::filesystem::is_regular_file(
                    basePath / "assets/icons/lucide"
                        / expectedFileNames[index]),
                "A mapped Lucide asset was not deployed with the editor.");
        }

        bool rejectedInvalidIcon = false;
        try
        {
            static_cast<void>(quantum::editor::editorIconFileName(
                EditorIcon::Count));
        }
        catch (const std::invalid_argument&)
        {
            rejectedInvalidIcon = true;
        }
        require(rejectedInvalidIcon,
            "The icon mapping accepted the Count sentinel as an icon.");
    }

    void verifyMetricsAtDpi(
        const quantum::editor::EditorIcons& icons,
        const float dpiScale)
    {
        using quantum::editor::EditorIcon;

        ImGui::GetStyle().FontScaleDpi = dpiScale;
        const float expectedIconExtent = std::round(19.0F * dpiScale);
        for (std::size_t index = 0;
            index < static_cast<std::size_t>(EditorIcon::Count);
            ++index)
        {
            const EditorIcon icon = static_cast<EditorIcon>(index);
            const quantum::editor::EditorIconMetrics metrics =
                icons.metrics(icon);
            requireNear(metrics.iconExtent, expectedIconExtent, 0.01F,
                "Lucide icon sizing did not follow the DPI scale.");
            require(metrics.buttonExtent >= metrics.iconExtent,
                "The square icon hit target is smaller than its icon.");
            requireNear(
                (metrics.drawingMinimum.x + metrics.drawingMaximum.x) * 0.5F,
                metrics.buttonExtent * 0.5F,
                0.01F,
                "A Lucide icon is not optically centered horizontally."
            );
            requireNear(
                (metrics.drawingMinimum.y + metrics.drawingMaximum.y) * 0.5F,
                metrics.buttonExtent * 0.5F,
                0.01F,
                "A Lucide icon is not optically centered vertically."
            );
        }

        const quantum::editor::EditorIconMetrics moveMetrics =
            icons.metrics(EditorIcon::Move);
        require(moveMetrics.strokeThickness >= 2.0F * dpiScale - 0.01F,
            "Default Lucide strokes became too thin at desktop DPI scales.");

        quantum::editor::EditorIconRenderStyle sourceWeight;
        sourceWeight.strokeThicknessMultiplier = 1.0F;
        quantum::editor::EditorIconRenderStyle heavierWeight;
        heavierWeight.strokeThicknessMultiplier = 1.4F;
        require(
            icons.metrics(EditorIcon::Move, heavierWeight).strokeThickness
                > icons.metrics(EditorIcon::Move, sourceWeight).strokeThickness,
            "The Lucide stroke-thickness multiplier did not affect rendering."
        );
    }

    void renderAndVerifyStates(
        const quantum::editor::EditorIcons& icons,
        const float dpiScale)
    {
        ImGui::GetStyle().FontScaleDpi = dpiScale;
        ImGuiIO& io = ImGui::GetIO();
        io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
        ImGui::NewFrame();
        ImGui::SetNextWindowPos({0.0F, 0.0F});
        ImGui::SetNextWindowSize({640.0F, 140.0F});
        ImGui::Begin(
            "Lucide toolbar test",
            nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
        );

        const RenderedButton normal = renderButton(
            icons,
            quantum::editor::EditorIcon::Move,
            "##Normal",
            false,
            true
        );
        ImGui::SameLine();
        const RenderedButton selected = renderButton(
            icons,
            quantum::editor::EditorIcon::Rotate,
            "##Selected",
            true,
            true
        );
        ImGui::SameLine();
        const RenderedButton disabled = renderButton(
            icons,
            quantum::editor::EditorIcon::Play,
            "##Disabled",
            false,
            false
        );

        requireNear(normal.size.x, normal.size.y, 0.01F,
            "A normal icon button does not have a square hit target.");
        requireNear(selected.size.x, selected.size.y, 0.01F,
            "A selected icon button does not have a square hit target.");
        requireNear(disabled.size.x, disabled.size.y, 0.01F,
            "A disabled icon button does not have a square hit target.");
        requireNear(normal.size.x, selected.size.x, 0.01F,
            "Icon button state changed the hit-target size.");
        requireNear(normal.size.x, disabled.size.x, 0.01F,
            "Disabled icon buttons do not retain the standard hit target.");

        require(normal.frameColor == ImGui::GetColorU32(
            quantum::editor::palette::control),
            "Normal Lucide buttons do not use the standard control surface.");
        require(containsColor(normal, ImGui::GetColorU32(
            quantum::editor::palette::text)),
            "Normal Lucide icons do not use the readable text color.");
        require(selected.frameColor == ImGui::GetColorU32(
            quantum::editor::palette::selection),
            "Selected Lucide buttons lost the QUANTUM selection surface.");
        require(containsColor(selected, ImGui::GetColorU32(
            quantum::editor::palette::accent)),
            "Selected Lucide icons lost the QUANTUM accent.");
        require(disabled.frameColor == ImGui::GetColorU32(
            quantum::editor::palette::surfaceInset),
            "Disabled Lucide buttons are not visually distinct.");
        require(containsColor(disabled, ImGui::GetColorU32(
            quantum::editor::palette::textSecondary)),
            "Disabled Lucide icons do not use the muted readable tint.");
        require(disabled.frameColor != normal.frameColor
                && selected.frameColor != normal.frameColor,
            "Disabled and selected Lucide states are not distinct from normal."
        );

        ImGui::End();
        ImGui::EndFrame();

        const RenderedButton hovered = renderInteractiveButton(icons, false);
        const RenderedButton active = renderInteractiveButton(icons, true);
        static_cast<void>(renderInteractiveButton(icons, false));
        require(hovered.frameColor == ImGui::GetColorU32(
            quantum::editor::palette::controlHovered),
            "Hovered Lucide buttons do not use the hover surface.");
        require(containsColor(hovered, ImGui::GetColorU32(
            quantum::editor::palette::accent)),
            "Hovered Lucide icons do not use the accent tint.");
        require(active.frameColor == ImGui::GetColorU32(
            quantum::editor::palette::selectionActive),
            "Held Lucide buttons do not use the active surface.");
        require(containsColor(active, ImGui::GetColorU32(
            quantum::editor::palette::accent)),
            "Held Lucide icons do not use the accent tint.");
    }
}

int main(const int argumentCount, const char* const* arguments)
{
    try
    {
        require(argumentCount == 2, "Expected the editor asset directory.");

        verifyIconMappings(std::filesystem::path{arguments[1]});
        quantum::editor::EditorIcons icons;
        icons.load(std::filesystem::path{arguments[1]});
        require(icons.loaded(), "The bundled Lucide icon set did not load.");

        ImGui::CreateContext();
        try
        {
            ImGuiIO& io = ImGui::GetIO();
            io.IniFilename = nullptr;
            io.DisplaySize = {640.0F, 160.0F};
            ImGuiStyle& style = ImGui::GetStyle();
            style.FontSizeBase = 14.0F;
            style.FramePadding = {5.0F, 4.0F};
            style.FrameBorderSize = 1.0F;
            io.Fonts->AddFontDefault();
            unsigned char* pixels = nullptr;
            int width = 0;
            int height = 0;
            io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
            require(pixels != nullptr && width > 0 && height > 0,
                "The ImGui test atlas did not build.");
            for (const float dpiScale : {1.0F, 1.25F, 1.5F})
            {
                verifyMetricsAtDpi(icons, dpiScale);
                renderAndVerifyStates(icons, dpiScale);
            }
        }
        catch (...)
        {
            ImGui::DestroyContext();
            throw;
        }
        ImGui::DestroyContext();

        icons.clear();
        require(!icons.loaded(), "Clearing Lucide icons did not reset them.");
    }
    catch (const std::exception& error)
    {
        std::cerr << "Editor icon test failed: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
