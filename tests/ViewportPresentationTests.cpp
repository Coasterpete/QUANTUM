#include <quantum/editor/EditorUi.hpp>
#include <quantum/renderer/ViewportAids.hpp>

#include <imgui.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace quantum::editor;
    constexpr double radiansPerDegree = 3.14159265358979323846 / 180.0;

    void require(const bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    void presetSurvivesSettings()
    {
        ViewportCamera camera;
        camera.setBounds({10.0, -5.0, 20.0}, {110.0, 30.0, 60.0});
        camera.frame(2.0);
        ViewportSettings settings;
        settings.fieldOfViewDegrees = 53.0F;
        for (const bool orthographic : {false, true})
        for (const auto preset : {ViewportCameraPreset::Perspective,
            ViewportCameraPreset::Isometric, ViewportCameraPreset::Top,
            ViewportCameraPreset::Bottom, ViewportCameraPreset::Left,
            ViewportCameraPreset::Right})
        {
            settings.orthographic = orthographic;
            const auto focus = camera.focus();
            const auto distance = camera.distance();
            settings.applyCameraPreset(camera, preset);
            const bool expectedOrtho = preset == ViewportCameraPreset::Isometric
                || (preset != ViewportCameraPreset::Perspective && orthographic);
            require(settings.orthographic == expectedOrtho, "preset must synchronize the settings UI");
            const auto matrix = camera.viewProjection(2.0);
            for (int frame = 0; frame < 3; ++frame)
            {
                settings.applyCameraSettings(camera);
                require(camera.viewProjection(2.0) == matrix,
                    "per-frame settings must not overwrite preset projection");
            }
            require((camera.projection() == ViewportProjection::Orthographic) == expectedOrtho,
                "canonical presets choose projection; axis presets preserve the user choice");
            require(camera.focus() == focus && camera.distance() == distance,
                "preset must preserve focus and distance");
            require(std::abs(camera.verticalFieldOfView() - 53.0 * radiansPerDegree) < 1e-12,
                "preset must preserve user FOV");
        }
        settings.applyCameraPreset(camera, ViewportCameraPreset::Isometric);
        const double yaw = camera.yaw(), pitch = camera.pitch();
        settings.orthographic = false;
        settings.applyCameraSettings(camera);
        require(camera.projection() == ViewportProjection::Perspective,
            "explicit user projection edit after a preset must win");
        require(camera.yaw() == yaw && camera.pitch() == pitch, "settings must not reset orbit");
    }

    void trackPresentationState()
    {
        using quantum::renderer::TrackPresentationMode;
        using quantum::renderer::TrackPresentationState;

        TrackPresentationState state;
        require(state.mode() == TrackPresentationMode::Shaded,
            "viewport track presentation defaults to shaded");
        for (const TrackPresentationMode mode : {
            TrackPresentationMode::Shaded,
            TrackPresentationMode::Wireframe,
            TrackPresentationMode::ShadedWireframe,
            TrackPresentationMode::CenterlineDebug})
        {
            require(quantum::renderer::isValidTrackPresentationMode(mode),
                "declared presentation mode is valid");
            state.setMode(mode);
            require(state.mode() == mode,
                "presentation mode change is retained without geometry state");
        }

        bool rejected = false;
        try
        {
            state.setMode(static_cast<TrackPresentationMode>(255));
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected, "invalid presentation mode is rejected");

        ViewportSettings settings;
        require(settings.trackPresentation.mode()
                == TrackPresentationMode::Shaded,
            "viewport settings use the product-facing shaded default");
    }

    void translatedGrid()
    {
        using namespace quantum::renderer;
        for (const auto center : {std::array{123.0F, -47.0F},
            std::array{-83.0F, 217.0F}, std::array{0.0F, 0.0F}})
        for (const float spacing : {1.0F, 2.0F, 5.0F, 10.0F, 20.0F, 50.0F, 100.0F, 200.0F})
        {
            const auto vertices = createViewportAidVertices(center[0], center[1], spacing);
            require(vertices.size() == viewportAidVertexCount, "grid buffer size must stay fixed");
            const float x = std::round(center[0] / spacing) * spacing;
            const float y = std::round(center[1] / spacing) * spacing;
            const float extent = 20.0F * spacing;
            for (int line = -20; line <= 20; ++line)
            {
                const auto index = static_cast<std::size_t>(line + 20) * 4;
                const auto& h0 = vertices[index];
                const auto& h1 = vertices[index + 1];
                const auto& v0 = vertices[index + 2];
                const auto& v1 = vertices[index + 3];
                require(h0.x == x - extent && h1.x == x + extent
                    && h0.y == y + line * spacing && h1.y == h0.y,
                    "horizontal lines must step around center Y and span center X");
                require(v0.y == y - extent && v1.y == y + extent
                    && v0.x == x + line * spacing && v1.x == v0.x,
                    "vertical lines must step around center X and span center Y");
                require(h0.z == 0 && h1.z == 0 && v0.z == 0 && v1.z == 0,
                    "grid must remain on world ground");
                const float expected = std::fmod(std::round(h0.y / spacing), 5.0F) == 0 ? 0.026F : 0.012F;
                require(h0.color[0] == expected, "Phase 2 grid contrast must survive translation");
            }
            const auto& origin = vertices[vertices.size() - 6];
            require(origin.x == x && origin.y == y && origin.z == 0, "axis origin must share grid center");
            require(vertices.back().z == 25.0F, "reference axis extent must remain unchanged");
        }
    }

    void framingMatrix()
    {
        for (const double length : {2.0, 60.0, 1200.0})
        for (const glm::dvec3 origin : {glm::dvec3{0.0}, glm::dvec3{125.0, -73.0, 0.0},
            glm::dvec3{-210.0, 137.0, 90.0}})
        for (const double aspect : {0.4, 1.0, 16.0 / 9.0, 5.0})
        for (const auto preset : {ViewportCameraPreset::Perspective, ViewportCameraPreset::Isometric,
            ViewportCameraPreset::Top, ViewportCameraPreset::Bottom,
            ViewportCameraPreset::Left, ViewportCameraPreset::Right})
        {
            ViewportCamera camera;
            const auto maximum = origin + glm::dvec3{length, 1.2, 1.4};
            camera.setBounds(origin, maximum);
            camera.applyPreset(preset);
            camera.frame(aspect);
            const double yaw = camera.yaw(), pitch = camera.pitch();
            require(camera.focus() == (origin + maximum) * 0.5, "Frame All centers bounds");
            if (preset == ViewportCameraPreset::Perspective)
            {
                require(pitch >= 35.0 * radiansPerDegree && pitch <= 40.0 * radiansPerDegree,
                    "product elevation must stay modest");
                require(std::abs(camera.verticalFieldOfView() - 45.0 * radiansPerDegree) < 1e-12,
                    "composition must retain 45 degree FOV");
            }
            for (int corner = 0; corner < 8; ++corner)
            {
                const glm::dvec3 point{corner & 1 ? maximum.x : origin.x,
                    corner & 2 ? maximum.y : origin.y, corner & 4 ? maximum.z : origin.z};
                const auto projected = projectViewportPoint(camera, point, aspect);
                require(projected && projected->normalizedPosition.x >= 0
                    && projected->normalizedPosition.x <= 1 && projected->normalizedPosition.y >= 0
                    && projected->normalizedPosition.y <= 1, "all AABB corners must fit at every length, offset and aspect");
            }
            require(camera.frameBounds(origin, origin + (maximum - origin) * 0.25, aspect),
                "Focus must frame a smaller selection");
            require(camera.yaw() == yaw && camera.pitch() == pitch, "Focus must preserve orientation");
            camera.orbit(0.17, -2.0);
            require(camera.pitch() < 0, "no ground-plane orbit floor");
            const auto eye = camera.position();
            const auto focus = camera.focus();
            const double userYaw = camera.yaw(), userPitch = camera.pitch();
            camera.setBounds(origin - glm::dvec3{5.0}, maximum + glm::dvec3{5.0});
            require(camera.position() == eye && camera.focus() == focus
                && camera.yaw() == userYaw && camera.pitch() == userPitch,
                "ordinary geometry updates must never reset user angles or pose");
        }
    }

    void actualReferenceCurvesFit()
    {
        for (const double length : {2.0, 60.0, 1200.0})
        for (const glm::dvec3 origin : {glm::dvec3{0.0}, glm::dvec3{125.0, -73.0, 90.0}})
        {
            auto track = quantum::coaster::createNewDocument();
            quantum::coaster::setSectionLength(track.section(0), length);
            track.setStartPose({origin, {1.0, 0.0, 0.0, 0.0}});
            const auto visualization = createCenterlineVisualization(track);
            require(visualization.minimumPosition == origin,
                "display framing must not change solved centerline bounds");
            for (const double aspect : {0.4, 1.0, 5.0})
            for (const auto* slice : {static_cast<const CenterlineSectionSlice*>(nullptr),
                &visualization.sectionSlices.front()})
            {
                const auto [minimum, maximum] = referenceCurveBounds(visualization, slice);
                ViewportCamera camera;
                camera.setBounds(minimum, maximum);
                camera.frame(aspect);
                for (const auto& vertex : visualization.vertices)
                {
                    const auto point = projectViewportPoint(camera, {vertex.x, vertex.y, vertex.z}, aspect);
                    require(point && point->normalizedPosition.x >= 0 && point->normalizedPosition.x <= 1
                        && point->normalizedPosition.y >= 0 && point->normalizedPosition.y <= 1,
                        "short/long/translated/elevated reference curves must fit Frame All and Focus");
                }
            }
        }
    }

    void fontsAndOverlay(const std::filesystem::path& basePath)
    {
        ImGui::CreateContext();
        try
        {
            applyQuantumStyle();
            const auto fonts = loadEditorFonts(basePath);
            auto& io = ImGui::GetIO();
            io.IniFilename = nullptr;
            io.DisplaySize = {800, 600};
            unsigned char* pixels = nullptr;
            int width = 0, height = 0;
            io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
            require(pixels && width > 0 && height > 0, "static OTF faces must rasterize");
            require(io.FontDefault == fonts.normal && io.Fonts->Fonts.Size == 3,
                "only the three atlas-owned faces must load");
            for (auto* font : {fonts.normal, fonts.header, fonts.technical})
            for (const ImWchar glyph : {ImWchar('0'), ImWchar('9'), ImWchar('+'), ImWchar('-'),
                ImWchar('.'), ImWchar('/'), ImWchar(0x00B0), ImWchar(0x2212), ImWchar(0x2014)})
                require(font->IsGlyphInFont(glyph), "degree, signs, decimal and em dash must not need fallback glyphs");
            for (const float scale : {1.0F, 1.5F, 2.0F})
            {
                ImGui::GetStyle().FontScaleDpi = scale;
                ImGui::NewFrame();
                ImGui::Begin("Typography");
                ImGui::PushFont(fonts.technical, editorTechnicalFontSize);
                require(std::abs(ImGui::CalcTextSize("111.111").x - ImGui::CalcTextSize("888.888").x) < 0.01F,
                    "technical digits must align");
                require(std::abs(ImGui::GetFontSize() - editorTechnicalFontSize * scale) < 0.01F,
                    "font scale must apply exactly once");
                require(viewportStyle::anchorRingRadius * editorPresentationScale()
                    <= viewportTrackAnchorHitRadiusPixels * editorPresentationScale(),
                    "anchor ring must stay within its picking radius at every DPI");
                const auto size = ImGui::CalcTextSize("Anchor 123456");
                const auto position = clampViewportLabel({299, 199}, size, {20, 20}, {300, 200});
                require(position.x + size.x <= 300.01F && position.y + size.y <= 200.01F,
                    "anchor label must fit at bottom/right image edges");
                const auto tiny = clampViewportLabel({-100, -100}, {200, 200}, {10, 10}, {15, 15});
                require(tiny.x == 10 && tiny.y == 10, "tiny image must not invert clamp bounds");
                ImGui::PopFont();
                ImGui::End();
                ImGui::EndFrame();
            }
        }
        catch (...) { ImGui::DestroyContext(); throw; }
        ImGui::DestroyContext();
        ImGui::CreateContext();
        bool missingRejected = false;
        try { static_cast<void>(loadEditorFonts(basePath / "missing-font-fixture")); }
        catch (const std::runtime_error& error)
        {
            missingRejected = std::string(error.what()).find("overpass-regular.otf") != std::string::npos;
        }
        ImGui::DestroyContext();
        require(missingRejected, "missing font must retain a useful explicit startup error");
    }
}

int main(int argc, char** argv)
{
    try
    {
        require(argc == 2, "pass the deployed editor asset directory");
        presetSurvivesSettings();
        trackPresentationState();
        translatedGrid();
        framingMatrix();
        actualReferenceCurvesFit();
        fontsAndOverlay(argv[1]);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Viewport presentation test failure: " << error.what() << '\n';
        return 1;
    }
    std::cout << "Viewport presentation tests passed.\n";
    return 0;
}
