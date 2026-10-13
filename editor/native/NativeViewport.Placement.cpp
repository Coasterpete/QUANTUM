#include "NativeViewport.hpp"
#include <SDL3/SDL.h>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <utility>

using namespace quantum::editor;

void NativeViewport::capturePlacement(uint64_t revision, uint64_t selectionRevision, uint64_t presentationRevision,
    bool active, bool selected, StartPoseTransformMode mode,
    const quantum::coaster::AuthoredStartPose& authored,
    std::optional<quantum::coaster::AuthoredStartPose> candidate, double scale,
    const CenterlineVisualization& geometry)
{
    if (pointerTransform_ && (revision != placementRevision_ || selectionRevision != placementSelectionRevision_
        || !active || !selected || mode != transformMode_)) cancelTransformInput();
    const bool changed = revision != placementRevision_ || presentationRevision != placementPresentationRevision_;
    placementRevision_ = revision; placementSelectionRevision_ = selectionRevision;
    placementPresentationRevision_ = presentationRevision;
    placementActive_ = active; startSelected_ = selected; transformMode_ = mode;
    displayedStartPose_ = candidate.value_or(authored);
    coordinateScale_ = scale; placementPreview_ = candidate.has_value();
    if (!changed) return;
    placementGhost_.clear();
    if (candidate && *candidate != authored && geometry.samples.size() > 1)
    {
        // A display-only rigid ghost from cached native samples. Force-driven
        // geometry can differ after regeneration under world gravity; never
        // publish this approximation as authored/generated track geometry.
        const auto rotation = candidate->orientation * glm::conjugate(authored.orientation);
        const auto point = [&](std::size_t i) -> quantum::renderer::LineVertex {
            const auto p = candidate->position + rotation * (geometry.samples[i].position - authored.position);
            return {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z), {1.0F, .72F, .2F, 1.0F}};
        };
        const auto stride = std::max<std::size_t>(1, (geometry.samples.size() - 1 + 511) / 512);
        for (std::size_t i = 0; i + 1 < geometry.samples.size(); i += stride)
        { placementGhost_.push_back(point(i)); placementGhost_.push_back(point(std::min(i + stride, geometry.samples.size() - 1))); }
    }
    trainDirty_ = true;
}
std::optional<NativeViewport::TrackStartPickIntent> NativeViewport::takeTrackStartPick() noexcept
{ return std::exchange(startPick_, std::nullopt); }
QnTransformIntent NativeViewport::takeTransformIntent() noexcept
{ return std::exchange(transformIntent_, QnTransformIntent{}); }
void NativeViewport::cancelTransformInput() noexcept
{
    if (transformIntent_.id) transformIntent_.phase = 3;
    if (pointerTransform_)
    {
        transformIntent_ = pointerTransform_->intent; transformIntent_.phase = 3;
        pointerTransform_.reset();
        if (!SDL_CaptureMouse(false)) SDL_LogWarn(SDL_LOG_CATEGORY_INPUT, "Transform release failed: %s", SDL_GetError());
    }
    hoveredAxis_.reset(); startHovered_ = false;
}
void NativeViewport::queueTransform(uint32_t phase, const glm::dvec2& pointer)
{
    if (!pointerTransform_) return;
    auto& drag = *pointerTransform_;
    if (drag.intent.mode == static_cast<uint32_t>(StartPoseTransformMode::Move))
        drag.intent.amount = glm::dot(pointer - drag.start, drag.direction) * drag.metersPerPixel;
    else
        drag.intent.amount = std::remainder(std::atan2(pointer.y - drag.origin.y, pointer.x - drag.origin.x)
            - drag.initialAngle, 2 * std::acos(-1.0));
    drag.intent.phase = phase; transformIntent_ = drag.intent;
    if (phase == 2)
    {
        pointerTransform_.reset();
        if (!SDL_CaptureMouse(false)) SDL_LogWarn(SDL_LOG_CATEGORY_INPUT, "Transform release failed: %s", SDL_GetError());
    }
}
void NativeViewport::placementPointer(uint32_t phase, const glm::dvec2& pointer, double pixelScale)
{
    pixelScale_ = pixelScale;
    if (phase == 3) { cancelTransformInput(); return; }
    if (pointerTransform_)
    { if (phase != 0) queueTransform(phase, pointer); return; }
    if (!placementActive_) return;
    const auto handles = viewportTransformHandles(camera_, displayedStartPose_.position, width_, height_, pixelScale);
    if (!handles) { startHovered_ = false; hoveredAxis_.reset(); return; }
    startHovered_ = glm::length(pointer - handles->origin) <= 12 * pixelScale;
    hoveredAxis_ = startSelected_ ? pickViewportTransformHandle(*handles, transformMode_, pointer) : std::nullopt;
    if (phase != 0) return; // 0 pointer down, 1 move, 2 release, 3 cancel
    if (hoveredAxis_)
    {
        if (!SDL_CaptureMouse(true))
        { SDL_LogWarn(SDL_LOG_CATEGORY_INPUT, "Transform capture failed: %s", SDL_GetError()); return; }
        const auto& handle = handles->move[static_cast<unsigned>(*hoveredAxis_)];
        QnTransformIntent intent{nextTransformIntent_++, placementRevision_, placementSelectionRevision_, 1, 0,
            static_cast<uint32_t>(transformMode_), static_cast<uint32_t>(*hoveredAxis_), 0};
        pointerTransform_ = PointerTransform{intent, pointer, handles->origin, handle.direction,
            handle.coordinateUnitsPerPixel * coordinateScale_, std::atan2(pointer.y - handles->origin.y, pointer.x - handles->origin.x)};
        transformIntent_ = intent;
    }
    else if (startHovered_)
    { startSelected_ = true; startPick_ = TrackStartPickIntent{placementRevision_, placementSelectionRevision_}; }
}
void NativeViewport::drawPlacementOverlay()
{
    const auto h = viewportTransformHandles(camera_, displayedStartPose_.position, width_, height_, pixelScale_);
    if (!h) return;
    const auto screen = [](glm::dvec2 p) { return ImVec2(static_cast<float>(p.x), static_cast<float>(p.y)); };
    auto* draw = ImGui::GetForegroundDrawList();
    const auto origin = screen(h->origin);
    const auto marker = startSelected_ && placementActive_ ? IM_COL32(255, 201, 74, 255)
        : startHovered_ ? IM_COL32(255, 240, 170, 255) : IM_COL32(136, 201, 255, 230);
    const float radius = static_cast<float>(8 * pixelScale_);
    draw->AddCircleFilled(origin, radius + 2, IM_COL32(20, 25, 34, 255), 20);
    draw->AddQuad(ImVec2(origin.x, origin.y - radius), ImVec2(origin.x + radius, origin.y),
        ImVec2(origin.x, origin.y + radius), ImVec2(origin.x - radius, origin.y), marker, 2);
    draw->AddCircleFilled(origin, 2, marker, 12);
    draw->AddText(ImVec2(origin.x + radius + 5, origin.y + radius + 5), marker,
        placementPreview_ ? "Track Start - preview (release regenerates)" : "Track Start");
    if (!startSelected_ || !placementActive_) return;
    constexpr ImU32 colors[]{IM_COL32(245, 99, 99, 255), IM_COL32(104, 221, 137, 255), IM_COL32(99, 169, 255, 255)};
    constexpr const char* names[]{"X", "Y", "Z"};
    for (unsigned i = 0; i < 3; ++i)
    {
        const bool active = pointerTransform_ && pointerTransform_->intent.axis == i;
        const bool hover = hoveredAxis_ && static_cast<unsigned>(*hoveredAxis_) == i;
        const auto color = active || hover ? IM_COL32(255, 228, 116, 255) : colors[i];
        if (transformMode_ == StartPoseTransformMode::Move)
        {
            const auto a = screen(h->move[i].begin), b = screen(h->move[i].end);
            draw->AddLine(a, b, IM_COL32(20, 25, 34, 255), active ? 7 : 5);
            draw->AddLine(a, b, color, active || hover ? 4 : 2.5F);
            draw->AddCircleFilled(b, static_cast<float>((active || hover ? 6 : 4) * pixelScale_), color, 16);
            draw->AddText(ImVec2(b.x + 6, b.y - 6), color, names[i]);
        }
        else if (transformMode_ == StartPoseTransformMode::Rotate)
        {
            const auto r = static_cast<float>(h->rotateRadii[i]);
            draw->AddCircle(origin, r, IM_COL32(20, 25, 34, 255), 64, 5);
            draw->AddCircle(origin, r, color, 64, active || hover ? 4 : 2.5F);
            draw->AddText(ImVec2(origin.x + r + 5, origin.y - 6), color, names[i]);
        }
    }
}
