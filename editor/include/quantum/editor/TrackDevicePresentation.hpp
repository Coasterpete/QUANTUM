#pragma once
#include <quantum/editor/CenterlineVisualization.hpp>

namespace quantum::editor
{
    // Display-only lines above the solved centerline and crosses at the exact
    // station endpoints. Uses native station/coordinate conventions, including
    // wrapped circuit intervals; does not change track geometry or device data.
    [[nodiscard]] std::vector<renderer::LineVertex> createTrackDeviceSelectionVertices(
        const coaster::AuthoredTrack& track, const CenterlineVisualization& geometry,
        coaster::TrackDeviceId selected);
}
