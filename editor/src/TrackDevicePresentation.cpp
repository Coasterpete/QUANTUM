#include <quantum/editor/TrackDevicePresentation.hpp>
#include <algorithm>
#include <array>

namespace quantum::editor
{
    std::vector<renderer::LineVertex> createTrackDeviceSelectionVertices(
        const coaster::AuthoredTrack& track, const CenterlineVisualization& geometry,
        coaster::TrackDeviceId selected)
    {
        std::vector<renderer::LineVertex> vertices;
        const auto& devices = track.trackDevices().devices;
        const auto device = std::find_if(devices.begin(), devices.end(),
            [selected](const auto& value) { return value.id == selected; });
        if (device == devices.end() || geometry.samples.size() < 2) return vertices;
        const double scale = track.physicalSettings().metersPerCoordinateUnit;
        constexpr double intervalLiftMeters = .35;
        constexpr double endpointHalfSizeMeters = .6;
        const double length = track.trackLengthMeters();
        const bool wraps = track.layoutMode() == coaster::LayoutMode::Circuit
            && device->startStationMeters > device->endStationMeters;
        const std::array<float, 4> color = device->enabled
            ? std::array<float, 4>{1.0f, .65f, .15f, 1.0f} : std::array<float, 4>{.65f, .65f, .65f, 1.0f};
        const auto line = [&](const glm::dvec3& a, const glm::dvec3& b)
        {
            for (const auto& p : {a, b})
                vertices.push_back({static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z), color});
        };
        for (std::size_t i = 1; i < geometry.samples.size(); ++i)
        {
            const auto& a = geometry.samples[i - 1]; const auto& b = geometry.samples[i];
            const double start = a.distance * scale, end = b.distance * scale;
            if (end <= start) continue;
            const auto position = [&](double station)
            {
                const double t = (station - start) / (end - start);
                return a.position * (1 - t) + b.position * t
                    + (a.frame.up * (1 - t) + b.frame.up * t) * (intervalLiftMeters / scale);
            };
            const auto interval = [&](double first, double last)
            {
                const auto lo = std::max(start, first), hi = std::min(end, last);
                if (hi > lo) line(position(lo), position(hi));
            };
            if (wraps) { interval(device->startStationMeters, length); interval(0, device->endStationMeters); }
            else interval(device->startStationMeters, device->endStationMeters);
            for (double station : {device->startStationMeters, device->endStationMeters})
                if (station >= start && (station < end || (i + 1 == geometry.samples.size() && station == end)))
                {
                    const auto p = position(station);
                    const auto across = a.frame.lateral * (endpointHalfSizeMeters / scale);
                    const auto up = a.frame.up * (endpointHalfSizeMeters / scale);
                    line(p - across, p + across); line(p - up, p + up);
                }
        }
        return vertices;
    }
}
