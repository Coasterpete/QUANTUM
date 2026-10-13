#include <quantum/editor/TrackDevicePresentation.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

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
    std::vector<renderer::LineVertex> createHardwareMechanismVertices(
        const coaster::AuthoredTrack& track, const CenterlineVisualization& geometry,
        coaster::TrackDeviceId selected, const physics::HardwareRuntime* runtime)
    {
        std::vector<renderer::LineVertex> result;
        auto device = std::find_if(track.trackDevices().devices.begin(), track.trackDevices().devices.end(),
            [selected](const auto& d) { return d.id == selected; });
        if (device == track.trackDevices().devices.end() || geometry.samples.size() < 2) return result;
        const double scale = track.physicalSettings().metersPerCoordinateUnit;
        const std::array<float, 4> color{.2f, .9f, .85f, 1};
        const auto line = [&](const glm::dvec3& a, const glm::dvec3& b)
        {
            for (const auto& p : {a, b}) result.push_back({static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z), color});
        };
        struct Anchor { glm::dvec3 position, tangent, lateral, up; };
        const auto anchor = [&](double station) -> Anchor
        {
            auto next = std::lower_bound(geometry.samples.begin(), geometry.samples.end(), station / scale,
                [](const auto& sample, double distance) { return sample.distance < distance; });
            if (next == geometry.samples.begin()) ++next;
            if (next == geometry.samples.end()) --next;
            const auto& a = *(next - 1); const auto& b = *next;
            const double t = std::clamp((station / scale - a.distance) / (b.distance - a.distance), 0.0, 1.0);
            return {a.position * (1 - t) + b.position * t,
                glm::normalize(a.frame.tangent * (1 - t) + b.frame.tangent * t),
                glm::normalize(a.frame.lateral * (1 - t) + b.frame.lateral * t),
                glm::normalize(a.frame.up * (1 - t) + b.frame.up * t)};
        };
        if (const auto* station = std::get_if<coaster::StationConfiguration>(&device->configuration))
        {
            const auto a = anchor(station->stopStationMeters);
            line(a.position - a.lateral / scale, a.position + a.lateral / scale);
            line(a.position, a.position + a.up * (2 / scale));
        }
        if (const auto* transfer = std::get_if<coaster::TransferConfiguration>(&device->configuration))
        {
            const auto a = anchor(device->startStationMeters), b = anchor(device->endStationMeters);
            double position = 0;
            for (const auto& alignment : transfer->alignments)
            {
                if (alignment.id == transfer->homeAlignment) position = alignment.lateralOffsetMeters;
                const auto p = a.position + a.lateral * (alignment.lateralOffsetMeters / scale);
                line(p - a.up / scale, p + a.up / scale);
                line(p, b.position + b.lateral * (alignment.lateralOffsetMeters / scale));
            }
            if (runtime) if (const auto* t = std::get_if<physics::TransferRuntime>(&runtime->mechanism)) position = t->positionMeters;
            std::array<glm::dvec3, 4> corners{a.position + a.lateral * ((position - 1) / scale),
                a.position + a.lateral * ((position + 1) / scale), b.position + b.lateral * ((position + 1) / scale),
                b.position + b.lateral * ((position - 1) / scale)};
            for (std::size_t i = 0; i < 4; ++i) line(corners[i], corners[(i + 1) % 4]);
        }
        if (const auto* gate = std::get_if<coaster::AccessConfiguration>(&device->configuration))
        {
            const auto a = anchor(device->startStationMeters);
            double fraction = gate->initiallyOpen ? 1 : 0;
            if (runtime) if (const auto* g = std::get_if<physics::AccessRuntime>(&runtime->mechanism)) fraction = g->fraction;
            const double yaw = gate->yawDegrees * std::numbers::pi / 180;
            const double direction = static_cast<int>(gate->travelDirection);
            auto pivot = a.position + (a.lateral * gate->lateralOffsetMeters + a.up * gate->verticalOffsetMeters) / scale;
            if (gate->motion == coaster::AccessMotion::Slide)
                pivot += (a.lateral * std::cos(yaw) + a.tangent * std::sin(yaw)) * (direction * fraction * gate->travelDistanceMeters / scale);
            const double angle = yaw + (gate->motion == coaster::AccessMotion::Swing ? direction * fraction * gate->openAngleDegrees * std::numbers::pi / 180 : 0);
            const auto edge = pivot + (a.lateral * std::cos(angle) + a.tangent * std::sin(angle)) / scale;
            const auto top = a.up * (1.1 / scale);
            line(pivot, edge); line(pivot + top, edge + top); line(pivot, pivot + top); line(edge, edge + top);
            line(pivot, edge + top);
        }
        return result;
    }

}
