#include <quantum/editor/InfrastructurePresentation.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <limits>
#include <stdexcept>

namespace quantum::editor
{
    namespace
    {
        using namespace coaster;
        void box(SupportSolidPresentation& solid, const glm::dvec3& center,
            const glm::dvec3& x, const glm::dvec3& y, const glm::dvec3& z,
            const glm::dvec3& dimensions, const SupportSurface& surface, float pattern = 0)
        {
            if (!std::isfinite(glm::length(x)) || !std::isfinite(glm::length(y)) || !std::isfinite(glm::length(z))
                || glm::length(x) < 1e-9 || glm::length(y) < 1e-9 || glm::length(z) < 1e-9)
                throw std::invalid_argument("Infrastructure requires a nondegenerate frame.");
            SupportMemberInstance instance;
            instance.transform[0] = glm::vec4{glm::vec3{x*dimensions.x},0};
            instance.transform[1] = glm::vec4{glm::vec3{y*dimensions.y},0};
            instance.transform[2] = glm::vec4{glm::vec3{z*dimensions.z},0};
            instance.transform[3] = glm::vec4{glm::vec3{center},1};
            instance.section = {1,0,pattern,0};
            auto batch = std::ranges::find_if(solid.batches, [&](const auto& b) { return b.surface == surface; });
            if (batch == solid.batches.end())
            {
                solid.batches.push_back({SupportMemberMeshKind::Rectangular, {}, surface});
                batch = solid.batches.end()-1;
            }
            batch->instances.push_back(instance);
        }
        void bar(SupportSolidPresentation& solid, const glm::dvec3& a, const glm::dvec3& b,
            const glm::dvec3& up, double diameter, const SupportSurface& surface)
        {
            const auto frame = resolveSupportMemberFrame(a,b, SupportMemberOrientation::Generic,
                normalizeSupportMemberOrientationReference(up));
            if (frame.length <= minimumSupportMemberLength) return;
            box(solid,frame.origin,frame.axisX,frame.axisY,frame.axisZ,
                {frame.length,diameter,diameter},surface);
        }
        struct DeckFrame { glm::dvec3 p, tangent, lateral, up; };
        DeckFrame frameAt(const AuthoredTrack& track, std::span<const RiderLocalGeometryState> states,
            double stationMeters, double lateralMeters, double verticalMeters)
        {
            const double scale = track.physicalSettings().metersPerCoordinateUnit;
            const auto a = resolveSupportTrackAttachment(states,
                {stationMeters/scale,lateralMeters/scale,verticalMeters/scale});
            return {a.position,a.frame.tangent,a.frame.lateral,a.frame.up};
        }
    }
    std::vector<InfrastructureSolid> createInfrastructurePresentation(const coaster::AuthoredTrack& track,
        std::span<const coaster::RiderLocalGeometryState> states)
    {
        using namespace coaster;
        std::vector<InfrastructureSolid> result;
        const double scale = track.physicalSettings().metersPerCoordinateUnit;
        for (const auto& item : track.trackDevices().infrastructure)
        {
            InfrastructureSolid object{item.id};
            object.solid.metersPerCoordinateUnit = scale;
            const auto material = item.surface.value_or(defaultSupportSurface(SupportMaterialKind::GalvanizedSteel));
            const auto railMaterial = defaultSupportSurface(SupportMaterialKind::PaintedSteel);
            const double lateral = static_cast<int>(item.side)*item.lateralOffsetMeters;
            const double width = item.widthMeters/scale, height = item.railings.heightMeters/scale;
            const double pathLength = item.startStationMeters <= item.endStationMeters
                ? item.endStationMeters-item.startStationMeters
                : track.trackLengthMeters()-item.startStationMeters+item.endStationMeters;
            auto first = item.worldPlacement ? DeckFrame{item.worldStartMeters/scale,{1,0,0},{0,1,0},{0,0,1}}
                : frameAt(track,states,item.startStationMeters,lateral,item.verticalOffsetMeters);
            auto last = item.worldPlacement ? DeckFrame{item.worldEndMeters/scale,{1,0,0},{0,1,0},{0,0,1}}
                : frameAt(track,states,item.endStationMeters,lateral,item.verticalOffsetMeters);
            if (item.kind == InfrastructureKind::Stairs)
            {
                const auto delta = last.p-first.p;
                auto run = glm::dvec3{delta.x,delta.y,0};
                const double length = glm::length(run);
                if (length <= 1e-6) throw std::invalid_argument("Stairs need horizontally separated endpoints.");
                run /= length;
                const glm::dvec3 up{0,0,1}, across = glm::cross(up,run);
                const auto steps = static_cast<unsigned>(std::ceil(std::max(std::abs(delta.z)*scale/item.stepRiseMeters,
                    length*scale/.3)));
                if (steps > 512) throw std::invalid_argument("Stairs exceed the bounded 512-tread presentation budget.");
                for (unsigned i = 0; i < steps; ++i)
                {
                    const double fraction = (i+.5)/steps;
                    auto p = first.p + fraction*delta;
                    box(object.solid,p,run,across,up,{length/steps+.01/scale,width,.06/scale},material,
                        item.deck == DeckStyle::Grating ? 1 : 0);
                }
                for (double side : {-1.0,1.0})
                {
                    const auto offset = across*(side*width*.45)-up*(.15/scale);
                    bar(object.solid,first.p+offset,last.p+offset,up,.10/scale,railMaterial);
                }
                for (unsigned side = 0; side < 2; ++side)
                    if (side == 0 ? item.railings.inner : item.railings.outer)
                    {
                        const auto offset = across*((side == 0 ? -1.0 : 1.0)*width*.5);
                        bar(object.solid,first.p+offset+up*height,last.p+offset+up*height,up,.045/scale,railMaterial);
                        for (const auto& p : {first.p,last.p}) bar(object.solid,p+offset,p+offset+up*height,run,.045/scale,railMaterial);
                    }
            }
            else
            {
                // Maximum 0.75 m chords, tightened where the existing evaluated
                // frame turns/banks by more than five degrees. No new solver.
                std::vector<DeckFrame> frames{first};
                const double length = item.worldPlacement ? glm::length(last.p-first.p)*scale : pathLength;
                const auto at = [&](double distance)
                {
                    if (item.worldPlacement)
                    {
                        auto f = first;
                        const auto delta = last.p-first.p;
                        f.p += delta*(distance/length);
                        f.tangent = glm::normalize(delta);
                        const auto reference = std::abs(f.tangent.z) > .99 ? glm::dvec3{0,1,0} : glm::dvec3{0,0,1};
                        f.lateral = glm::normalize(glm::cross(reference,f.tangent));
                        f.up = glm::normalize(glm::cross(f.tangent,f.lateral));
                        return f;
                    }
                    double station = item.startStationMeters+distance;
                    if (station > track.trackLengthMeters()) station -= track.trackLengthMeters();
                    return frameAt(track,states,station,lateral,item.verticalOffsetMeters);
                };
                const auto atDistance = [&](double distance) { return at(distance); };
                frames.front() = atDistance(0);
                double distance = 0;
                while (distance < length-1e-8)
                {
                    double step = std::min(.75,length-distance);
                    auto next = atDistance(distance+step);
                    for (int refine = 0; refine < 6; ++refine)
                    {
                        if (glm::dot(frames.back().tangent,next.tangent) > .996
                            && glm::dot(frames.back().up,next.up) > .996) break;
                        step *= .5; next = atDistance(distance+step);
                    }
                    distance += step;
                    frames.push_back(next);
                    if (frames.size() > 4096) throw std::invalid_argument("Infrastructure exceeds the bounded 4096-chord budget; split the interval.");
                }
                const auto edge = [&](const DeckFrame& f, unsigned side)
                    { return f.p + f.lateral*((side == 0 ? -1.0 : 1.0)*static_cast<int>(item.side)*width*.5); };
                for (std::size_t i = 1; i < frames.size(); ++i)
                {
                    const auto& a = frames[i-1]; const auto& b = frames[i];
                    const auto x = glm::normalize(b.p-a.p);
                    const auto y = glm::normalize(a.lateral+b.lateral-x*glm::dot(x,a.lateral+b.lateral));
                    const auto z = glm::normalize(glm::cross(x,y));
                    if (item.kind != InfrastructureKind::Guardrail)
                    {
                        box(object.solid,(a.p+b.p)*.5-z*(.035/scale),x,y,z,
                            {glm::length(b.p-a.p)+.012/scale,width,.07/scale},material,item.deck == DeckStyle::Grating ? 1 : 0);
                        for (unsigned side = 0; side < 2; ++side)
                            bar(object.solid,edge(a,side)-a.up*(.10/scale),edge(b,side)-b.up*(.10/scale),a.up,.10/scale,railMaterial);
                    }
                    for (unsigned side = 0; side < 2; ++side)
                        if (side == 0 ? item.railings.inner : item.railings.outer)
                            for (double h : {.5,1.0}) bar(object.solid,edge(a,side)+a.up*(height*h),
                                edge(b,side)+b.up*(height*h),a.up,.045/scale,railMaterial);
                }
                // Post placement is independent of deck tessellation.
                const auto posts = static_cast<unsigned>(std::ceil(length/item.railings.postSpacingMeters));
                if (posts > 1024) throw std::invalid_argument("Guardrails exceed the bounded 1024-post interval budget.");
                for (unsigned i = 0; i <= posts; ++i)
                {
                    const auto f = atDistance(length*i/std::max(posts,1u));
                    for (unsigned side = 0; side < 2; ++side)
                        if (side == 0 ? item.railings.inner : item.railings.outer)
                            bar(object.solid,edge(f,side),edge(f,side)+f.up*height,f.tangent,.055/scale,railMaterial);
                }
            }
            result.push_back(std::move(object));
        }
        return result;
    }
    std::vector<InfrastructureSolid> createAccessPresentation(const coaster::AuthoredTrack& track,
        std::span<const coaster::RiderLocalGeometryState> states, std::span<const physics::HardwareRuntime> runtime)
    {
        using namespace coaster;
        std::vector<InfrastructureSolid> result;
        const double scale = track.physicalSettings().metersPerCoordinateUnit;
        for (const auto& device : track.trackDevices().devices)
        {
            const auto* gate = std::get_if<AccessConfiguration>(&device.configuration);
            if (!gate) continue;
            InfrastructureSolid object{device.id,true}; object.solid.metersPerCoordinateUnit = scale;
            const auto f = frameAt(track,states,device.startStationMeters,gate->lateralOffsetMeters,gate->verticalOffsetMeters);
            double fraction = gate->initiallyOpen ? 1 : 0;
            const auto live = std::ranges::find(runtime,device.id,&physics::HardwareRuntime::id);
            if (live != runtime.end()) if (const auto* state = std::get_if<physics::AccessRuntime>(&live->mechanism)) fraction = state->fraction;
            fraction = std::clamp(fraction,0.0,1.0);
            const auto surface = gate->surface.value_or(defaultSupportSurface(SupportMaterialKind::GalvanizedSteel));
            const auto direction = [&](double angle) { return f.lateral*std::cos(angle)+f.tangent*std::sin(angle); };
            const double yaw = gate->yawDegrees*std::numbers::pi/180;
            const double sign = static_cast<int>(gate->travelDirection);
            const auto closed = direction(yaw);
            const double width = gate->widthMeters/scale,height = gate->heightMeters/scale;
            for (const auto& p : {f.p,f.p+closed*width}) bar(object.solid,p,p+f.up*(height+.1/scale),closed,.08/scale,surface);
            auto pivot = f.p;
            if (gate->motion == AccessMotion::Slide) pivot += closed*(sign*fraction*gate->travelDistanceMeters/scale);
            const auto span = direction(yaw + (gate->motion == AccessMotion::Swing ? sign*fraction*gate->openAngleDegrees*std::numbers::pi/180 : 0));
            const auto far = pivot+span*width;
            bar(object.solid,pivot,far,f.up,.045/scale,surface);
            bar(object.solid,pivot+f.up*height,far+f.up*height,f.up,.045/scale,surface);
            bar(object.solid,pivot,pivot+f.up*height,span,.045/scale,surface);
            bar(object.solid,far,far+f.up*height,span,.045/scale,surface);
            const auto normal = glm::normalize(glm::cross(span,f.up));
            box(object.solid,(pivot+far)*.5+f.up*height*.5,span,f.up,normal,
                {std::max(width-.05/scale,.001/scale),std::max(height-.05/scale,.001/scale),.012/scale},surface,
                gate->infill == AccessInfill::Mesh ? 2 : 0);
            result.push_back(std::move(object));
        }
        return result;
    }

    std::optional<InfrastructureHit> pickInfrastructureSolid(std::span<const InfrastructureSolid> objects,
        const glm::dvec3& origin, const glm::dvec3& direction)
    {
        std::optional<InfrastructureHit> best;
        for (const auto& object : objects) for (const auto& batch : object.solid.batches)
            for (const auto& instance : batch.instances)
            {
                const auto inverse = glm::inverse(glm::dmat4{instance.transform});
                const auto o = glm::dvec3{inverse*glm::dvec4{origin,1}};
                const auto d = glm::dvec3{inverse*glm::dvec4{direction,0}};
                double near = 0, far = std::numeric_limits<double>::infinity();
                for (int axis = 0; axis < 3; ++axis)
                {
                    if (std::abs(d[axis]) < 1e-12)
                    {
                        if (std::abs(o[axis]) > .5) { far = -1; break; }
                    }
                    else
                    {
                        const double a = (-.5-o[axis])/d[axis], b = (.5-o[axis])/d[axis];
                        near = std::max(near,std::min(a,b)); far = std::min(far,std::max(a,b));
                    }
                }
                if (near <= far && (!best || near < best->distance)) best = {object.id,object.access,near};
            }
        return best;
    }
}
