#include <quantum/coaster/SupportSolidGeometry.hpp>

#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace quantum::coaster
{
    namespace
    {
        [[nodiscard]] bool isFinite(const glm::dvec3& value) noexcept
        {
            return std::isfinite(value.x) && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        [[nodiscard]] glm::vec3 toFloat(const glm::dvec3& value) noexcept
        {
            return {static_cast<float>(value.x), static_cast<float>(value.y),
                static_cast<float>(value.z)};
        }

        // Resolved node positions keyed by SupportElementId. Ids are sparse
        // and structure-local, so a lookup map keeps the member loop linear
        // in member count instead of quadratic.
        using NodeLookup = std::unordered_map<SupportElementId, glm::dvec3>;
        // Nodes carrying a Foundation anchor, used to size that node's pad
        // from the members it actually carries.
        using FoundationLookup =
            std::unordered_map<SupportElementId, double>;

        [[nodiscard]] NodeLookup buildNodeLookup(
            const SupportStructure& structure)
        {
            NodeLookup lookup;
            lookup.reserve(structure.nodes.size() * 2);
            for (const SupportNode& node : structure.nodes)
            {
                if (!isFinite(node.position))
                {
                    throw std::invalid_argument(
                        "Support node positions must be finite.");
                }
                lookup.emplace(node.id, node.position);
            }
            return lookup;
        }

        [[nodiscard]] const glm::dvec3& requireNode(
            const NodeLookup& lookup,
            const SupportElementId nodeId)
        {
            const auto found = lookup.find(nodeId);
            if (found == lookup.end())
            {
                throw std::logic_error(
                    "Validated support member endpoints were not found.");
            }
            return found->second;
        }

        // A rectangular cross-section takes outerDimensions.x as its width and
        // .y as its depth. A circular one is validated to have equal
        // dimensions, so the first component is the diameter.
        [[nodiscard]] glm::dvec2 crossSection(
            const SupportMemberProfile& profile) noexcept
        {
            if (profile.shape == SupportMemberProfileShape::Circular)
            {
                return {profile.outerDimensions.x, profile.outerDimensions.x};
            }
            return profile.outerDimensions;
        }

        // The unit meshes are unit-cube and unit-diameter, so the basis
        // columns carry the member's length, width, and depth directly.
        [[nodiscard]] glm::mat4 memberTransform(
            const SupportMemberFrame& frame,
            const glm::dvec2& section) noexcept
        {
            glm::mat4 transform{1.0F};
            transform[0] = glm::vec4(
                toFloat(frame.axisX) * static_cast<float>(frame.length),
                0.0F);
            transform[1] = glm::vec4(
                toFloat(frame.axisY) * static_cast<float>(section.x), 0.0F);
            transform[2] = glm::vec4(
                toFloat(frame.axisZ) * static_cast<float>(section.y), 0.0F);
            transform[3] = glm::vec4(toFloat(frame.origin), 1.0F);
            return transform;
        }
    }

    std::size_t SupportSolidPresentation::memberCount() const noexcept
    {
        std::size_t total = 0;
        for (const SupportSolidBatch& batch : batches)
        {
            total += batch.instances.size();
        }
        return total;
    }

    std::size_t SupportSolidPresentation::drawCallCount() const noexcept
    {
        std::size_t calls = 0;
        for (const SupportSolidBatch& batch : batches)
        {
            if (!batch.instances.empty())
            {
                ++calls;
            }
        }
        return calls;
    }

    bool SupportSolidPresentation::empty() const noexcept
    {
        return memberCount() == 0 && foundations.empty();
    }

    SupportMemberFrame resolveSupportMemberFrame(
        const glm::dvec3& start,
        const glm::dvec3& end)
    {
        if (!isFinite(start) || !isFinite(end))
        {
            throw std::invalid_argument(
                "Support member endpoints must be finite.");
        }

        SupportMemberFrame frame;
        const glm::dvec3 delta = end - start;
        frame.length = glm::length(delta);
        frame.origin = (start + end) * 0.5;
        if (frame.length <= minimumSupportMemberLength)
        {
            return frame;
        }

        frame.axisX = delta / frame.length;

        // The cross-section reference is world +Z projected off the member
        // axis. For any member that is not essentially vertical this is well
        // conditioned, continuous in the axis, and gives neighbouring members
        // the same orientation -- which is what stops a bent's posts from
        // randomly rolling against each other.
        if (std::abs(frame.axisX.z) > supportMemberVerticalTolerance)
        {
            frame.axisY = glm::dvec3(1.0, 0.0, 0.0);
            frame.usedVerticalFallback = true;
        }
        else
        {
            frame.axisY = glm::dvec3(0.0, 0.0, 1.0);
        }

        // Gram-Schmidt against the exact longitudinal axis. axisY is
        // recomputed rather than reused so a fallback that happens to be
        // parallel still yields a valid basis.
        frame.axisY = frame.axisY
            - frame.axisX * glm::dot(frame.axisX, frame.axisY);
        const double perpendicularLength = glm::length(frame.axisY);
        if (perpendicularLength <= minimumSupportMemberLength)
        {
            throw std::invalid_argument(
                "Support member orientation is degenerate.");
        }
        frame.axisY /= perpendicularLength;
        // Right-handed completion, so the local basis always maps +Y x +Z to
        // +X and triangle winding is preserved for every member.
        frame.axisZ = glm::cross(frame.axisX, frame.axisY);
        return frame;
    }

    SupportSolidPresentation buildSupportSolidPresentation(
        const SupportStructure& structure)
    {
        SupportSolidPresentation presentation;
        presentation.structureId = structure.id;
        presentation.appearance =
            structure.appearance.value_or(SupportAppearance{});
        presentation.foundationAppearance =
            structure.foundationAppearance.value_or(
                SupportFoundationAppearance{});
        validateSupportAppearance(presentation.appearance);
        validateSupportFoundationAppearance(
            presentation.foundationAppearance);

        const NodeLookup nodes = buildNodeLookup(structure);

        std::vector<SupportMemberInstance> rectangular;
        std::vector<SupportMemberInstance> circular;
        rectangular.reserve(structure.members.size());
        circular.reserve(structure.members.size());

        // The largest cross-section dimension meeting each foundation node.
        // Indexing these IDs avoids searching every node for every member.
        // This is a presentation proportion, not structural footing design.
        FoundationLookup foundationSections;
        foundationSections.reserve(structure.nodes.size());
        for (const SupportNode& node : structure.nodes)
        {
            if (node.foundation)
            {
                foundationSections.emplace(node.id, 0.0);
            }
        }

        for (const SupportMember& member : structure.members)
        {
            validateSupportMemberProfile(member.profile);
            const SupportMemberFrame frame = resolveSupportMemberFrame(
                requireNode(nodes, member.startNodeId),
                requireNode(nodes, member.endNodeId));
            if (frame.length <= minimumSupportMemberLength)
            {
                // The generator already rejects these, so reaching one here
                // means externally written or hand-edited data. Skipping keeps
                // a sub-pixel or NaN basis out of the instance buffer without
                // failing the whole presentation.
                continue;
            }

            const glm::dvec2 section = crossSection(member.profile);
            SupportMemberInstance instance;
            instance.transform = memberTransform(frame, section);

            const bool circularSection =
                member.profile.shape == SupportMemberProfileShape::Circular;
            (circularSection ? circular : rectangular).push_back(instance);

            const double largest = std::max(section.x, section.y);
            for (const SupportElementId nodeId :
                {member.startNodeId, member.endNodeId})
            {
                const auto found = foundationSections.find(nodeId);
                if (found != foundationSections.end())
                {
                    found->second = std::max(found->second, largest);
                }
            }
        }

        if (!rectangular.empty())
        {
            presentation.batches.push_back(
                {SupportMemberMeshKind::Rectangular, std::move(rectangular)});
        }
        if (!circular.empty())
        {
            presentation.batches.push_back(
                {SupportMemberMeshKind::Circular, std::move(circular)});
        }

        for (const SupportNode& node : structure.nodes)
        {
            if (!node.foundation.has_value())
            {
                continue;
            }
            SupportFoundationPad pad;
            pad.position = node.position;
            const auto found = foundationSections.find(node.id);
            const double carried =
                found == foundationSections.end() ? 0.0 : found->second;
            const double footprint = carried > 0.0
                ? carried * supportFoundationPadFootprintScale
                : supportFoundationPadFootprintScale;
            pad.padDimensions = {
                presentation.foundationAppearance.padDimensions.x > 0.0
                    ? presentation.foundationAppearance.padDimensions.x
                    : footprint,
                presentation.foundationAppearance.padDimensions.y > 0.0
                    ? presentation.foundationAppearance.padDimensions.y
                    : footprint};
            // Thickness comes from the post's own cross-section, not from the
            // pad footprint. Scaling the footprint instead would make a footing
            // as thick as it is wide, which reads as a square pillar stub.
            const double section = carried;
            pad.padDepth =
                presentation.foundationAppearance.padDepth > 0.0
                ? presentation.foundationAppearance.padDepth
                : (section > 0.0 ? section : 1.0)
                    * supportFoundationPadDepthScale;
            presentation.foundations.push_back(pad);
        }

        return presentation;
    }

    std::vector<SupportSolidPresentation> buildSupportSolidPresentation(
        const SupportCollection& collection)
    {
        std::vector<SupportSolidPresentation> presentations;
        presentations.reserve(collection.structures.size());
        for (const SupportStructure& structure : collection.structures)
        {
            presentations.push_back(buildSupportSolidPresentation(structure));
        }
        return presentations;
    }
}
