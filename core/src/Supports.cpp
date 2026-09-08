#include <quantum/coaster/Supports.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace quantum::coaster
{
    namespace
    {
        template<typename Id>
        void validateAllocationCounter(
            const Id nextId,
            const Id maximumAllocatedId,
            const char* const counterName)
        {
            if (nextId == 0 || nextId <= maximumAllocatedId)
            {
                throw std::invalid_argument(
                    std::string(counterName)
                    + " must be nonzero and greater than every allocated ID.");
            }
        }

        template<typename Id>
        [[nodiscard]] Id allocateId(
            Id& nextId,
            const Id maximumAllocatedId,
            const char* const counterName)
        {
            validateAllocationCounter(nextId, maximumAllocatedId, counterName);
            if (nextId == std::numeric_limits<Id>::max())
            {
                throw std::overflow_error(
                    std::string(counterName) + " has exhausted its ID space.");
            }

            return nextId++;
        }
    }

    SupportStructureId allocateSupportStructureId(
        SupportCollection& collection)
    {
        SupportStructureId maximumId = invalidSupportStructureId;
        for (const SupportStructure& structure : collection.structures)
        {
            maximumId = std::max(maximumId, structure.id);
        }

        return allocateId(
            collection.nextStructureId,
            maximumId,
            "SupportCollection::nextStructureId");
    }

    SupportElementId allocateSupportElementId(SupportStructure& structure)
    {
        SupportElementId maximumId = invalidSupportElementId;
        for (const SupportNode& node : structure.nodes)
        {
            maximumId = std::max(maximumId, node.id);
        }
        for (const SupportMember& member : structure.members)
        {
            maximumId = std::max(maximumId, member.id);
        }

        return allocateId(
            structure.nextElementId,
            maximumId,
            "SupportStructure::nextElementId");
    }

    void validateSupportMemberProfile(const SupportMemberProfile& profile)
    {
        switch (profile.shape)
        {
        case SupportMemberProfileShape::Circular:
        case SupportMemberProfileShape::Rectangular:
            break;
        default:
            throw std::invalid_argument(
                "Support member profile shape is not supported.");
        }

        const double width = profile.outerDimensions.x;
        const double height = profile.outerDimensions.y;
        if (!std::isfinite(width) || !std::isfinite(height)
            || width <= 0.0 || height <= 0.0)
        {
            throw std::invalid_argument(
                "Support member outer dimensions must be finite and positive.");
        }
        if (profile.shape == SupportMemberProfileShape::Circular
            && width != height)
        {
            throw std::invalid_argument(
                "A circular support member must have equal outer dimensions.");
        }
        if (!std::isfinite(profile.wallThickness)
            || profile.wallThickness < 0.0
            || profile.wallThickness * 2.0 >= std::min(width, height))
        {
            throw std::invalid_argument(
                "Support member wall thickness must be finite, nonnegative, "
                "and less than half the smallest outer dimension.");
        }
    }

    void validateSupportCollection(const SupportCollection& collection)
    {
        std::unordered_set<SupportStructureId> structureIds;
        SupportStructureId maximumStructureId = invalidSupportStructureId;

        for (const SupportStructure& structure : collection.structures)
        {
            if (structure.id == invalidSupportStructureId)
            {
                throw std::invalid_argument(
                    "Support structure IDs must be nonzero.");
            }
            if (!structureIds.insert(structure.id).second)
            {
                throw std::invalid_argument(
                    "Support structure IDs must be unique within a document.");
            }
            maximumStructureId = std::max(maximumStructureId, structure.id);

            std::unordered_set<SupportElementId> elementIds;
            std::unordered_set<SupportElementId> nodeIds;
            SupportElementId maximumElementId = invalidSupportElementId;

            for (const SupportNode& node : structure.nodes)
            {
                if (node.id == invalidSupportElementId)
                {
                    throw std::invalid_argument(
                        "Support node IDs must be nonzero.");
                }
                if (!elementIds.insert(node.id).second)
                {
                    throw std::invalid_argument(
                        "Support element IDs must be unique within a structure.");
                }
                nodeIds.insert(node.id);
                maximumElementId = std::max(maximumElementId, node.id);

                if (!std::isfinite(node.position.x)
                    || !std::isfinite(node.position.y)
                    || !std::isfinite(node.position.z))
                {
                    throw std::invalid_argument(
                        "Support node positions must be finite.");
                }
            }

            for (const SupportMember& member : structure.members)
            {
                if (member.id == invalidSupportElementId)
                {
                    throw std::invalid_argument(
                        "Support member IDs must be nonzero.");
                }
                if (!elementIds.insert(member.id).second)
                {
                    throw std::invalid_argument(
                        "Support element IDs must be unique within a structure.");
                }
                maximumElementId = std::max(maximumElementId, member.id);

                if (!nodeIds.contains(member.startNodeId)
                    || !nodeIds.contains(member.endNodeId))
                {
                    throw std::invalid_argument(
                        "Support member endpoints must reference nodes in their "
                        "owning structure.");
                }
                if (member.startNodeId == member.endNodeId)
                {
                    throw std::invalid_argument(
                        "A support member must connect two distinct nodes.");
                }

                validateSupportMemberProfile(member.profile);
            }

            validateAllocationCounter(
                structure.nextElementId,
                maximumElementId,
                "SupportStructure::nextElementId");
        }

        validateAllocationCounter(
            collection.nextStructureId,
            maximumStructureId,
            "SupportCollection::nextStructureId");
    }
}
