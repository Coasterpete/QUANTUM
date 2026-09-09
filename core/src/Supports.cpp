#include <quantum/coaster/Supports.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace quantum::coaster
{
    namespace
    {
        [[nodiscard]] SupportNode& findSupportNode(
            SupportCollection& collection,
            const SupportStructureId structureId,
            const SupportElementId nodeId)
        {
            const auto structure = std::find_if(
                collection.structures.begin(),
                collection.structures.end(),
                [structureId](const SupportStructure& value)
                {
                    return value.id == structureId;
                });
            if (structure == collection.structures.end())
            {
                throw std::invalid_argument("Unknown support structure ID.");
            }

            const auto node = std::find_if(
                structure->nodes.begin(),
                structure->nodes.end(),
                [nodeId](const SupportNode& value)
                {
                    return value.id == nodeId;
                });
            if (node == structure->nodes.end())
            {
                throw std::invalid_argument("Unknown support node ID.");
            }
            return *node;
        }

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

    SupportMemberProfile defaultSupportMemberProfile() noexcept
    {
        return {};
    }

    SupportStructureId createSupportStructure(
        SupportCollection& collection,
        std::string name)
    {
        const SupportStructureId id = allocateSupportStructureId(collection);
        if (name.empty())
        {
            name = "Support " + std::to_string(id);
        }

        collection.structures.push_back({id, std::move(name)});
        validateSupportCollection(collection);
        return id;
    }

    void removeSupportStructure(
        SupportCollection& collection,
        const SupportStructureId structureId)
    {
        const auto structure = std::find_if(
            collection.structures.begin(),
            collection.structures.end(),
            [structureId](const SupportStructure& value)
            {
                return value.id == structureId;
            });
        if (structure == collection.structures.end())
        {
            throw std::invalid_argument("Unknown support structure ID.");
        }

        collection.structures.erase(structure);
        validateSupportCollection(collection);
    }

    SupportElementId createSupportNode(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const glm::dvec3& position)
    {
        if (!std::isfinite(position.x) || !std::isfinite(position.y)
            || !std::isfinite(position.z))
        {
            throw std::invalid_argument(
                "Support node positions must be finite.");
        }

        auto structure = std::find_if(
            collection.structures.begin(),
            collection.structures.end(),
            [structureId](const SupportStructure& value)
            {
                return value.id == structureId;
            });
        if (structure == collection.structures.end())
        {
            throw std::invalid_argument("Unknown support structure ID.");
        }

        const SupportElementId id = allocateSupportElementId(*structure);
        structure->nodes.push_back({id, position});
        validateSupportCollection(collection);
        return id;
    }

    void removeSupportNode(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId nodeId)
    {
        auto structure = std::find_if(
            collection.structures.begin(),
            collection.structures.end(),
            [structureId](const SupportStructure& value)
            {
                return value.id == structureId;
            });
        if (structure == collection.structures.end())
        {
            throw std::invalid_argument("Unknown support structure ID.");
        }

        const auto node = std::find_if(
            structure->nodes.begin(),
            structure->nodes.end(),
            [nodeId](const SupportNode& value)
            {
                return value.id == nodeId;
            });
        if (node == structure->nodes.end())
        {
            throw std::invalid_argument("Unknown support node ID.");
        }

        const bool referenced = std::any_of(
            structure->members.begin(),
            structure->members.end(),
            [nodeId](const SupportMember& member)
            {
                return member.startNodeId == nodeId
                    || member.endNodeId == nodeId;
            });
        if (referenced)
        {
            throw std::invalid_argument(
                "Delete connected members first.");
        }

        structure->nodes.erase(node);
        validateSupportCollection(collection);
    }

    SupportElementId createSupportMember(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId startNodeId,
        const SupportElementId endNodeId,
        const SupportMemberProfile& profile)
    {
        auto structure = std::find_if(
            collection.structures.begin(),
            collection.structures.end(),
            [structureId](const SupportStructure& value)
            {
                return value.id == structureId;
            });
        if (structure == collection.structures.end())
        {
            throw std::invalid_argument("Unknown support structure ID.");
        }

        const bool hasStart = std::any_of(
            structure->nodes.begin(),
            structure->nodes.end(),
            [startNodeId](const SupportNode& node)
            {
                return node.id == startNodeId;
            });
        const bool hasEnd = std::any_of(
            structure->nodes.begin(),
            structure->nodes.end(),
            [endNodeId](const SupportNode& node)
            {
                return node.id == endNodeId;
            });
        if (!hasStart || !hasEnd)
        {
            throw std::invalid_argument(
                "Support member endpoints must reference nodes in their "
                "owning structure.");
        }
        if (startNodeId == endNodeId)
        {
            throw std::invalid_argument(
                "A support member must connect two distinct nodes.");
        }

        const std::pair<SupportElementId, SupportElementId> pair{
            std::min(startNodeId, endNodeId),
            std::max(startNodeId, endNodeId)};
        const bool alreadyConnected = std::any_of(
            structure->members.begin(),
            structure->members.end(),
            [pair](const SupportMember& member)
            {
                return std::min(member.startNodeId, member.endNodeId)
                        == pair.first
                    && std::max(member.startNodeId, member.endNodeId)
                        == pair.second;
            });
        if (alreadyConnected)
        {
            throw std::invalid_argument(
                "A member already connects this pair of nodes.");
        }

        const SupportElementId id = allocateSupportElementId(*structure);
        structure->members.push_back({
            id, startNodeId, endNodeId, profile});
        validateSupportCollection(collection);
        return id;
    }

    void removeSupportMember(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId memberId)
    {
        auto structure = std::find_if(
            collection.structures.begin(),
            collection.structures.end(),
            [structureId](const SupportStructure& value)
            {
                return value.id == structureId;
            });
        if (structure == collection.structures.end())
        {
            throw std::invalid_argument("Unknown support structure ID.");
        }

        const auto member = std::find_if(
            structure->members.begin(),
            structure->members.end(),
            [memberId](const SupportMember& value)
            {
                return value.id == memberId;
            });
        if (member == structure->members.end())
        {
            throw std::invalid_argument("Unknown support member ID.");
        }

        structure->members.erase(member);
        validateSupportCollection(collection);
    }

    void setSupportTrackAttachment(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId nodeId,
        const TrackAttachment& attachment)
    {
        validateSupportCollection(collection);
        if (!std::isfinite(attachment.station)
            || !std::isfinite(attachment.lateralOffset)
            || !std::isfinite(attachment.verticalOffset))
        {
            throw std::invalid_argument(
                "Support track attachment values must be finite.");
        }
        SupportNode& node = findSupportNode(collection, structureId, nodeId);
        if (node.foundation.has_value())
        {
            throw std::invalid_argument(
                "A support node cannot be both track-attached and a foundation.");
        }
        node.trackAttachment = attachment;
        validateSupportCollection(collection);
    }

    void clearSupportTrackAttachment(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId nodeId)
    {
        validateSupportCollection(collection);
        findSupportNode(collection, structureId, nodeId)
            .trackAttachment.reset();
    }

    void setSupportFoundation(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId nodeId)
    {
        validateSupportCollection(collection);
        SupportNode& node = findSupportNode(collection, structureId, nodeId);
        if (node.trackAttachment.has_value())
        {
            throw std::invalid_argument(
                "A support node cannot be both track-attached and a foundation.");
        }
        node.foundation.emplace();
        validateSupportCollection(collection);
    }

    void clearSupportFoundation(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId nodeId)
    {
        validateSupportCollection(collection);
        findSupportNode(collection, structureId, nodeId).foundation.reset();
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
                if (node.trackAttachment.has_value())
                {
                    const TrackAttachment& attachment = *node.trackAttachment;
                    if (!std::isfinite(attachment.station)
                        || !std::isfinite(attachment.lateralOffset)
                        || !std::isfinite(attachment.verticalOffset))
                    {
                        throw std::invalid_argument(
                            "Support track attachment values must be finite.");
                    }
                }
                if (node.trackAttachment.has_value()
                    && node.foundation.has_value())
                {
                    throw std::invalid_argument(
                        "A support node cannot be both track-attached and a foundation.");
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
