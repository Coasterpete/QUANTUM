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

        [[nodiscard]] bool finite(const glm::dvec3& value) noexcept
        {
            return std::isfinite(value.x)
                && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        [[nodiscard]] bool finite(const glm::dquat& value) noexcept
        {
            return std::isfinite(value.w)
                && std::isfinite(value.x)
                && std::isfinite(value.y)
                && std::isfinite(value.z);
        }

        // True when the quaternion is not the deterministic signed form used
        // for serialization. Mirrors the AuthoredStartPose convention: the
        // first nonzero component (w, x, y, z order) must be positive.
        [[nodiscard]] bool negativeCanonicalSign(
            const glm::dquat& value) noexcept
        {
            return value.w < 0.0
                || (value.w == 0.0 && value.x < 0.0)
                || (value.w == 0.0 && value.x == 0.0
                    && value.y < 0.0)
                || (value.w == 0.0 && value.x == 0.0
                    && value.y == 0.0 && value.z < 0.0);
        }

        [[nodiscard]] glm::dquat canonicalPlacementOrientation(
            const glm::dquat& orientation)
        {
            if (!finite(orientation))
            {
                throw std::invalid_argument(
                    "A member-end connection orientation must be finite.");
            }

            const double scale = std::max({
                std::abs(orientation.w),
                std::abs(orientation.x),
                std::abs(orientation.y),
                std::abs(orientation.z)
            });
            if (scale == 0.0)
            {
                throw std::invalid_argument(
                    "A member-end connection orientation must be nonzero.");
            }

            glm::dquat normalized = orientation / scale;
            const double magnitude = std::hypot(
                std::hypot(normalized.w, normalized.x),
                std::hypot(normalized.y, normalized.z)
            );
            if (!std::isfinite(magnitude) || magnitude == 0.0)
            {
                throw std::invalid_argument(
                    "A member-end connection orientation could not be normalized.");
            }
            normalized /= magnitude;

            // q and -q encode the same rotation. Keeping one canonical sign
            // makes document serialization deterministic across edit paths.
            if (negativeCanonicalSign(normalized))
            {
                normalized = -normalized;
            }
            return normalized;
        }

        [[nodiscard]] SupportMemberEndPlacement canonicalPlacement(
            const SupportMemberEndPlacement& placement)
        {
            SupportMemberEndPlacement canonical = placement;
            if (!finite(canonical.position))
            {
                throw std::invalid_argument(
                    "A member-end connection position must be finite.");
            }
            canonical.orientation = canonicalPlacementOrientation(
                placement.orientation);
            if (!finite(canonical.scale)
                || canonical.scale.x <= 0.0
                || canonical.scale.y <= 0.0
                || canonical.scale.z <= 0.0)
            {
                throw std::invalid_argument(
                    "A member-end connection scale must be finite and positive.");
            }
            return canonical;
        }

        [[nodiscard]] SupportMember& findSupportMember(
            SupportStructure& structure,
            const SupportElementId memberId)
        {
            const auto member = std::find_if(
                structure.members.begin(),
                structure.members.end(),
                [memberId](const SupportMember& value)
                {
                    return value.id == memberId;
                });
            if (member == structure.members.end())
            {
                throw std::invalid_argument("Unknown support member ID.");
            }
            return *member;
        }

        // Resolves the node that owns the interface context for one member
        // end. Connections are member metadata and live independently of the
        // node, but the node's anchor metadata decides whether a track-only
        // or foundation-only treatment is meaningful there.
        [[nodiscard]] const SupportNode& findMemberEndNode(
            const SupportStructure& structure,
            const SupportElementId endNodeId)
        {
            const auto node = std::find_if(
                structure.nodes.begin(),
                structure.nodes.end(),
                [endNodeId](const SupportNode& value)
                {
                    return value.id == endNodeId;
                });
            if (node == structure.nodes.end())
            {
                throw std::invalid_argument("Unknown support node ID.");
            }
            return *node;
        }

        void validateMemberEndNodeContext(
            const SupportNode& node,
            const SupportMemberEndTreatment treatment)
        {
            const bool requiresTrack = treatment
                    == SupportMemberEndTreatment::Saddle
                || treatment == SupportMemberEndTreatment::Clamp;
            const bool requiresFoundation = treatment
                    == SupportMemberEndTreatment::Base
                || treatment == SupportMemberEndTreatment::Footing;
            if (requiresTrack && !node.trackAttachment.has_value())
            {
                throw std::invalid_argument(
                    "A Saddle or Clamp member-end connection requires a node "
                    "with a track attachment.");
            }
            if (requiresFoundation && !node.foundation.has_value())
            {
                throw std::invalid_argument(
                    "A Base or Footing member-end connection requires a node "
                    "with a foundation.");
            }
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

    std::string normalizeSupportConnectorAssetIdentifier(
        const std::string_view identifier)
    {
        return normalizeStaticMeshAssetIdentifier(identifier, "support");
    }

    SupportMemberEndConnection normalizeSupportMemberEndConnection(
        const SupportMemberEndConnection& connection)
    {
        SupportMemberEndConnection normalized = connection;

        switch (connection.treatment)
        {
        case SupportMemberEndTreatment::MiteredCut:
        case SupportMemberEndTreatment::EndCap:
        case SupportMemberEndTreatment::Plate:
        case SupportMemberEndTreatment::Flange:
        case SupportMemberEndTreatment::Splice:
        case SupportMemberEndTreatment::Saddle:
        case SupportMemberEndTreatment::Clamp:
        case SupportMemberEndTreatment::Base:
        case SupportMemberEndTreatment::Footing:
            break;
        default:
            throw std::invalid_argument(
                "Support member-end treatment is not supported.");
        }

        if (normalized.asset.has_value())
        {
            if (normalized.asset->path.empty())
            {
                throw std::invalid_argument(
                    "A member-end connection asset requires an asset reference.");
            }
            normalized.asset->path = normalizeSupportConnectorAssetIdentifier(
                normalized.asset->path);
        }
        if (normalized.localPlacement.has_value())
        {
            normalized.localPlacement = canonicalPlacement(
                *normalized.localPlacement);
        }
        return normalized;
    }

    void validateSupportMemberEndConnection(
        const SupportMemberEndConnection& connection)
    {
        switch (connection.treatment)
        {
        case SupportMemberEndTreatment::MiteredCut:
        case SupportMemberEndTreatment::EndCap:
        case SupportMemberEndTreatment::Plate:
        case SupportMemberEndTreatment::Flange:
        case SupportMemberEndTreatment::Splice:
        case SupportMemberEndTreatment::Saddle:
        case SupportMemberEndTreatment::Clamp:
        case SupportMemberEndTreatment::Base:
        case SupportMemberEndTreatment::Footing:
            break;
        default:
            throw std::invalid_argument(
                "Support member-end treatment is not supported.");
        }

        if (connection.asset.has_value())
        {
            const StaticMeshAssetReference& asset = *connection.asset;
            if (asset.path.empty())
            {
                throw std::invalid_argument(
                    "A member-end connection asset requires an asset reference.");
            }
            static_cast<void>(
                normalizeSupportConnectorAssetIdentifier(asset.path));
        }
        if (connection.localPlacement.has_value())
        {
            const SupportMemberEndPlacement& placement =
                *connection.localPlacement;
            if (!finite(placement.position))
            {
                throw std::invalid_argument(
                    "A member-end connection position must be finite.");
            }
            const glm::dquat& orientation = placement.orientation;
            if (!finite(orientation))
            {
                throw std::invalid_argument(
                    "A member-end connection orientation must be finite.");
            }
            const double magnitude = std::hypot(
                std::hypot(orientation.w, orientation.x),
                std::hypot(orientation.y, orientation.z)
            );
            if (!std::isfinite(magnitude)
                || std::abs(magnitude - 1.0) > 1.0e-9)
            {
                throw std::invalid_argument(
                    "A member-end connection orientation must be normalized.");
            }
            if (negativeCanonicalSign(orientation))
            {
                throw std::invalid_argument(
                    "A stored member-end connection orientation must use "
                    "the canonical sign.");
            }
            if (!finite(placement.scale)
                || placement.scale.x <= 0.0
                || placement.scale.y <= 0.0
                || placement.scale.z <= 0.0)
            {
                throw std::invalid_argument(
                    "A member-end connection scale must be finite and positive.");
            }
        }
    }

    void setSupportMemberEndConnection(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId memberId,
        const SupportMemberEnd end,
        const SupportMemberEndConnection& connection)
    {
        validateSupportCollection(collection);
        const SupportMemberEndConnection normalized =
            normalizeSupportMemberEndConnection(connection);

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

        SupportMember& member = findSupportMember(*structure, memberId);
        const SupportElementId endNodeId = end == SupportMemberEnd::Start
            ? member.startNodeId : member.endNodeId;
        validateMemberEndNodeContext(
            findMemberEndNode(*structure, endNodeId),
            normalized.treatment);

        if (end == SupportMemberEnd::Start)
        {
            member.startConnection = normalized;
        }
        else
        {
            member.endConnection = normalized;
        }
        validateSupportCollection(collection);
    }

    void clearSupportMemberEndConnection(
        SupportCollection& collection,
        const SupportStructureId structureId,
        const SupportElementId memberId,
        const SupportMemberEnd end)
    {
        validateSupportCollection(collection);

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

        SupportMember& member = findSupportMember(*structure, memberId);
        if (end == SupportMemberEnd::Start)
        {
            member.startConnection.reset();
        }
        else
        {
            member.endConnection.reset();
        }
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

                const auto validateMemberEndConnection =
                    [&structure](
                        const SupportMember& memberValue,
                        const SupportMemberEnd end,
                        const SupportMemberEndConnection& connection)
                {
                    validateSupportMemberEndConnection(connection);
                    const SupportElementId endNodeId = end
                        == SupportMemberEnd::Start
                        ? memberValue.startNodeId : memberValue.endNodeId;
                    const auto node = std::find_if(
                        structure.nodes.begin(),
                        structure.nodes.end(),
                        [endNodeId](const SupportNode& value)
                        {
                            return value.id == endNodeId;
                        });
                    validateMemberEndNodeContext(*node, connection.treatment);
                };

                if (member.startConnection.has_value())
                {
                    validateMemberEndConnection(
                        member,
                        SupportMemberEnd::Start,
                        *member.startConnection);
                }
                if (member.endConnection.has_value())
                {
                    validateMemberEndConnection(
                        member,
                        SupportMemberEnd::End,
                        *member.endConnection);
                }
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
