#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/Supports.hpp>

#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
    using namespace quantum::coaster;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    template<typename Function>
    void requireInvalid(Function&& function, const std::string_view message)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const std::invalid_argument&)
        {
            return;
        }
        throw std::runtime_error(std::string(message));
    }

    [[nodiscard]] SupportMemberProfile roundTube()
    {
        return {SupportMemberProfileShape::Circular, {0.3, 0.3}, 0.025};
    }

    [[nodiscard]] SupportMemberProfile timber()
    {
        return {SupportMemberProfileShape::Rectangular, {0.2, 0.3}, 0.0};
    }

    SupportStructure& addStructure(
        SupportCollection& collection,
        std::string name)
    {
        const SupportStructureId id = allocateSupportStructureId(collection);
        collection.structures.push_back({id, std::move(name)});
        return collection.structures.back();
    }

    SupportElementId addNode(
        SupportStructure& structure,
        const glm::dvec3 position)
    {
        const SupportElementId id = allocateSupportElementId(structure);
        structure.nodes.push_back({id, position});
        return id;
    }

    void addMember(
        SupportStructure& structure,
        const SupportElementId start,
        const SupportElementId end,
        const SupportMemberProfile& profile)
    {
        structure.members.push_back({
            allocateSupportElementId(structure), start, end, profile});
    }

    void emptyCollectionIsValid()
    {
        const SupportCollection collection;
        validateSupportCollection(collection);
        require(collection.empty(), "default support collection must be empty");

        const AuthoredTrack track;
        validateSupportCollection(track.supports());
        require(track.supports().empty(),
            "default AuthoredTrack must own an empty valid support collection");
    }

    void allocationIsMonotonicAndDoesNotReuseIds()
    {
        SupportCollection collection;
        const auto firstStructure = allocateSupportStructureId(collection);
        collection.structures.push_back({firstStructure, "First"});
        const auto secondStructure = allocateSupportStructureId(collection);
        collection.structures.push_back({secondStructure, "Second"});
        collection.structures.erase(collection.structures.begin());
        const auto thirdStructure = allocateSupportStructureId(collection);
        require(firstStructure == 1 && secondStructure == 2
                && thirdStructure == 3,
            "structure IDs must increase without gap reuse");

        SupportStructure& structure = collection.structures.front();
        const auto firstElement = addNode(structure, {0.0, 0.0, 0.0});
        const auto secondElement = addNode(structure, {0.0, 0.0, 1.0});
        structure.nodes.erase(structure.nodes.begin());
        const auto thirdElement = addNode(structure, {0.0, 0.0, 2.0});
        require(firstElement == 1 && secondElement == 2 && thirdElement == 3,
            "element IDs must increase without gap reuse");
    }

    void invalidIdsAndReferencesAreRejected()
    {
        SupportCollection collection;
        SupportStructure& structure = addStructure(collection, "Frame");
        const auto first = addNode(structure, {0.0, 0.0, 0.0});
        const auto second = addNode(structure, {0.0, 0.0, 1.0});
        addMember(structure, first, second, roundTube());
        validateSupportCollection(collection);

        auto zeroStructure = collection;
        zeroStructure.structures.front().id = 0;
        requireInvalid([&] { validateSupportCollection(zeroStructure); },
            "zero structure ID must be rejected");

        auto duplicateStructure = collection;
        duplicateStructure.structures.push_back(
            duplicateStructure.structures.front());
        requireInvalid([&] { validateSupportCollection(duplicateStructure); },
            "duplicate structure ID must be rejected");

        auto zeroElement = collection;
        zeroElement.structures.front().nodes.front().id = 0;
        requireInvalid([&] { validateSupportCollection(zeroElement); },
            "zero element ID must be rejected");

        auto duplicateElement = collection;
        duplicateElement.structures.front().members.front().id = first;
        requireInvalid([&] { validateSupportCollection(duplicateElement); },
            "node/member IDs share one unique element namespace");

        auto dangling = collection;
        dangling.structures.front().members.front().endNodeId = 999;
        requireInvalid([&] { validateSupportCollection(dangling); },
            "dangling member endpoint must be rejected");

        auto selfMember = collection;
        selfMember.structures.front().members.front().endNodeId = first;
        requireInvalid([&] { validateSupportCollection(selfMember); },
            "self-connected member must be rejected");
    }

    void invalidGeometryAndCountersAreRejected()
    {
        SupportCollection collection;
        SupportStructure& structure = addStructure(collection, "Column");
        const auto bottom = addNode(structure, {0.0, 0.0, 0.0});
        const auto top = addNode(structure, {0.0, 0.0, 5.0});
        addMember(structure, bottom, top, roundTube());

        auto nonfinite = collection;
        nonfinite.structures.front().nodes.front().position.x =
            std::numeric_limits<double>::infinity();
        requireInvalid([&] { validateSupportCollection(nonfinite); },
            "non-finite node position must be rejected");

        auto invalidDimensions = collection;
        invalidDimensions.structures.front().members.front()
            .profile.outerDimensions.x = 0.0;
        requireInvalid([&] { validateSupportCollection(invalidDimensions); },
            "non-positive profile dimension must be rejected");

        auto invalidWall = collection;
        invalidWall.structures.front().members.front().profile.wallThickness =
            0.15;
        requireInvalid([&] { validateSupportCollection(invalidWall); },
            "wall consuming the profile must be rejected");

        auto invalidShape = collection;
        invalidShape.structures.front().members.front().profile.shape =
            static_cast<SupportMemberProfileShape>(255);
        requireInvalid([&] { validateSupportCollection(invalidShape); },
            "unsupported profile shape must be rejected");

        auto invalidElementCounter = collection;
        invalidElementCounter.structures.front().nextElementId =
            invalidElementCounter.structures.front().members.front().id;
        requireInvalid([&] { validateSupportCollection(invalidElementCounter); },
            "element counter capable of reuse must be rejected");

        auto invalidStructureCounter = collection;
        invalidStructureCounter.nextStructureId =
            invalidStructureCounter.structures.front().id;
        requireInvalid([&] { validateSupportCollection(invalidStructureCounter); },
            "structure counter capable of reuse must be rejected");
    }

    void steelGraphsUseTheCommonModel()
    {
        SupportCollection collection;
        SupportStructure& column = addStructure(collection, "Steel column");
        const auto base = addNode(column, {0.0, 0.0, 0.0});
        const auto top = addNode(column, {0.0, 0.0, 8.0});
        addMember(column, base, top, roundTube());

        SupportStructure& frame = addStructure(collection, "A/V frame");
        const auto left = addNode(frame, {-2.0, 0.0, 0.0});
        const auto right = addNode(frame, {2.0, 0.0, 0.0});
        const auto center = addNode(frame, {0.0, 0.0, 6.0});
        const auto upper = addNode(frame, {0.0, 0.0, 9.0});
        addMember(frame, left, center, roundTube());
        addMember(frame, right, center, roundTube());
        addMember(frame, center, upper, roundTube());
        validateSupportCollection(collection);
    }

    void woodenBentRunUsesTheCommonModel()
    {
        SupportCollection collection;
        SupportStructure& bent = addStructure(collection, "Wood bent");
        const auto leftBottom = addNode(bent, {-2.0, 0.0, 0.0});
        const auto rightBottom = addNode(bent, {2.0, 0.0, 0.0});
        const auto leftTop = addNode(bent, {-2.0, 0.0, 6.0});
        const auto rightTop = addNode(bent, {2.0, 0.0, 6.0});
        const auto leftMid = addNode(bent, {-2.0, 0.0, 3.0});
        const auto rightMid = addNode(bent, {2.0, 0.0, 3.0});
        const auto nextLeftBottom = addNode(bent, {-2.0, 4.0, 0.0});
        const auto nextRightBottom = addNode(bent, {2.0, 4.0, 0.0});
        const auto nextLeftTop = addNode(bent, {-2.0, 4.0, 6.0});
        const auto nextRightTop = addNode(bent, {2.0, 4.0, 6.0});
        addMember(bent, leftBottom, leftTop, timber());
        addMember(bent, rightBottom, rightTop, timber());
        addMember(bent, leftTop, rightTop, timber());
        addMember(bent, leftBottom, rightMid, timber());
        addMember(bent, rightBottom, leftMid, timber());
        addMember(bent, leftMid, rightMid, timber());
        addMember(bent, nextLeftBottom, nextLeftTop, timber());
        addMember(bent, nextRightBottom, nextRightTop, timber());
        addMember(bent, nextLeftTop, nextRightTop, timber());
        addMember(bent, nextLeftBottom, nextRightTop, timber());
        addMember(bent, nextRightBottom, nextLeftTop, timber());
        addMember(bent, leftTop, nextLeftTop, timber());
        addMember(bent, rightTop, nextRightTop, timber());
        validateSupportCollection(collection);
    }
}

int main()
{
    emptyCollectionIsValid();
    allocationIsMonotonicAndDoesNotReuseIds();
    invalidIdsAndReferencesAreRejected();
    invalidGeometryAndCountersAreRejected();
    steelGraphsUseTheCommonModel();
    woodenBentRunUsesTheCommonModel();
}
