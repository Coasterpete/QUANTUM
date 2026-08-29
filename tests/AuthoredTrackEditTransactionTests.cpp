#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>

#include <glm/geometric.hpp>

#include <array>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using quantum::coaster::AuthoredTrack;
    using quantum::coaster::PlanarArcRegion;
    using quantum::coaster::RiderLocalGeometryState;
    using quantum::editor::AuthoredTrackEditTransaction;

    void require(const bool condition, const std::string_view message)
    {
        if (!condition)
        {
            throw std::runtime_error(std::string(message));
        }
    }

    template<typename Exception, typename Function>
    void requireThrows(Function&& function, const std::string_view message)
    {
        try
        {
            function();
        }
        catch (const Exception&)
        {
            return;
        }

        throw std::runtime_error(std::string(message));
    }

    [[nodiscard]] const PlanarArcRegion& planarArc(
        const AuthoredTrack& track,
        const std::size_t sectionIndex)
    {
        return std::get<PlanarArcRegion>(
            std::get<quantum::coaster::GeometryRegion>(
                track.section(sectionIndex).region).construction);
    }

    void requireSameGeneratedTrack(
        const std::vector<RiderLocalGeometryState>& expected,
        const AuthoredTrack& track,
        const std::string_view message)
    {
        const std::vector<RiderLocalGeometryState> actual =
            quantum::coaster::integrateAuthoredTrack(track, 1.0);
        require(actual.size() == expected.size(), message);

        for (std::size_t index = 0; index < expected.size(); ++index)
        {
            require(actual[index].distance == expected[index].distance
                    && glm::length(actual[index].position
                        - expected[index].position) == 0.0
                    && glm::length(actual[index].frame.tangent
                        - expected[index].frame.tangent) == 0.0
                    && glm::length(actual[index].frame.lateral
                        - expected[index].frame.lateral) == 0.0
                    && glm::length(actual[index].frame.up
                        - expected[index].frame.up) == 0.0,
                message);
        }
    }

    void rejectedSectionLengthRestoresCommittedBuffer()
    {
        AuthoredTrack committed = quantum::coaster::createDefaultAuthoredTrack();
        const double committedLength =
            quantum::coaster::sectionLength(committed.section(0));
        const std::vector<RiderLocalGeometryState> generatedBefore =
            quantum::coaster::integrateAuthoredTrack(committed, 1.0);
        double editorLengthBuffer = -1.0;

        AuthoredTrackEditTransaction transaction{committed};
        transaction.requestSectionLengthBufferSync();
        requireThrows<std::invalid_argument>(
            [&transaction]
            {
                quantum::coaster::setSectionLength(
                    transaction.candidate().section(0),
                    -1.0);
            },
            "invalid section length must be rejected");

        if (transaction.sectionLengthBufferSyncRequested())
        {
            editorLengthBuffer = quantum::coaster::sectionLength(
                committed.section(0));
        }

        require(!transaction.committed(),
            "a rejected length edit must not commit its candidate");
        require(editorLengthBuffer == committedLength,
            "a rejected length edit must restore the committed UI value");
        require(quantum::coaster::sectionLength(committed.section(0))
                == committedLength,
            "a rejected length edit must leave the document unchanged");
        requireSameGeneratedTrack(generatedBefore, committed,
            "a rejected length edit must leave generated track unchanged");
    }

    void rejectedPlanarArcRadiusRestoresCommittedBuffers()
    {
        AuthoredTrack committed = quantum::coaster::createNewDocument();
        quantum::coaster::convertSectionToPlanarArc(committed.section(0));
        const PlanarArcRegion committedArc = planarArc(committed, 0);
        const std::vector<RiderLocalGeometryState> generatedBefore =
            quantum::coaster::integrateAuthoredTrack(committed, 1.0);
        std::array<double, 4> editorArcBuffers{
            -1.0,
            0.0,
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity()
        };

        AuthoredTrackEditTransaction transaction{committed};
        transaction.requestRegionBufferSync();
        requireThrows<std::invalid_argument>(
            [&transaction]
            {
                quantum::coaster::setPlanarArcRadius(
                    transaction.candidate().section(0),
                    0.0);
            },
            "invalid planar-arc radius must be rejected");

        if (transaction.regionBufferSyncRequested())
        {
            const PlanarArcRegion& authoritative = planarArc(committed, 0);
            editorArcBuffers = {
                authoritative.radius,
                authoritative.sweptAngle,
                authoritative.planeTilt,
                authoritative.bankChange
            };
        }

        require(!transaction.committed(),
            "a rejected planar-arc edit must not commit its candidate");
        require(editorArcBuffers[0] == committedArc.radius
                && editorArcBuffers[1] == committedArc.sweptAngle
                && editorArcBuffers[2] == committedArc.planeTilt
                && editorArcBuffers[3] == committedArc.bankChange,
            "a rejected planar-arc edit must restore committed UI values");
        require(planarArc(committed, 0).radius == committedArc.radius
                && planarArc(committed, 0).sweptAngle
                    == committedArc.sweptAngle,
            "a rejected planar-arc edit must leave the document unchanged");
        requireSameGeneratedTrack(generatedBefore, committed,
            "a rejected planar-arc edit must leave generated track unchanged");
    }

    void rejectedStructuralCandidateDoesNotPublishSelection()
    {
        AuthoredTrack committed = quantum::coaster::createNewDocument();
        committed.appendSection();
        quantum::coaster::setSectionLength(committed.section(0), 10.0);
        quantum::coaster::setSectionLength(committed.section(1), 15.0);
        const std::vector<RiderLocalGeometryState> generatedBefore =
            quantum::coaster::integrateAuthoredTrack(committed, 1.0);
        std::size_t editorSelection = 0;

        AuthoredTrackEditTransaction transaction{committed};
        transaction.candidate().duplicateSection(0);
        transaction.stageSelectionAfterCommit(1);

        // Corrupt only the candidate to model a later validation/solve
        // rejection after the structural mutation has computed a selection.
        transaction.candidate().section(1).length = 0.0;
        requireThrows<std::invalid_argument>(
            [&transaction]
            {
                static_cast<void>(quantum::coaster::integrateAuthoredTrack(
                    transaction.candidate(),
                    1.0));
            },
            "invalid structural candidate must fail validation");

        if (const auto selection = transaction.selectionAfterCommit())
        {
            editorSelection = *selection;
        }

        require(!transaction.committed(),
            "a rejected structural candidate must not commit");
        require(committed.sectionCount() == 2,
            "a rejected structural candidate must leave the document unchanged");
        require(editorSelection == 0,
            "a rejected structural candidate must leave selection unchanged");
        requireSameGeneratedTrack(generatedBefore, committed,
            "a rejected structural candidate must leave generated track unchanged");
    }
}

int main()
{
    try
    {
        rejectedSectionLengthRestoresCommittedBuffer();
        rejectedPlanarArcRadiusRestoresCommittedBuffers();
        rejectedStructuralCandidateDoesNotPublishSelection();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Authored-track transaction test failure: "
                  << exception.what() << '\n';
        return 1;
    }

    std::cout << "Authored-track transaction tests passed.\n";
    return 0;
}
