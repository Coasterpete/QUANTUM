#include <quantum/engine/Application.hpp>
#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace quantum::engine
{
    int Application::run()
    {
        if (!SDL_Init(SDL_INIT_VIDEO))
        {
            throw std::runtime_error(
                std::string("SDL_Init failed: ") + SDL_GetError()
            );
        }

        // SDL render-category chatter (backend/loader info) is noise during
        // normal interactive use. Warnings and errors stay visible; Debug
        // additionally shows render-category warnings such as Vulkan
        // validation-layer diagnostics. Release stays quieter than Debug.
#ifdef NDEBUG
        SDL_SetLogPriority(SDL_LOG_CATEGORY_RENDER, SDL_LOG_PRIORITY_ERROR);
#else
        SDL_SetLogPriority(SDL_LOG_CATEGORY_RENDER, SDL_LOG_PRIORITY_WARN);
#endif

        SDL_Window* window = SDL_CreateWindow(
            "QUANTUM",
            1600,
            900,
            SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE
        );

        if (window == nullptr)
        {
            const std::string error = SDL_GetError();
            SDL_Quit();

            throw std::runtime_error(
                std::string("SDL_CreateWindow failed: ") + error
            );
        }

        try
        {
            {
                quantum::coaster::AuthoredTrack authoredTrack =
                    quantum::coaster::createDefaultAuthoredTrack();
                quantum::editor::CenterlineVisualization centerline =
                    quantum::editor::createCenterlineVisualization(
                        authoredTrack
                    );

                SDL_LogInfo(
                    SDL_LOG_CATEGORY_APPLICATION,
                    "Core reference curves: %zu line vertices across %zu "
                    "authored section(s), bounds "
                    "[(%.6f, %.6f, %.6f), (%.6f, %.6f, %.6f)].",
                    centerline.vertices.size(),
                    authoredTrack.sectionCount(),
                    centerline.minimumPosition.x,
                    centerline.minimumPosition.y,
                    centerline.minimumPosition.z,
                    centerline.maximumPosition.x,
                    centerline.maximumPosition.y,
                    centerline.maximumPosition.z
                );

                quantum::renderer::VulkanContext vulkan;
                vulkan.initialize(
                    window,
                    centerline.vertices,
                    centerline.verticesPerCurve
                );

                quantum::editor::EditorUi editorUi;
                editorUi.initialize(
                    window,
                    vulkan,
                    authoredTrack,
                    centerline.minimumPosition,
                    centerline.maximumPosition
                );
                editorUi.setCenterlineSections(centerline.sectionSlices);

                bool running = true;

                while (running)
                {
                    SDL_Event event{};

                    while (SDL_PollEvent(&event))
                    {
                        editorUi.processEvent(event);

                        if (event.type == SDL_EVENT_QUIT)
                        {
                            running = false;
                        }
                    }

                    if (running)
                    {
                        if ((SDL_GetWindowFlags(window)
                            & SDL_WINDOW_MINIMIZED) != 0)
                        {
                            SDL_Delay(10);
                            continue;
                        }

                        // Editor requests are queued during render and
                        // applied here against a candidate copy of the
                        // authored document. The candidate becomes the
                        // committed document only after Core generation and
                        // the dynamic GPU update both accept it.
                        const auto requestedCommand =
                            editorUi.takeTrackCommand();
                        const auto requestedLengthEdit =
                            editorUi.takeSectionLengthEdit();
                        const auto requestedRegionCommand =
                            editorUi.takeRegionCommand();
                        const auto requestedValueEdit =
                            editorUi.takeProfileEndpointValueEdit();
                        const auto requestedTransitionType =
                            editorUi.takeProfileTransitionTypeEdit();
                        const auto requestedSegmentCommand =
                            editorUi.takeProfileSegmentCommand();
                        const auto requestedDistanceEdit =
                            editorUi.takeProfileSegmentDistanceEdit();

                        // Continuous handle drags queue a changed-value or
                        // changed-boundary edit every motion frame; both
                        // success logs stay silent for them. The single
                        // end-of-drag [EDIT] summaries are emitted by
                        // EditorUi on release.
                        const bool continuousDrag =
                            (requestedValueEdit.has_value()
                                && requestedValueEdit->continuous)
                            || requestedDistanceEdit.has_value();

                        quantum::coaster::AuthoredTrack candidateTrack =
                            authoredTrack;
                        bool candidateChanged = false;
                        bool trackStructureChanged = false;
                        bool lengthEditApplied = false;
                        bool valueEditApplied = false;
                        bool boundsApplied = false;
                        // Structural segment commands log after acceptance;
                        // these capture their outcome for that report.
                        bool segmentCommandApplied = false;
                        quantum::coaster::SegmentId splitCreatedId =
                            quantum::coaster::invalidSegmentId;
                        quantum::coaster::SegmentId removeSurvivorId =
                            quantum::coaster::invalidSegmentId;
                        bool regionCommandApplied = false;
                        // Selection the editor should follow after an
                        // accepted structural edit; empty when indices
                        // stay stable.
                        std::optional<std::size_t> selectionAfterCommand;

                        try
                        {
                            if (requestedCommand.has_value())
                            {
                                const quantum::editor::TrackCommand&
                                    command = *requestedCommand;

                                switch (command.type)
                                {
                                case quantum::editor::TrackCommandType::
                                    AppendSection:
                                    candidateTrack.appendSection();
                                    break;
                                case quantum::editor::TrackCommandType::
                                    PrependSection:
                                    candidateTrack.prependSection();
                                    break;
                                case quantum::editor::TrackCommandType::
                                    RemoveSection:
                                    candidateTrack.removeSection(
                                        command.sectionIndex
                                    );
                                    // Prefer whichever region now occupies
                                    // the removed index; fall back to the
                                    // previous final region when the tail
                                    // region was removed.
                                    selectionAfterCommand = std::min(
                                        command.sectionIndex,
                                        candidateTrack.sectionCount() - 1
                                    );
                                    break;
                                case quantum::editor::TrackCommandType::
                                    MoveSectionUp:
                                    candidateTrack.moveSection(
                                        command.sectionIndex,
                                        command.sectionIndex - 1
                                    );
                                    // Follow the moved region to its new
                                    // slot so selection keeps its identity.
                                    selectionAfterCommand =
                                        command.sectionIndex - 1;
                                    break;
                                case quantum::editor::TrackCommandType::
                                    MoveSectionDown:
                                    candidateTrack.moveSection(
                                        command.sectionIndex,
                                        command.sectionIndex + 1
                                    );
                                    selectionAfterCommand =
                                        command.sectionIndex + 1;
                                    break;
                                case quantum::editor::TrackCommandType::
                                    DuplicateSection:
                                    candidateTrack.duplicateSection(
                                        command.sectionIndex
                                    );
                                    // The duplicate occupies the slot right
                                    // after its origin.
                                    selectionAfterCommand =
                                        command.sectionIndex + 1;
                                    break;
                                case quantum::editor::TrackCommandType::
                                    SetSectionLength:
                                    break;
                                }

                                candidateChanged = true;
                                trackStructureChanged = true;
                            }

                            if (!trackStructureChanged
                                && requestedRegionCommand.has_value())
                            {
                                const quantum::editor::RegionCommand&
                                    command = *requestedRegionCommand;

                                // All region-kind mutation rules live in
                                // Core; the application layer only maps
                                // editor commands onto them so the
                                // candidate/commit gate stays uniform.
                                auto& section = candidateTrack.section(
                                    command.sectionIndex);

                                using quantum::editor::RegionCommandType;
                                switch (command.type)
                                {
                                case RegionCommandType::
                                    AppendRateProfiles:
                                    candidateTrack.appendSection();
                                    break;
                                case RegionCommandType::
                                    PrependRateProfiles:
                                    candidateTrack.prependSection();
                                    break;
                                case RegionCommandType::AppendPlanarArc:
                                    candidateTrack.appendSection();
                                    quantum::coaster::
                                        convertSectionToPlanarArc(
                                            candidateTrack.section(
                                                candidateTrack
                                                    .sectionCount()
                                                - 1)
                                        );
                                    break;
                                case RegionCommandType::PrependPlanarArc:
                                    candidateTrack.prependSection();
                                    quantum::coaster::
                                        convertSectionToPlanarArc(
                                            candidateTrack.section(0)
                                        );
                                    break;
                                case RegionCommandType::
                                    InsertAfterRateProfiles:
                                    candidateTrack.insertSectionAfter(
                                        command.sectionIndex,
                                        quantum::coaster::
                                            createRateProfileSection(
                                                quantum::coaster::
                                                    defaultNewSectionLength
                                            )
                                    );
                                    selectionAfterCommand =
                                        command.sectionIndex + 1;
                                    break;
                                case RegionCommandType::
                                    InsertAfterPlanarArc:
                                    {
                                        // Same safe defaults as typed
                                        // append/prepend; conversion owns
                                        // the planar-arc length policy.
                                        quantum::coaster::AuthoredTrackSection
                                            insertedArc =
                                                quantum::coaster::
                                                    createRateProfileSection(
                                                        quantum::coaster::
                                                            defaultNewSectionLength
                                                    );
                                        quantum::coaster::
                                            convertSectionToPlanarArc(
                                                insertedArc
                                            );
                                        candidateTrack.insertSectionAfter(
                                            command.sectionIndex,
                                            insertedArc
                                        );
                                    }
                                    selectionAfterCommand =
                                        command.sectionIndex + 1;
                                    break;
                                case RegionCommandType::
                                    ConvertToRateProfiles:
                                    quantum::coaster::
                                        convertSectionToRateProfiles(
                                            section
                                        );
                                    break;
                                case RegionCommandType::ConvertToPlanarArc:
                                    quantum::coaster::
                                        convertSectionToPlanarArc(section);
                                    break;
                                case RegionCommandType::SetPlanarArcRadius:
                                    quantum::coaster::setPlanarArcRadius(
                                        section,
                                        command.value
                                    );
                                    break;
                                case RegionCommandType::
                                    SetPlanarArcSweptAngle:
                                    quantum::coaster::setPlanarArcSweptAngle(
                                        section,
                                        command.value
                                    );
                                    break;
                                case RegionCommandType::
                                    SetPlanarArcPlaneTilt:
                                    quantum::coaster::setPlanarArcPlaneTilt(
                                        section,
                                        command.value
                                    );
                                    break;
                                case RegionCommandType::
                                    SetPlanarArcBankChange:
                                    quantum::coaster::setPlanarArcBankChange(
                                        section,
                                        command.value
                                    );
                                    break;
                                }

                                candidateChanged = true;
                                regionCommandApplied = true;

                                const bool changesEditors =
                                    command.type
                                        != RegionCommandType::
                                            SetPlanarArcRadius
                                    && command.type
                                        != RegionCommandType::
                                            SetPlanarArcSweptAngle
                                    && command.type
                                        != RegionCommandType::
                                            SetPlanarArcPlaneTilt
                                    && command.type
                                        != RegionCommandType::
                                            SetPlanarArcBankChange;
                                if (changesEditors)
                                {
                                    // Creation and conversion change which
                                    // editor machinery regions support and
                                    // shift indices, so same-frame profile
                                    // edits would refer to stale state.
                                    trackStructureChanged = true;
                                }
                            }

                            if (!trackStructureChanged)
                            {
                                if (requestedLengthEdit.has_value())
                                {
                                    quantum::coaster::setSectionLength(
                                        candidateTrack.section(
                                            requestedLengthEdit
                                                ->sectionIndex),
                                        requestedLengthEdit->length
                                    );
                                    candidateChanged = true;
                                    lengthEditApplied = true;
                                }

                                if (requestedValueEdit.has_value())
                                {
                                    if (!std::isfinite(
                                        requestedValueEdit->value))
                                    {
                                        throw std::invalid_argument(
                                            "the profile rate value must be "
                                            "finite"
                                        );
                                    }

                                    using quantum::editor::ScalarProfileEndpoint;
                                    if (requestedValueEdit->endpoint
                                        != ScalarProfileEndpoint::Begin
                                        && requestedValueEdit->endpoint
                                            != ScalarProfileEndpoint::End)
                                    {
                                        throw std::invalid_argument(
                                            "the profile rate endpoint is "
                                            "invalid"
                                        );
                                    }

                                    const quantum::coaster::ProfileBoundary
                                        boundary =
                                        requestedValueEdit->endpoint
                                            == ScalarProfileEndpoint::Begin
                                        ? quantum::coaster::ProfileBoundary
                                            ::Begin
                                        : quantum::coaster::ProfileBoundary
                                            ::End;

                                    // The Core operation propagates shared
                                    // joint values so C0 continuity holds.
                                    quantum::coaster::setChannelSegmentValue(
                                        quantum::editor::sectionRateChannel(
                                            candidateTrack.section(
                                                requestedValueEdit
                                                    ->sectionIndex),
                                            requestedValueEdit->channel
                                        ),
                                        requestedValueEdit->segmentId,
                                        boundary,
                                        requestedValueEdit->value
                                    );
                                    valueEditApplied = true;
                                    candidateChanged = true;
                                }

                                if (requestedDistanceEdit.has_value())
                                {
                                    if (!std::isfinite(
                                        requestedDistanceEdit->distance))
                                    {
                                        throw std::invalid_argument(
                                            "the moved boundary distance "
                                            "must be finite"
                                        );
                                    }

                                    using quantum::editor::ScalarProfileEndpoint;
                                    if (requestedDistanceEdit->endpoint
                                        != ScalarProfileEndpoint::Begin
                                        && requestedDistanceEdit->endpoint
                                            != ScalarProfileEndpoint::End)
                                    {
                                        throw std::invalid_argument(
                                            "the moved boundary endpoint is "
                                            "invalid"
                                        );
                                    }

                                    const quantum::coaster::ProfileBoundary
                                        boundary =
                                        requestedDistanceEdit->endpoint
                                            == ScalarProfileEndpoint::Begin
                                        ? quantum::coaster::ProfileBoundary
                                            ::Begin
                                        : quantum::coaster::ProfileBoundary
                                            ::End;

                                    quantum::coaster::
                                        moveChannelSegmentBoundary(
                                            quantum::editor::
                                                sectionRateChannel(
                                                    candidateTrack.section(
                                                        requestedDistanceEdit
                                                            ->sectionIndex),
                                                    requestedDistanceEdit
                                                        ->channel
                                                ),
                                            requestedDistanceEdit->segmentId,
                                            boundary,
                                            requestedDistanceEdit->distance
                                        );
                                    candidateChanged = true;
                                }

                                if (requestedSegmentCommand.has_value())
                                {
                                    const quantum::editor::
                                        ProfileSegmentCommand& command =
                                        *requestedSegmentCommand;
                                    quantum::coaster::ChannelProfile&
                                        channelProfile =
                                        quantum::editor::sectionRateChannel(
                                            candidateTrack.section(
                                                command.sectionIndex),
                                            command.channel
                                        );

                                    switch (command.operation)
                                    {
                                    case quantum::editor::
                                        ProfileSegmentOperation::Split:
                                        splitCreatedId =
                                            quantum::coaster::
                                                splitChannelSegment(
                                                    channelProfile,
                                                    command.segmentId,
                                                    command.splitDistance
                                                );
                                        break;
                                    case quantum::editor::
                                        ProfileSegmentOperation::Remove:
                                        removeSurvivorId =
                                            quantum::coaster::
                                                removeChannelSegment(
                                                    channelProfile,
                                                    command.segmentId
                                                );
                                        break;
                                    }

                                    segmentCommandApplied = true;
                                    candidateChanged = true;
                                }

                                if (requestedTransitionType.has_value())
                                {
                                    auto& candidateChannel =
                                        quantum::editor::sectionRateChannel(
                                            candidateTrack.section(
                                                requestedTransitionType
                                                    ->sectionIndex),
                                            requestedTransitionType->channel
                                        );
                                    auto* candidateTransition =
                                        quantum::coaster::
                                            findChannelSegmentTransition(
                                                candidateChannel,
                                                requestedTransitionType
                                                    ->segmentId
                                            );

                                    if (candidateTransition == nullptr
                                        || !quantum::editor::
                                            trySetTransitionTypePreset(
                                                *candidateTransition,
                                                requestedTransitionType->type))
                                    {
                                        throw std::invalid_argument(
                                            "the transition preset is "
                                            "unsupported or the segment is "
                                            "unknown"
                                        );
                                    }

                                    candidateChanged = true;
                                }
                            }
                            else if (
                                requestedLengthEdit.has_value()
                                || requestedValueEdit.has_value()
                                || requestedTransitionType.has_value()
                                || requestedDistanceEdit.has_value()
                                || requestedSegmentCommand.has_value())
                            {
                                SDL_LogInfo(
                                    SDL_LOG_CATEGORY_APPLICATION,
                                    "Dropped same-frame profile edits that "
                                    "referred to pre-command section "
                                    "indices."
                                );
                            }

                            if (candidateChanged)
                            {
                                quantum::editor::CenterlineVisualization
                                    candidateCenterline =
                                        quantum::editor::
                                            createCenterlineVisualization(
                                                candidateTrack
                                            );

                                editorUi.setCenterlineBounds(
                                    candidateCenterline.minimumPosition,
                                    candidateCenterline.maximumPosition
                                );
                                editorUi.setCenterlineSections(
                                    candidateCenterline.sectionSlices
                                );
                                boundsApplied = true;
                                vulkan.updateTrackCurveVertices(
                                    candidateCenterline.vertices,
                                    candidateCenterline.verticesPerCurve
                                );

                                centerline = std::move(candidateCenterline);
                                authoredTrack = std::move(candidateTrack);

                                if (!continuousDrag)
                                {
                                    SDL_LogInfo(
                                        SDL_LOG_CATEGORY_APPLICATION,
                                        "Authored edit accepted; track now "
                                        "has %zu section(s), %zu centerline "
                                        "samples.",
                                        authoredTrack.sectionCount(),
                                        centerline.vertices.size()
                                    );
                                }

                                if (valueEditApplied
                                    && requestedValueEdit
                                    && !continuousDrag)
                                {
                                    SDL_LogInfo(
                                        SDL_LOG_CATEGORY_APPLICATION,
                                        "[EDIT] section=%zu channel=%d "
                                        "endpoint=%d value=%.6f segment=%u",
                                        requestedValueEdit->sectionIndex,
                                        static_cast<int>(
                                            requestedValueEdit->channel),
                                        static_cast<int>(
                                            requestedValueEdit->endpoint),
                                        requestedValueEdit->value,
                                        requestedValueEdit->segmentId
                                    );
                                }

                                if (segmentCommandApplied
                                    && requestedSegmentCommand)
                                {
                                    const quantum::editor::
                                        ProfileSegmentCommand& command =
                                        *requestedSegmentCommand;
                                    switch (command.operation)
                                    {
                                    case quantum::editor::
                                        ProfileSegmentOperation::Split:
                                        SDL_LogInfo(
                                            SDL_LOG_CATEGORY_APPLICATION,
                                            "[EDIT] section=%zu channel=%d "
                                            "segment=%u split distance=%.6f "
                                            "newSegment=%u",
                                            command.sectionIndex,
                                            static_cast<int>(command.channel),
                                            command.segmentId,
                                            command.splitDistance,
                                            splitCreatedId
                                        );
                                        break;
                                    case quantum::editor::
                                        ProfileSegmentOperation::Remove:
                                        SDL_LogInfo(
                                            SDL_LOG_CATEGORY_APPLICATION,
                                            "[EDIT] section=%zu channel=%d "
                                            "segment=%u removed "
                                            "mergedInto=%u",
                                            command.sectionIndex,
                                            static_cast<int>(command.channel),
                                            command.segmentId,
                                            removeSurvivorId
                                        );
                                        break;
                                    }
                                }

                                if (regionCommandApplied
                                    && requestedRegionCommand)
                                {
                                    const quantum::editor::RegionCommand&
                                        command = *requestedRegionCommand;
                                    using quantum::editor::
                                        RegionCommandType;

                                    const bool isCreate =
                                        command.type ==
                                            RegionCommandType::
                                                AppendRateProfiles
                                        || command.type ==
                                            RegionCommandType::
                                                PrependRateProfiles
                                        || command.type ==
                                            RegionCommandType::
                                                InsertAfterRateProfiles
                                        || command.type ==
                                            RegionCommandType::
                                                AppendPlanarArc
                                        || command.type ==
                                            RegionCommandType::
                                                PrependPlanarArc
                                        || command.type ==
                                            RegionCommandType::
                                                InsertAfterPlanarArc;

                                    if (isCreate)
                                    {
                                        // Appended regions land at the end
                                        // of the ordering, prepended ones
                                        // at the front, and inserted ones
                                        // right after their anchor region.
                                        const bool prepended =
                                            command.type ==
                                                RegionCommandType::
                                                    PrependRateProfiles
                                            || command.type ==
                                                RegionCommandType::
                                                    PrependPlanarArc;
                                        const bool insertedAfter =
                                            command.type ==
                                                RegionCommandType::
                                                    InsertAfterRateProfiles
                                            || command.type ==
                                                RegionCommandType::
                                                    InsertAfterPlanarArc;
                                        const std::size_t createdIndex =
                                            insertedAfter
                                                ? command.sectionIndex + 1
                                                : prepended
                                                    ? 0
                                                    : authoredTrack
                                                        .sectionCount()
                                                    - 1;
                                        const char* verb =
                                            prepended ? "prepended"
                                            : insertedAfter ? "inserted"
                                                            : "appended";

                                        if (command.type ==
                                            RegionCommandType::
                                                AppendRateProfiles
                                            || command.type ==
                                            RegionCommandType::
                                                PrependRateProfiles
                                            || command.type ==
                                            RegionCommandType::
                                                InsertAfterRateProfiles)
                                        {
                                            SDL_LogInfo(
                                                SDL_LOG_CATEGORY_APPLICATION,
                                                "[EDIT] %s region=%zu "
                                                "kind=rateProfiles",
                                                verb,
                                                createdIndex
                                            );
                                        }
                                        else
                                        {
                                            const auto& arc =
                                                std::get<quantum::coaster::
                                                    PlanarArcRegion>(
                                                    std::get<quantum::
                                                        coaster::
                                                            GeometryRegion>(
                                                        authoredTrack
                                                            .section(
                                                                createdIndex)
                                                            .region)
                                                    .construction);
                                            SDL_LogInfo(
                                                SDL_LOG_CATEGORY_APPLICATION,
                                                "[EDIT] %s region=%zu "
                                                "kind=planarArc "
                                                "radius=%.6f "
                                                "sweptAngle=%.6f "
                                                "planeTilt=%.6f "
                                                "bankChange=%.6f",
                                                verb,
                                                createdIndex,
                                                arc.radius,
                                                arc.sweptAngle,
                                                arc.planeTilt,
                                                arc.bankChange
                                            );
                                        }
                                    }
                                    else if (command.type ==
                                        RegionCommandType::
                                            ConvertToRateProfiles)
                                    {
                                        SDL_LogInfo(
                                            SDL_LOG_CATEGORY_APPLICATION,
                                            "[EDIT] section=%zu "
                                            "kind=rateProfiles",
                                            command.sectionIndex
                                        );
                                    }
                                    else
                                    {
                                        const auto& arc =
                                            std::get<quantum::coaster::
                                                PlanarArcRegion>(
                                                std::get<quantum::coaster::
                                                    GeometryRegion>(
                                                    authoredTrack.section(
                                                        command
                                                            .sectionIndex)
                                                    .region)
                                                .construction);

                                        if (command.type ==
                                            RegionCommandType::
                                                ConvertToPlanarArc)
                                        {
                                            SDL_LogInfo(
                                                SDL_LOG_CATEGORY_APPLICATION,
                                                "[EDIT] section=%zu "
                                                "kind=planarArc "
                                                "radius=%.6f "
                                                "sweptAngle=%.6f "
                                                "planeTilt=%.6f "
                                                "bankChange=%.6f",
                                                command.sectionIndex,
                                                arc.radius,
                                                arc.sweptAngle,
                                                arc.planeTilt,
                                                arc.bankChange
                                            );
                                        }
                                        else
                                        {
                                            SDL_LogInfo(
                                                SDL_LOG_CATEGORY_APPLICATION,
                                                "[EDIT] section=%zu "
                                                "planarArc radius=%.6f "
                                                "sweptAngle=%.6f "
                                                "planeTilt=%.6f "
                                                "bankChange=%.6f",
                                                command.sectionIndex,
                                                arc.radius,
                                                arc.sweptAngle,
                                                arc.planeTilt,
                                                arc.bankChange
                                            );
                                        }
                                    }
                                }
                            }
                        }
                        catch (const std::exception& exception)
                        {
                            if (boundsApplied)
                            {
                                editorUi.setCenterlineBounds(
                                    centerline.minimumPosition,
                                    centerline.maximumPosition
                                );
                                editorUi.setCenterlineSections(
                                    centerline.sectionSlices
                                );
                            }

                            SDL_LogError(
                                SDL_LOG_CATEGORY_APPLICATION,
                                "Authored edit was rejected: %s",
                                exception.what()
                            );
                        }

                        // Resynchronize the edit buffers with the committed
                        // document so rejected edits revert instead of
                        // lingering in the UI.
                        if (lengthEditApplied && requestedLengthEdit)
                        {
                            editorUi.synchronizeSectionLength(
                                quantum::coaster::sectionLength(
                                    authoredTrack.section(
                                        requestedLengthEdit->sectionIndex))
                            );
                        }

                        if (valueEditApplied && requestedValueEdit)
                        {
                            auto& committedChannel =
                                quantum::editor::sectionRateChannel(
                                    authoredTrack.section(
                                        requestedValueEdit->sectionIndex),
                                    requestedValueEdit->channel
                                );
                            auto* committedSegment =
                                quantum::coaster::
                                    findChannelSegmentTransition(
                                        committedChannel,
                                        requestedValueEdit->segmentId
                                    );
                            editorUi.synchronizeSegmentValueEnd(
                                requestedValueEdit->channel,
                                requestedValueEdit->segmentId,
                                committedSegment != nullptr
                                    ? committedSegment->valueEnd
                                    : 0.0
                            );
                        }

                        if (regionCommandApplied && requestedRegionCommand)
                        {
                            const auto& committedSection =
                                authoredTrack.section(
                                    requestedRegionCommand->sectionIndex);

                            if (committedSection.kind ==
                                quantum::coaster::RegionKind::Geometry)
                            {
                                editorUi.synchronizePlanarArcParams(
                                    std::get<quantum::coaster::
                                        PlanarArcRegion>(
                                        std::get<quantum::coaster::
                                            GeometryRegion>(
                                            committedSection.region)
                                        .construction));
                            }

                            editorUi.synchronizeSectionLength(
                                quantum::coaster::sectionLength(
                                    committedSection)
                            );
                        }

                        // Structural edits that move or create regions
                        // re-target the selection here, after the buffers
                        // above have been resynchronized: selectSection
                        // then refreshes every editor surface from the
                        // region the user should now be working on.
                        if (selectionAfterCommand.has_value())
                        {
                            editorUi.selectSection(*selectionAfterCommand);
                        }

                        editorUi.beginFrame(vulkan);
                        vulkan.drawFrame(
                            [](VkCommandBuffer commandBuffer, void* userData)
                            {
                                static_cast<quantum::editor::EditorUi*>(
                                    userData
                                )->render(commandBuffer);
                            },
                            &editorUi
                        );
                    }
                }
            }
        }
        catch (...)
        {
            SDL_DestroyWindow(window);
            SDL_Quit();
            throw;
        }

        SDL_DestroyWindow(window);
        SDL_Quit();

        return 0;
    }
}
