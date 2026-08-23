#include <quantum/engine/Application.hpp>
#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL.h>

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

        SDL_SetLogPriority(SDL_LOG_CATEGORY_RENDER, SDL_LOG_PRIORITY_INFO);

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
                vulkan.initialize(window, centerline.vertices);

                quantum::editor::EditorUi editorUi;
                editorUi.initialize(
                    window,
                    vulkan,
                    authoredTrack,
                    centerline.minimumPosition,
                    centerline.maximumPosition
                );

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
                        const auto requestedValueEdit =
                            editorUi.takeProfileEndpointValueEdit();
                        const auto requestedTransitionType =
                            editorUi.takeProfileTransitionTypeEdit();

                        // Continuous handle drags queue a changed-value
                        // edit every motion frame; both success logs stay
                        // silent for them. The single end-of-drag [EDIT]
                        // summary is emitted by EditorUi on release.
                        const bool continuousDrag =
                            requestedValueEdit.has_value()
                            && requestedValueEdit->continuous;

                        quantum::coaster::AuthoredTrack candidateTrack =
                            authoredTrack;
                        bool candidateChanged = false;
                        bool trackStructureChanged = false;
                        bool lengthEditApplied = false;
                        bool valueEditApplied = false;
                        bool boundsApplied = false;

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
                                    break;
                                case quantum::editor::TrackCommandType::
                                    MoveSectionUp:
                                    candidateTrack.moveSection(
                                        command.sectionIndex,
                                        command.sectionIndex - 1
                                    );
                                    break;
                                case quantum::editor::TrackCommandType::
                                    MoveSectionDown:
                                    candidateTrack.moveSection(
                                        command.sectionIndex,
                                        command.sectionIndex + 1
                                    );
                                    break;
                                case quantum::editor::TrackCommandType::
                                    SetSectionLength:
                                    break;
                                }

                                candidateChanged = true;
                                trackStructureChanged = true;
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

                                    quantum::math::ScalarTransition&
                                        candidateProfile =
                                        quantum::editor::sectionRateProfile(
                                            candidateTrack.section(
                                                requestedValueEdit
                                                    ->sectionIndex),
                                            requestedValueEdit->channel
                                        );

                                    double* candidateValue = nullptr;

                                    switch (requestedValueEdit->endpoint)
                                    {
                                    case quantum::editor::
                                        ScalarProfileEndpoint::Begin:
                                        candidateValue =
                                            &candidateProfile.valueBegin;
                                        break;
                                    case quantum::editor::
                                        ScalarProfileEndpoint::End:
                                        candidateValue =
                                            &candidateProfile.valueEnd;
                                        break;
                                    case quantum::editor::
                                        ScalarProfileEndpoint::None:
                                        break;
                                    }

                                    if (candidateValue == nullptr)
                                    {
                                        throw std::invalid_argument(
                                            "the profile rate endpoint is "
                                            "invalid"
                                        );
                                    }

                                    *candidateValue =
                                        requestedValueEdit->value;
                                    valueEditApplied = true;
                                    candidateChanged = true;
                                }

                                if (requestedTransitionType.has_value())
                                {
                                    if (!quantum::editor::
                                        trySetTransitionTypePreset(
                                            quantum::editor::
                                                sectionRateProfile(
                                                    candidateTrack.section(
                                                        requestedTransitionType
                                                            ->sectionIndex),
                                                    requestedTransitionType
                                                        ->channel
                                                ),
                                            requestedTransitionType->type))
                                    {
                                        throw std::invalid_argument(
                                            "the transition preset is "
                                            "unsupported"
                                        );
                                    }

                                    candidateChanged = true;
                                }
                            }
                            else if (
                                requestedLengthEdit.has_value()
                                || requestedValueEdit.has_value()
                                || requestedTransitionType.has_value())
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
                                boundsApplied = true;
                                vulkan.updateTrackCurveVertices(
                                    candidateCenterline.vertices
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
                                        "endpoint=%d value=%.6f",
                                        requestedValueEdit->sectionIndex,
                                        static_cast<int>(
                                            requestedValueEdit->channel),
                                        static_cast<int>(
                                            requestedValueEdit->endpoint),
                                        requestedValueEdit->value
                                    );
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
                            editorUi.synchronizeProfileValueEnd(
                                requestedValueEdit->channel,
                                quantum::editor::sectionRateProfile(
                                    authoredTrack.section(
                                        requestedValueEdit->sectionIndex),
                                    requestedValueEdit->channel
                                ).valueEnd
                            );
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
