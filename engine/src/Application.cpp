#include <quantum/engine/Application.hpp>
#include <quantum/coaster/AuthoredTrack.hpp>
#include <quantum/coaster/ChannelProfileEditing.hpp>
#include <quantum/coaster/CircuitCompletion.hpp>
#include <quantum/coaster/CoasterDocument.hpp>
#include <quantum/editor/AuthoredTrackEditTransaction.hpp>
#include <quantum/editor/CenterlineVisualization.hpp>
#include <quantum/editor/DocumentHistory.hpp>
#include <quantum/editor/DocumentState.hpp>
#include <quantum/editor/EditorUi.hpp>
#include <quantum/editor/FramePerformanceTelemetry.hpp>
#include <quantum/editor/PlatformDialogs.hpp>
#include <quantum/editor/PreviewSmoke.hpp>
#include <quantum/editor/RegionSelection.hpp>
#include <quantum/editor/RiderLoadDiagnostics.hpp>
#include <quantum/editor/SimulationPreview.hpp>
#include <quantum/editor/TransitionTypePresets.hpp>
#include <quantum/engine/Logging.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <expected>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace
{
    constexpr double piRadians = 3.14159265358979323846;
    constexpr double radiansPerDegree = piRadians / 180.0;

    [[nodiscard]] bool interactiveAuthoringDemoEnabled() noexcept
    {
        const char* const value =
            std::getenv("QUANTUM_INTERACTIVE_AUTHORING_DEMO");
        return value != nullptr && value[0] != '\0'
            && std::string_view{value} != "0";
    }

    void setSingleSegmentChannel(
        quantum::coaster::ChannelProfile& channel,
        const double valueBegin,
        const double valueEnd,
        const quantum::math::TransitionType transitionType)
    {
        quantum::coaster::ProfileSegment& segment =
            channel.segments.front();
        segment.transition.valueBegin = valueBegin;
        segment.transition.valueEnd = valueEnd;
        segment.transition.transitionType = transitionType;
    }

    void shapeRateProfileSection(
        quantum::coaster::AuthoredTrackSection& section,
        const double rollBegin,
        const double rollEnd,
        const double pitchBegin,
        const double pitchEnd,
        const double yawBegin,
        const double yawEnd)
    {
        quantum::coaster::GeometricSection& profiles =
            section.rateProfileRegion().rateProfiles;
        setSingleSegmentChannel(
            profiles.roll,
            rollBegin,
            rollEnd,
            quantum::math::TransitionType::Smoothstep
        );
        setSingleSegmentChannel(
            profiles.pitch,
            pitchBegin,
            pitchEnd,
            quantum::math::TransitionType::CosineEaseInOut
        );
        setSingleSegmentChannel(
            profiles.yaw,
            yawBegin,
            yawEnd,
            quantum::math::TransitionType::Smootherstep
        );
    }

    [[nodiscard]] quantum::coaster::AuthoredTrack
    createInteractiveAuthoringDemoTrack()
    {
        quantum::coaster::AuthoredTrack track;

        track.appendSection();
        quantum::coaster::setSectionLength(track.section(0), 20.0);

        track.appendSection();
        quantum::coaster::convertSectionToPlanarArc(track.section(1));
        quantum::coaster::setPlanarArcRadius(track.section(1), 32.0);
        quantum::coaster::setPlanarArcSweptAngle(
            track.section(1),
            70.0 * radiansPerDegree
        );
        quantum::coaster::setPlanarArcPlaneTilt(
            track.section(1),
            12.0 * radiansPerDegree
        );
        quantum::coaster::setPlanarArcBankChange(
            track.section(1),
            22.0 * radiansPerDegree
        );

        track.appendSection();
        quantum::coaster::setSectionLength(track.section(2), 36.0);
        shapeRateProfileSection(
            track.section(2),
            0.002,
            0.014,
            -0.004,
            0.007,
            0.006,
            0.003
        );

        track.appendSection();
        quantum::coaster::setSectionLength(track.section(3), 28.0);
        shapeRateProfileSection(
            track.section(3),
            0.012,
            -0.004,
            0.006,
            0.002,
            -0.003,
            0.004
        );

        return track;
    }

    struct PreparedDocument
    {
        quantum::coaster::AuthoredTrack track;
        quantum::editor::CenterlineVisualization centerline;
        quantum::coaster::RiderLoadHistory riderLoads;
    };

    // Both interactive Open and developer smoke startup use this complete
    // read/parse/generate/accept path before publishing a document.
    [[nodiscard]] std::expected<PreparedDocument, std::string>
    prepareDocument(const std::filesystem::path& path)
    {
        std::ifstream input(path, std::ios::binary);
        if (!input)
            return std::unexpected(
                "Could not open document: " + path.string());
        const std::string json{
            std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()};
        auto track = quantum::coaster::deserializeCoasterDocument(json);
        if (!track) return std::unexpected(track.error());

        try
        {
            auto centerline = quantum::editor::createCenterlineVisualization(
                *track, track->trackStyle());
            auto riderLoads =
                quantum::editor::evaluateRiderLoadDiagnostics(*track);
            quantum::editor::AuthoredTrackEditTransaction transaction{*track};
            transaction.requireAcceptableRiderLoads(riderLoads);
            return PreparedDocument{
                std::move(*track),
                std::move(centerline),
                std::move(riderLoads)};
        }
        catch (const std::exception& error)
        {
            return std::unexpected(std::string{error.what()});
        }
    }
}

namespace quantum::engine
{
    int Application::run()
    {
        return runImpl(nullptr);
    }

    int Application::run(
        const editor::PreviewSmokeOptions& previewSmokeOptions)
    {
        const auto validInput =
            editor::validatePreviewSmokeInput(previewSmokeOptions);
        if (!validInput)
            throw std::invalid_argument(validInput.error());
        return runImpl(&previewSmokeOptions);
    }

    int Application::runImpl(
        const editor::PreviewSmokeOptions* const previewSmokeOptions)
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

        int applicationExitCode = 0;
        try
        {
            {
                std::optional<PreparedDocument> startupDocument;
                if (previewSmokeOptions != nullptr)
                {
                    auto prepared = prepareDocument(
                        previewSmokeOptions->documentPath);
                    if (!prepared)
                        throw std::runtime_error(
                            "Preview smoke document load failed: "
                            + prepared.error());
                    startupDocument = std::move(*prepared);
                }

                quantum::coaster::AuthoredTrack authoredTrack =
                    startupDocument
                    ? std::move(startupDocument->track)
                    : interactiveAuthoringDemoEnabled()
                        ? createInteractiveAuthoringDemoTrack()
                        : quantum::coaster::createNewDocument();
                quantum::editor::DocumentState documentState;
                if (previewSmokeOptions != nullptr)
                    documentState.setOpenDocument(
                        previewSmokeOptions->documentPath);
                quantum::editor::DocumentHistory documentHistory;
                documentHistory.reset(authoredTrack);
                quantum::editor::CenterlineVisualizationCache
                    centerlineCache;
                centerlineCache.setTrackStyle(authoredTrack.trackStyle());
                if (startupDocument)
                    centerlineCache.replace(
                        std::move(startupDocument->centerline));
                else
                    static_cast<void>(
                        centerlineCache.rebuildIfDirty(authoredTrack));
                const quantum::editor::CenterlineVisualization& centerline =
                    centerlineCache.visualization();

                quantum::logging::logMessagef(
                    quantum::logging::LogLevel::Info,
                    "APP",
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
                    centerline.verticesPerCurve,
                    centerline.renderableTrack
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
                editorUi.setCenterlineVisualization(centerline);
                editorUi.setRiderLoadHistory(startupDocument
                    ? std::move(startupDocument->riderLoads)
                    : quantum::editor::evaluateRiderLoadDiagnostics(
                        authoredTrack));
                editorUi.updateWindowTitle(documentState.windowTitle());
                editorUi.setHistoryAvailability(
                    documentHistory.canUndo(),
                    documentHistory.canRedo());

                quantum::editor::SimulationPreview simulationPreview;
                std::uint64_t simulationTrackGeneration =
                    centerlineCache.generation();
                quantum::coaster::LayoutMode simulationLayoutMode =
                    authoredTrack.layoutMode();
                std::uint64_t uploadedSimulationVertexGeneration =
                    std::numeric_limits<std::uint64_t>::max();

                const auto rebuildSimulationPreview = [&]
                {
                    if (simulationPreview.rebuild(authoredTrack))
                    {
                        quantum::logging::logMessagef(
                            quantum::logging::LogLevel::Info,
                            "SIM",
                            "Four-car preview initialized at %.3f m and "
                            "%.3f m/s",
                            simulationPreview.dynamicsState()
                                ->generalizedReferenceLocation.stationMeters,
                            simulationPreview.speedMetersPerSecond());
                    }
                    else
                    {
                        quantum::logging::logMessagef(
                            quantum::logging::LogLevel::Warning,
                            "SIM",
                            "%s",
                            simulationPreview.error().c_str());
                    }
                };

                const auto publishSimulationStatus = [&]
                {
                    if (!simulationPreview.isAvailable())
                    {
                        editorUi.setSimulationUnavailable(
                            simulationPreview.error());
                        return;
                    }

                    quantum::editor::SimulationPlaybackState uiState =
                        quantum::editor::SimulationPlaybackState::Stopped;
                    switch (simulationPreview.playbackState())
                    {
                    case quantum::editor::SimulationPreview::PlaybackState::
                        Stopped:
                        break;
                    case quantum::editor::SimulationPreview::PlaybackState::
                        Playing:
                        uiState = quantum::editor::
                            SimulationPlaybackState::Playing;
                        break;
                    case quantum::editor::SimulationPreview::PlaybackState::
                        Paused:
                        uiState = quantum::editor::
                            SimulationPlaybackState::Paused;
                        break;
                    }
                    editorUi.setSimulationStatus(
                        uiState,
                        simulationPreview.speedMetersPerSecond());
                };

                rebuildSimulationPreview();
                publishSimulationStatus();

                using PerformanceClock = std::chrono::steady_clock;
                std::optional<quantum::editor::PreviewSmokeCollector>
                    previewSmokeCollector;
                std::optional<PerformanceClock::time_point>
                    previewSmokeStart;
                bool previewSmokeDurationCompleted = false;
                bool previewSmokeFailure = false;
                std::string previewSmokeFailureMessage;
                if (previewSmokeOptions != nullptr)
                {
                    previewSmokeCollector.emplace(*previewSmokeOptions);
                    if (!simulationPreview.isAvailable())
                    {
                        previewSmokeFailure = true;
                        previewSmokeFailureMessage =
                            "Simulation preview unavailable: "
                            + simulationPreview.error();
                    }
                    else
                    {
                        simulationPreview.play();
                        previewSmokeStart = PerformanceClock::now();
                    }
                }

                const auto synchronizeDirtyState = [&]
                {
                    if (documentHistory.isDirty())
                    {
                        documentState.markDirty();
                    }
                    else
                    {
                        documentState.clearDirty();
                    }
                    editorUi.updateWindowTitle(documentState.windowTitle());
                };

                const auto publishHistoryState =
                    [&](const quantum::coaster::AuthoredTrack& restoredTrack)
                {
                    quantum::editor::CenterlineVisualization
                        restoredCenterline = quantum::editor::
                            createCenterlineVisualization(
                                restoredTrack,
                                restoredTrack.trackStyle());
                    quantum::coaster::RiderLoadHistory restoredRiderLoads =
                        quantum::editor::evaluateRiderLoadDiagnostics(
                            restoredTrack);
                    quantum::editor::AuthoredTrackEditTransaction
                        restoredTransaction{restoredTrack};
                    restoredTransaction.requireAcceptableRiderLoads(
                        restoredRiderLoads);

                    // Use the same generated geometry and renderer uploads as
                    // a forward authored edit before publishing the restored
                    // authoritative state.
                    vulkan.updateTrackCurveVertices(
                        restoredCenterline.vertices,
                        restoredCenterline.verticesPerCurve);
                    vulkan.updateRenderableTrack(
                        restoredCenterline.renderableTrack);

                    const std::size_t restoredSelection = std::min(
                        editorUi.selectedSection(),
                        restoredTrack.sectionCount() - 1);
                    authoredTrack = restoredTrack;
                    centerlineCache.setTrackStyle(
                        authoredTrack.trackStyle());
                    centerlineCache.replace(std::move(restoredCenterline));
                    editorUi.setCenterlineBounds(
                        centerline.minimumPosition,
                        centerline.maximumPosition);
                    editorUi.setCenterlineSections(centerline.sectionSlices);
                    editorUi.setRiderLoadHistory(
                        std::move(restoredRiderLoads));
editorUi.selectSection(restoredSelection, true);
                };

                // Shared save plumbing. All save paths serialize the
                // committed AuthoredTrack and present the same failure
                // message; callers decide the post-save title, log, and
                // loop behavior because those differ by workflow.
                const auto writeSerializedDocument =
                    [&](const std::filesystem::path& path) -> bool
                {
                    const std::string json =
                        quantum::coaster::serializeCoasterDocument(
                            authoredTrack);
                    std::ofstream ofs(path);

                    if (ofs.is_open()
                        && ofs.write(
                            json.data(),
                            static_cast<std::streamsize>(
                                json.size())))
                    {
                        return true;
                    }

                    SDL_ShowSimpleMessageBox(
                        SDL_MESSAGEBOX_ERROR,
                        "Save Failed",
                        "Could not write the document file.",
                        window
                    );
                    return false;
                };

                // Saves to the committed document path and marks the
                // history baseline so the document is no longer dirty.
                const auto saveToCurrentPath = [&]() -> bool
                {
                    if (!writeSerializedDocument(
                        documentState.currentPath()))
                    {
                        return false;
                    }

                    documentHistory.markSaved();
                    documentState.clearDirty();
                    return true;
                };

                // Prompts for a destination (defaulting the extension) and
                // saves there, adopting it as the current document path.
                const auto saveToChosenPath = [&]() -> bool
                {
                    auto savePath =
                        quantum::editor::saveFileDialog(window);

                    if (!savePath.has_value())
                    {
                        return false;
                    }

                    if (savePath->extension().empty())
                    {
                        *savePath += ".quantum";
                    }

                    if (!writeSerializedDocument(*savePath))
                    {
                        return false;
                    }

                    documentState.setOpenDocument(*savePath);
                    documentHistory.markSaved();
                    return true;
                };

                bool running = !previewSmokeFailure;
                std::optional<PerformanceClock::time_point>
                    previousRenderedFrameStart;
                std::uint64_t renderedFrameId = 0;

                while (running)
                {
                    const auto frameLoopStart = PerformanceClock::now();
                    const auto eventPumpBegin = frameLoopStart;
                    SDL_Event event{};

                    while (SDL_PollEvent(&event))
                    {
                        editorUi.processEvent(event);

                        if (event.type == SDL_EVENT_QUIT)
                        {
                            if (previewSmokeOptions != nullptr)
                            {
                                previewSmokeFailure = true;
                                previewSmokeFailureMessage =
                                    "Window closed before the requested smoke duration elapsed.";
                                running = false;
                                continue;
                            }
                            if (documentState.isDirty())
                            {
                                const SDL_MessageBoxButtonData buttons[] = {
                                    {SDL_MESSAGEBOX_BUTTON_RETURNKEY_DEFAULT,
                                        0, "Save"},
                                    {0, 1, "Don't Save"},
                                    {SDL_MESSAGEBOX_BUTTON_ESCAPEKEY_DEFAULT,
                                        2, "Cancel"}
                                };
                                const SDL_MessageBoxData messageBoxData{
                                    SDL_MESSAGEBOX_WARNING,
                                    window,
                                    "Unsaved Changes",
                                    "This document has unsaved changes.",
                                    3,
                                    buttons,
                                    nullptr
                                };
                                int buttonId = 2;
                                SDL_ShowMessageBox(
                                    &messageBoxData,
                                    &buttonId
                                );

                                if (buttonId == 0)
                                {
                                    const bool saved =
                                        documentState.hasPath()
                                        ? saveToCurrentPath()
                                        : saveToChosenPath();

                                    if (saved)
                                    {
                                        running = false;
                                    }
                                }
                                else if (buttonId == 1)
                                {
                                    running = false;
                                }
                                // buttonId == 2: Cancel, do nothing
                            }
                            else
                            {
                                running = false;
                            }
                        }
                    }
                    const auto eventPumpEnd = PerformanceClock::now();
                    const double eventPumpMilliseconds =
                        std::chrono::duration<double, std::milli>(
                            eventPumpEnd - eventPumpBegin).count();

                    if (running)
                    {
                        if ((SDL_GetWindowFlags(window)
                            & SDL_WINDOW_MINIMIZED) != 0)
                        {
                            SDL_Delay(10);
                            continue;
                        }

                        const auto renderedFrameStart =
                            PerformanceClock::now();
                        const double frameTimeMilliseconds =
                            previousRenderedFrameStart.has_value()
                            ? std::chrono::duration<double, std::milli>(
                                renderedFrameStart
                                    - *previousRenderedFrameStart).count()
                            : 0.0;
                        previousRenderedFrameStart = renderedFrameStart;
                        ++renderedFrameId;
                        simulationPreview.beginFrameTelemetry();
                        quantum::editor::FrameBlockingEvents
                            applicationBlockingEvents;

                        const auto pendingHistoryOperation =
                            editorUi.takePendingHistoryOperation();
                        if (pendingHistoryOperation.has_value())
                        {
                            using quantum::editor::HistoryOperationType;
                            const bool undoRequested =
                                *pendingHistoryOperation
                                    == HistoryOperationType::Undo;
                            std::optional<quantum::coaster::AuthoredTrack>
                                restoredTrack = undoRequested
                                    ? documentHistory.undo()
                                    : documentHistory.redo();
                            if (restoredTrack.has_value())
                            {
                                try
                                {
                                    publishHistoryState(*restoredTrack);
                                    applicationBlockingEvents
                                        .trackBufferMutation = true;
                                    synchronizeDirtyState();
                                    quantum::logging::logMessage(
                                        quantum::logging::LogLevel::Info,
                                        "EDIT",
                                        undoRequested
                                            ? "Undo restored the previous "
                                                "document revision"
                                            : "Redo restored the next "
                                                "document revision");
                                }
                                catch (const std::exception& exception)
                                {
                                    // The cursor advances before publication;
                                    // roll it back when generation or upload
                                    // cannot publish the requested revision.
                                    if (undoRequested)
                                    {
                                        static_cast<void>(
                                            documentHistory.redo());
                                    }
                                    else
                                    {
                                        static_cast<void>(
                                            documentHistory.undo());
                                    }
                                    SDL_ShowSimpleMessageBox(
                                        SDL_MESSAGEBOX_ERROR,
                                        undoRequested
                                            ? "Undo Failed" : "Redo Failed",
                                        exception.what(),
                                        window);
                                }
                            }
                        }

                        // Process file operations requested through the
                        // menu bar or command area.
                        const auto pendingFileOp =
                            editorUi.takePendingFileOperation();

                        if (pendingFileOp.has_value())
                        {
                            using quantum::editor::FileOperationType;
                            applicationBlockingEvents.modalOrFileDialog =
                                *pendingFileOp == FileOperationType::Open
                                || *pendingFileOp == FileOperationType::SaveAs
                                || (*pendingFileOp == FileOperationType::Save
                                    && !documentState.hasPath())
                                || (*pendingFileOp == FileOperationType::New
                                    && documentState.isDirty());
                        }

                        if (pendingFileOp.has_value())
                        {
                            using quantum::editor::FileOperationType;

                            auto confirmUnsaved = [&]() -> bool
                            {
                                if (!documentState.isDirty())
                                {
                                    return true;
                                }

                                const SDL_MessageBoxButtonData buttons[] = {
                                    {SDL_MESSAGEBOX_BUTTON_RETURNKEY_DEFAULT,
                                        0, "Save"},
                                    {0, 1, "Don't Save"},
                                    {SDL_MESSAGEBOX_BUTTON_ESCAPEKEY_DEFAULT,
                                        2, "Cancel"}
                                };
                                const SDL_MessageBoxData messageBoxData{
                                    SDL_MESSAGEBOX_WARNING,
                                    window,
                                    "Unsaved Changes",
                                    "This document has unsaved changes.",
                                    3,
                                    buttons,
                                    nullptr
                                };
                                int buttonId = 2;
                                SDL_ShowMessageBox(
                                    &messageBoxData,
                                    &buttonId
                                );

                                if (buttonId == 0)
                                {
                                    const bool saved =
                                        documentState.hasPath()
                                        ? saveToCurrentPath()
                                        : saveToChosenPath();

                                    if (saved)
                                    {
                                        editorUi.updateWindowTitle(
                                            documentState.windowTitle()
                                        );
                                    }
                                    return saved;
                                }

                                if (buttonId == 1)
                                {
                                    return true;
                                }

                                return false;
                            };

                            if (*pendingFileOp == FileOperationType::New)
                            {
                                if (!confirmUnsaved())
                                {
                                    editorUi.updateWindowTitle(
                                        documentState.windowTitle()
                                    );
                                }
                                else
                                {
                                    authoredTrack =
                                        quantum::coaster::createNewDocument();
                                    documentHistory.reset(authoredTrack);
                                    documentState.newDocument();
                                    editorUi.resetTransientState();
                                    editorUi.selectSection(0, true);

                                    centerlineCache.markDirty();
                                    centerlineCache.setTrackStyle(
                                        authoredTrack.trackStyle());
                                    static_cast<void>(
                                        centerlineCache.rebuildIfDirty(
                                            authoredTrack));
                                    editorUi.setCenterlineBounds(
                                        centerline.minimumPosition,
                                        centerline.maximumPosition
                                    );
                                    editorUi.setCenterlineSections(
                                        centerline.sectionSlices
                                    );
                                    editorUi.setRiderLoadHistory(
                                        quantum::editor::
                                            evaluateRiderLoadDiagnostics(
                                                authoredTrack)
                                    );
                                    applicationBlockingEvents
                                        .trackBufferMutation = true;
                                    vulkan.updateTrackCurveVertices(
                                        centerline.vertices,
                                        centerline.verticesPerCurve
                                    );
                                    vulkan.updateRenderableTrack(
                                        centerline.renderableTrack);
                                    editorUi.updateWindowTitle(
                                        documentState.windowTitle()
                                    );

                                    quantum::logging::logMessage(
                                        quantum::logging::LogLevel::Info,
                                        "FILE",
                                        "New document created"
                                    );
                                }
                            }
                            else if (
                                *pendingFileOp == FileOperationType::Open)
                            {
                                if (!confirmUnsaved())
                                {
                                    editorUi.updateWindowTitle(
                                        documentState.windowTitle()
                                    );
                                }
                                else
                                {
                                    auto openPath =
                                        quantum::editor::openFileDialog(
                                            window
                                        );

                                    if (openPath.has_value())
                                    {
                                        auto loaded = prepareDocument(*openPath);
                                        if (loaded.has_value())
                                        {
                                            applicationBlockingEvents
                                                .trackBufferMutation = true;
                                            vulkan.updateTrackCurveVertices(
                                                loaded->centerline.vertices,
                                                loaded->centerline.verticesPerCurve);
                                            vulkan.updateRenderableTrack(
                                                loaded->centerline.renderableTrack);
                                            authoredTrack =
                                                std::move(loaded->track);
                                            centerlineCache.setTrackStyle(
                                                authoredTrack.trackStyle());
                                            centerlineCache.replace(
                                                std::move(loaded->centerline));
                                            documentHistory.reset(authoredTrack);
                                            documentState.setOpenDocument(*openPath);
                                            editorUi.resetTransientState();
                                            editorUi.selectSection(0, true);
                                            editorUi.setCenterlineBounds(centerline.minimumPosition,
                                                centerline.maximumPosition);
                                            editorUi.setCenterlineSections(centerline.sectionSlices);
                                            editorUi.setRiderLoadHistory(
                                                std::move(loaded->riderLoads));
                                            editorUi.updateWindowTitle(
                                                documentState.windowTitle()
                                            );

                                            quantum::logging::logMessagef(
                                                quantum::logging::LogLevel::Info,
                                                "FILE",
                                                "Opened %s "
                                                "(%zu section(s))",
                                                openPath->string()
                                                    .c_str(),
                                                authoredTrack
                                                    .sectionCount()
                                            );
                                        }
                                        else
                                        {
                                            SDL_ShowSimpleMessageBox(
                                                SDL_MESSAGEBOX_ERROR,
                                                "Open Failed",
                                                loaded.error().c_str(),
                                                window
                                            );
                                        }
                                    }
                                }
                            }
                            else if (
                                *pendingFileOp == FileOperationType::Save)
                            {
                                const bool saved =
                                    documentState.hasPath()
                                    ? saveToCurrentPath()
                                    : saveToChosenPath();

                                if (saved)
                                {
                                    editorUi.updateWindowTitle(
                                        documentState.windowTitle()
                                    );

                                    quantum::logging::logMessagef(
                                        quantum::logging::LogLevel::Info,
                                        "FILE",
                                        "Saved %s",
                                        documentState.currentPath()
                                            .string()
                                            .c_str()
                                    );
                                }
                            }
                            else if (
                                *pendingFileOp
                                == FileOperationType::SaveAs)
                            {
                                if (saveToChosenPath())
                                {
                                    editorUi.updateWindowTitle(
                                        documentState.windowTitle()
                                    );

                                    quantum::logging::logMessagef(
                                        quantum::logging::LogLevel::Info,
                                        "FILE",
                                        "Saved As %s",
                                        documentState.currentPath()
                                            .string()
                                            .c_str()
                                    );
                                }
                            }
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
                        const auto requestedStartPoseEdit =
                            editorUi.takeStartPoseEdit();
                        const auto requestedHardwareEdit =
                            editorUi.takeTrackHardwareEdit();

                        // Continuous handle drags queue a changed-value or
                        // changed-boundary edit every motion frame; both
                        // success logs stay silent for them. The single
                        // end-of-drag [EDIT] summaries are emitted by
                        // EditorUi on release.
                        const bool continuousDrag =
                            (requestedValueEdit.has_value()
                                && requestedValueEdit->continuous)
                            || requestedDistanceEdit.has_value()
                            || (requestedStartPoseEdit.has_value()
                                && requestedStartPoseEdit->continuous)
                            || (requestedHardwareEdit.has_value()
                                && requestedHardwareEdit->continuous);
                        // A paused pointer can produce no changed value for
                        // one or more frames while the same drag is still
                        // held. Keep that gesture coalesced until the UI
                        // observes its release.
                        if (!continuousDrag
                            && !editorUi.documentDragActive())
                        {
                            documentHistory.endContinuousEdit();
                        }

                        quantum::editor::AuthoredTrackEditTransaction
                            editTransaction{authoredTrack};
                        quantum::coaster::AuthoredTrack& candidateTrack =
                            editTransaction.candidate();
                        if (requestedLengthEdit.has_value())
                        {
                            editTransaction
                                .requestSectionLengthBufferSync();
                        }
                        if (requestedValueEdit.has_value())
                        {
                            editTransaction.requestProfileValueBufferSync();
                        }
                        if (requestedRegionCommand.has_value())
                        {
                            editTransaction.requestRegionBufferSync();
                        }

                        bool candidateChanged = false;
                        bool trackStructureChanged = false;
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
                        bool hardwareEditApplied = false;
                        try
                        {
                            if (requestedHardwareEdit.has_value())
                            {
                                using quantum::editor::TrackHardwareEditType;
                                if (requestedHardwareEdit->type
                                    == TrackHardwareEditType::ReloadAsset)
                                {
                                    applicationBlockingEvents
                                        .hardwareAssetReload = true;
                                    vulkan.reloadTrackHardwareAsset(
                                        requestedHardwareEdit
                                            ->hardware.asset.path,
                                        centerline.renderableTrack);
                                }
                                else
                                {
                                    quantum::coaster::TrackStylePreset style =
                                        candidateTrack.trackStyle();
                                    if (style.repeatingHardware.empty())
                                    {
                                        style.repeatingHardware.push_back(
                                            requestedHardwareEdit->hardware);
                                    }
                                    else
                                    {
                                        style.repeatingHardware.front() =
                                            requestedHardwareEdit->hardware;
                                    }
                                    candidateTrack.setTrackStyle(style);
                                    candidateChanged = true;
                                    hardwareEditApplied = true;
                                }
                            }

                            if (requestedStartPoseEdit.has_value())
                            {
                                candidateTrack.setStartPose(
                                    requestedStartPoseEdit->pose
                                );
                                candidateChanged = true;
                            }

                            if (requestedCommand.has_value())
                            {
                                const quantum::editor::TrackCommand&
                                    command = *requestedCommand;

                                switch (command.type)
                                {
                                case quantum::editor::TrackCommandType::
                                    AppendSection:
                                    candidateTrack.appendSection();
                                    editTransaction.stageSelectionAfterCommit(
                                        candidateTrack.sectionCount() - 1
                                    );
                                    break;
                                case quantum::editor::TrackCommandType::
                                    PrependSection:
                                    candidateTrack.prependSection();
                                    editTransaction.stageSelectionAfterCommit(
                                        0);
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
                                    editTransaction.stageSelectionAfterCommit(
                                        quantum::editor::
                                            selectionAfterRemoval(
                                                editorUi.selectedSection(),
                                                command.sectionIndex,
                                                candidateTrack.sectionCount()
                                    ));
                                    break;
                                case quantum::editor::TrackCommandType::
                                    MoveSectionUp:
                                    candidateTrack.moveSection(
                                        command.sectionIndex,
                                        command.sectionIndex - 1
                                    );
                                    // Follow the moved region to its new
                                    // slot so selection keeps its identity.
                                    editTransaction.stageSelectionAfterCommit(
                                        quantum::editor::selectionAfterMove(
                                            editorUi.selectedSection(),
                                            command.sectionIndex,
                                            command.sectionIndex - 1
                                        ));
                                    break;
                                case quantum::editor::TrackCommandType::
                                    MoveSectionDown:
                                    candidateTrack.moveSection(
                                        command.sectionIndex,
                                        command.sectionIndex + 1
                                    );
                                    editTransaction.stageSelectionAfterCommit(
                                        quantum::editor::selectionAfterMove(
                                            editorUi.selectedSection(),
                                            command.sectionIndex,
                                            command.sectionIndex + 1
                                        ));
                                    break;
                                case quantum::editor::TrackCommandType::
                                    DuplicateSection:
                                    candidateTrack.duplicateSection(
                                        command.sectionIndex
                                    );
                                    // The duplicate occupies the slot right
                                    // after its origin.
                                    editTransaction.stageSelectionAfterCommit(
                                        command.sectionIndex + 1);
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
                                    editTransaction.stageSelectionAfterCommit(
                                        candidateTrack.sectionCount() - 1
                                    );
                                    break;
                                case RegionCommandType::
                                    PrependRateProfiles:
                                    candidateTrack.prependSection();
                                    editTransaction.stageSelectionAfterCommit(
                                        0);
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
                                    editTransaction.stageSelectionAfterCommit(
                                        candidateTrack.sectionCount() - 1
                                    );
                                    break;
                                case RegionCommandType::PrependPlanarArc:
                                    candidateTrack.prependSection();
                                    quantum::coaster::
                                        convertSectionToPlanarArc(
                                            candidateTrack.section(0)
                                        );
                                    editTransaction.stageSelectionAfterCommit(
                                        0);
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
                                    editTransaction.stageSelectionAfterCommit(
                                        command.sectionIndex + 1);
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
                                    editTransaction.stageSelectionAfterCommit(
                                        command.sectionIndex + 1);
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
                                quantum::logging::logMessage(
                                    quantum::logging::LogLevel::Debug,
                                    "EDIT",
                                    "Dropped same-frame profile edits that "
                                    "referred to pre-command section "
                                    "indices."
                                );
                            }

                            if (candidateChanged)
                            {
                                applicationBlockingEvents
                                    .trackBufferMutation = true;
                                quantum::editor::CenterlineVisualization
                                    candidateCenterline =
                                        quantum::editor::
                                            createCenterlineVisualization(
                                                candidateTrack,
                                                candidateTrack.trackStyle()
                                            );
                                quantum::coaster::RiderLoadHistory
                                    candidateRiderLoads =
                                        quantum::editor::
                                            evaluateRiderLoadDiagnostics(
                                                candidateTrack
                                            );

                                editTransaction.requireAcceptableRiderLoads(candidateRiderLoads);

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
                                vulkan.updateRenderableTrack(
                                    candidateCenterline.renderableTrack);

                                centerlineCache.setTrackStyle(
                                    candidateTrack.trackStyle());
                                centerlineCache.replace(
                                    std::move(candidateCenterline));
                                editTransaction.commit(authoredTrack);
                                documentHistory.record(
                                    authoredTrack,
                                    continuousDrag);
                                editorUi.setRiderLoadHistory(
                                    std::move(candidateRiderLoads)
                                );
                                synchronizeDirtyState();

                                if (hardwareEditApplied && !continuousDrag)
                                {
                                    const auto& hardware = authoredTrack
                                        .trackStyle().repeatingHardware.front();
                                    quantum::logging::logMessagef(
                                        quantum::logging::LogLevel::Info,
                                        "EDIT",
                                        "Track hardware updated: asset=%s "
                                        "spacing=%.6f offset=%.6f",
                                        hardware.asset.path.c_str(),
                                        hardware.spacing,
                                        hardware.startOffset);
                                }

                                if (!continuousDrag)
                                {
                                    quantum::logging::logMessagef(
                                        quantum::logging::LogLevel::Info,
                                        "EDIT",
                                        "Authored edit accepted; track now "
                                        "has %zu section(s), %zu centerline "
                                        "samples.",
                                        authoredTrack.sectionCount(),
                                        centerline.samples.size()
                                    );
                                }

                                if (valueEditApplied
                                    && requestedValueEdit
                                    && !continuousDrag)
                                {
                                    quantum::logging::logMessagef(
                                        quantum::logging::LogLevel::Info,
                                        "EDIT",
                                        "section=%zu channel=%d "
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
                                        quantum::logging::logMessagef(
                                            quantum::logging::LogLevel::Info,
                                            "EDIT",
                                            "section=%zu channel=%d "
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
                                        quantum::logging::logMessagef(
                                            quantum::logging::LogLevel::Info,
                                            "EDIT",
                                            "section=%zu channel=%d "
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
                                            quantum::logging::logMessagef(
                                                quantum::logging::LogLevel::Info,
                                                "EDIT",
                                                "%s region=%zu "
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
                                            quantum::logging::logMessagef(
                                                quantum::logging::LogLevel::Info,
                                                "EDIT",
                                                "%s region=%zu "
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
                                        quantum::logging::logMessagef(
                                            quantum::logging::LogLevel::Info,
                                            "EDIT",
                                            "section=%zu "
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
                                            quantum::logging::logMessagef(
                                                quantum::logging::LogLevel::Info,
                                                "EDIT",
                                                "section=%zu "
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
                                            quantum::logging::logMessagef(
                                                quantum::logging::LogLevel::Info,
                                                "EDIT",
                                                "section=%zu "
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

                            if (requestedStartPoseEdit.has_value())
                            {
                                editorUi.rejectStartPoseManipulation();
                                quantum::logging::logMessagef(
                                    quantum::logging::LogLevel::Debug,
                                    "EDIT",
                                    "start-pose candidate rejected: %s",
                                    exception.what()
                                );
                            }

                            quantum::logging::logMessagef(
                                quantum::logging::LogLevel::Warning,
                                "EDIT",
                                "Authored edit was rejected: %s",
                                exception.what()
                            );
                        }

                        // A staged structural selection becomes visible only
                        // after the candidate document has committed. On
                        // rejection selectionAfterCommit() stays empty.
                        if (const auto committedSelection =
                                editTransaction.selectionAfterCommit())
                        {
                            // Removing the selected region can keep the same
                            // numeric index while changing its identity.
                            editorUi.selectSection(
                                *committedSelection,
                                true);
                        }

                        // Numeric fields are editor-side buffers. Refresh
                        // every buffer for which the user submitted an intent
                        // from the final committed document, regardless of
                        // whether mutation, solve, or upload rejected it.
                        const std::size_t selectedSection =
                            editorUi.selectedSection();
                        if (selectedSection < authoredTrack.sectionCount()
                            && (editTransaction
                                    .sectionLengthBufferSyncRequested()
                                || editTransaction
                                    .regionBufferSyncRequested()))
                        {
                            editorUi.synchronizeSectionLength(
                                quantum::coaster::sectionLength(
                                    authoredTrack.section(selectedSection))
                            );
                        }

                        if (editTransaction.profileValueBufferSyncRequested()
                            && requestedValueEdit
                            && (!editTransaction.committed()
                                || !trackStructureChanged)
                            && requestedValueEdit->sectionIndex
                                < authoredTrack.sectionCount())
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
                            const quantum::editor::ScalarProfileEndpoint
                                endpoint = requestedValueEdit->endpoint;
                            double committedValue = 0.0;
                            if (committedSegment != nullptr)
                            {
                                committedValue = endpoint
                                        == quantum::editor::
                                            ScalarProfileEndpoint::Begin
                                    ? committedSegment->valueBegin
                                    : committedSegment->valueEnd;
                            }
                            editorUi.synchronizeSegmentEndpointValue(
                                requestedValueEdit->channel,
                                requestedValueEdit->segmentId,
                                endpoint,
                                committedValue
                            );
                        }

                        if (editTransaction.regionBufferSyncRequested()
                            && selectedSection < authoredTrack.sectionCount())
                        {
                            const auto& committedSection =
                                authoredTrack.section(selectedSection);

                            if (committedSection.kind ==
                                quantum::coaster::RegionKind::Geometry
                                && !quantum::coaster::isForceDrivenSection(committedSection))
                            {
                                editorUi.synchronizePlanarArcParams(
                                    std::get<quantum::coaster::
                                        PlanarArcRegion>(
                                        std::get<quantum::coaster::
                                            GeometryRegion>(
                                            committedSection.region)
                                        .construction));
                            }
                        }

                        // Layout mode is metadata-only; no geometry
                        // recalculation needed.
                        const auto requestedLayoutMode =
                            editorUi.takePendingLayoutModeChange();

                        if (requestedLayoutMode.has_value()
                            && *requestedLayoutMode
                                != authoredTrack.layoutMode())
                        {
                            authoredTrack.setLayoutMode(
                                *requestedLayoutMode);
                            documentHistory.record(authoredTrack);
                            synchronizeDirtyState();

                            quantum::logging::logMessagef(
                                quantum::logging::LogLevel::Info,
                                "CFG",
                                "Layout mode set to %s",
                                quantum::coaster::layoutModeToString(
                                    authoredTrack.layoutMode()));
                        }

                        // Setup edits are document configuration. Only a
                        // heartline edit regenerates viewport reference-curve
                        // vertices; it does not change authored geometry,
                        // track mesh geometry, or simulation physics.
                        const auto requestedCoasterSetup =
                            editorUi.takePendingCoasterSetupEdit();

                        if (requestedCoasterSetup.has_value()
                            && *requestedCoasterSetup
                                != authoredTrack.coasterSetup())
                        {
                            try
                            {
                                quantum::coaster::AuthoredTrack candidateTrack =
                                    authoredTrack;
                                candidateTrack.setCoasterSetup(
                                    *requestedCoasterSetup);

                                std::optional<quantum::editor::
                                    CenterlineVisualization>
                                    candidateCenterline;
                                if (requestedCoasterSetup->heartline
                                    != authoredTrack.coasterSetup().heartline)
                                {
                                    candidateCenterline = quantum::editor::
                                        createCenterlineVisualization(
                                            candidateTrack,
                                            candidateTrack.trackStyle());
                                    vulkan.updateTrackCurveVertices(
                                        candidateCenterline->vertices,
                                        candidateCenterline->verticesPerCurve);
                                    applicationBlockingEvents
                                        .trackBufferMutation = true;
                                }

                                authoredTrack = std::move(candidateTrack);
                                if (candidateCenterline.has_value())
                                {
                                    centerlineCache.replace(
                                        std::move(*candidateCenterline));
                                }
                                documentHistory.record(authoredTrack);
                                synchronizeDirtyState();
                                quantum::logging::logMessagef(
                                    quantum::logging::LogLevel::Info,
                                    "CFG",
                                    "Coaster setup applied (style %s, "
                                    "%u cars)",
                                    authoredTrack.coasterSetup()
                                        .styleId.c_str(),
                                    static_cast<unsigned>(
                                        authoredTrack.coasterSetup()
                                            .carsPerTrain));
                            }
                            catch (const std::invalid_argument& error)
                            {
                                quantum::logging::logMessagef(
                                    quantum::logging::LogLevel::Error,
                                    "CFG",
                                    "Coaster setup rejected: %s",
                                    error.what());
                            }
                        }

                        // Circuit completion: run solver and show
                        // result.
                        if (editorUi.takeCircuitCompletionRequest())
                        {
                            applicationBlockingEvents.modalOrFileDialog = true;
                            const quantum::coaster::
                                CircuitCompletionResult result =
                                    quantum::coaster::
                                        completeCircuitCandidate(
                                            authoredTrack);

                            char message[512]{};

                            if (result.success)
                            {
                                std::snprintf(
                                    message,
                                    sizeof(message),
                                    "Circuit Completed\n\n"
                                    "Connector Regions: %zu\n"
                                    "Final gap: %.4f m\n"
                                    "Tangent error: %.2f deg\n"
                                    "Frame error: %.2f deg",
                                    result.connectorRegionCount,
                                    result.finalPositionalGap,
                                    result.finalTangentErrorDegrees,
                                    result.finalFrameErrorDegrees);

                                // Commit: replace document with the
                                // completed track.
                                authoredTrack =
                                    std::move(
                                        result.completedTrack);

                                quantum::editor::
                                    CenterlineVisualization
                                        newCenterline =
                                            quantum::editor::
                                                createCenterlineVisualization(
                                                    authoredTrack,
                                                    authoredTrack.trackStyle());

                                editorUi.setCenterlineBounds(
                                    newCenterline.minimumPosition,
                                    newCenterline.maximumPosition);
                                editorUi.setCenterlineSections(
                                    newCenterline.sectionSlices);
                                applicationBlockingEvents
                                    .trackBufferMutation = true;
                                vulkan.updateTrackCurveVertices(
                                    newCenterline.vertices,
                                    newCenterline.verticesPerCurve);
                                vulkan.updateRenderableTrack(
                                    newCenterline.renderableTrack);

                                centerlineCache.setTrackStyle(
                                    authoredTrack.trackStyle());
                                centerlineCache.replace(
                                    std::move(newCenterline));
                                editorUi.setRiderLoadHistory(
                                    quantum::editor::
                                        evaluateRiderLoadDiagnostics(
                                            authoredTrack)
                                );
                                documentHistory.record(authoredTrack);
                                synchronizeDirtyState();

                                // Select the newly created connector.
                                editorUi.selectSection(
                                    authoredTrack.sectionCount() - 1);

                                quantum::logging::logMessagef(
                                    quantum::logging::LogLevel::Info,
                                    "EDIT",
                                    "Circuit completed: gap=%.4f m "
                                    "tang=%.2f deg frame=%.2f deg "
                                    "iter=%u",
                                    result.finalPositionalGap,
                                    result.finalTangentErrorDegrees,
                                    result.finalFrameErrorDegrees,
                                    result.iterationCount);
                            }
                            else
                            {
                                std::snprintf(
                                    message,
                                    sizeof(message),
                                    "Circuit Completion Failed\n\n"
                                    "Reason: %s",
                                    result.failureMessage
                                        .c_str());
                            }

                            SDL_ShowSimpleMessageBox(
                                result.success
                                    ? SDL_MESSAGEBOX_INFORMATION
                                    : SDL_MESSAGEBOX_WARNING,
                                result.success
                                    ? "Circuit Completed"
                                    : "Circuit Completion Failed",
                                message,
                                window);
                        }

                        // Any accepted geometry/document replacement advances
                        // the cache generation. Layout mode is also tracked
                        // because it changes open/circuit physics semantics
                        // without regenerating visible geometry.
                        if (simulationTrackGeneration
                                != centerlineCache.generation()
                            || simulationLayoutMode
                                != authoredTrack.layoutMode())
                        {
                            simulationTrackGeneration =
                                centerlineCache.generation();
                            simulationLayoutMode =
                                authoredTrack.layoutMode();
                            rebuildSimulationPreview();
                        }

                        publishSimulationStatus();
                        editorUi.setHistoryAvailability(
                            documentHistory.canUndo(),
                            documentHistory.canRedo());
                        editorUi.beginFrame(vulkan);
                        quantum::editor::FrameBlockingEvents
                            frameBlockingEvents =
                                editorUi.takeFrameBlockingEvents();
                        frameBlockingEvents.trackBufferMutation =
                            applicationBlockingEvents.trackBufferMutation;
                        frameBlockingEvents.hardwareAssetReload =
                            applicationBlockingEvents.hardwareAssetReload;
                        frameBlockingEvents.modalOrFileDialog =
                            applicationBlockingEvents.modalOrFileDialog;

                        if (const auto control =
                                editorUi.takeSimulationControl())
                        {
                            using quantum::editor::SimulationControlType;
                            switch (*control)
                            {
                            case SimulationControlType::Play:
                                simulationPreview.play();
                                break;
                            case SimulationControlType::Pause:
                                simulationPreview.pause();
                                break;
                            case SimulationControlType::Reset:
                                simulationPreview.reset();
                                break;
                            }
                        }

                        const auto simulationUpdateBegin =
                            PerformanceClock::now();
                        const double preSimulationCpuMilliseconds =
                            std::chrono::duration<double, std::milli>(
                                simulationUpdateBegin - eventPumpEnd).count();
                        const double frameStartToSimulationMilliseconds =
                            std::chrono::duration<double, std::milli>(
                                simulationUpdateBegin - frameLoopStart).count();
                        // Minimized iterations skip ImGui NewFrame, so its
                        // first restored delta includes the entire suspension.
                        // The event flags survive those skipped iterations.
                        simulationPreview.update(editorUi.frameDeltaSeconds(),
                            frameBlockingEvents.windowMinimized
                                || frameBlockingEvents.windowRestored);
                        publishSimulationStatus();

                        double previewVertexPublishMilliseconds = 0.0;
                        if (uploadedSimulationVertexGeneration
                            != simulationPreview.vertexGeneration())
                        {
                            const auto previewPublishBegin =
                                PerformanceClock::now();
                            vulkan.updateTrainPreviewVertices(
                                simulationPreview.vertices());
                            previewVertexPublishMilliseconds =
                                std::chrono::duration<double, std::milli>(
                                    PerformanceClock::now()
                                        - previewPublishBegin).count();
                            uploadedSimulationVertexGeneration =
                                simulationPreview.vertexGeneration();
                        }

                        vulkan.drawFrame(
                    [](VkCommandBuffer commandBuffer, void* userData)
                    {
                        static_cast<quantum::editor::EditorUi*>(
                            userData
                        )->render(commandBuffer);
                    },
                    &editorUi
                        );

                        const quantum::editor::
                            SimulationPreviewFrameTelemetry& preview =
                                simulationPreview.frameTelemetry();
                        const quantum::renderer::DrawFrameCpuTelemetry& draw =
                            vulkan.lastDrawFrameCpuTelemetry();
                        const quantum::editor::FramePerformanceSample
                            performanceSample{
                                .frameId = renderedFrameId,
                                .rawSimulationDeltaMilliseconds =
                                    preview.rawDeltaMilliseconds,
                                .accumulatorBeforeMilliseconds =
                                    preview.accumulatorBeforeMilliseconds,
                                .accumulatorAfterIncomingMilliseconds =
                                    preview.accumulatorAfterIncomingMilliseconds,
                                .accumulatorRemainingMilliseconds =
                                    preview.accumulatorRemainingMilliseconds,
                                .discardedWallTimeMilliseconds =
                                    preview.discardedWallTimeMilliseconds,
                                .requestedPhysicsStepCount =
                                    preview.requestedStepCount,
                                .frameTimeMilliseconds =
                                    frameTimeMilliseconds,
                                .fixedPhysicsStepCount =
                                    preview.fixedStepCount,
                                .maximumPhysicsStepsHit =
                                    preview.maximumStepsHit,
                                .minimumPhysicsStepMilliseconds =
                                    preview.minimumStepMilliseconds,
                                .averagePhysicsStepMilliseconds =
                                    preview.averageStepMilliseconds,
                                .maximumPhysicsStepMilliseconds =
                                    preview.maximumStepMilliseconds,
                                .physicsMilliseconds =
                                    preview.physicsMilliseconds,
                                .consecutiveCatchUpFrameCount =
                                    preview.consecutiveCatchUpFrameCount,
                                .eventPumpMilliseconds =
                                    eventPumpMilliseconds,
                                .preSimulationCpuMilliseconds =
                                    preSimulationCpuMilliseconds,
                                .frameStartToSimulationMilliseconds =
                                    frameStartToSimulationMilliseconds,
                                .interpolationMilliseconds =
                                    preview.interpolationMilliseconds,
                                .renderPoseSolveMilliseconds =
                                    preview.renderPoseSolveMilliseconds,
                                .renderPoseSolveCount = preview.renderPoseSolveCount,
                                .renderPoseFailureCount = preview.renderPoseFailureCount,
                                .previewVertexPreparationMilliseconds =
                                    preview.vertexPreparationMilliseconds,
                                .previewVertexPublishMilliseconds =
                                    previewVertexPublishMilliseconds,
                                .previewFrameSlotUpdateMilliseconds =
                                    draw.previewFrameSlotUpdateMilliseconds,
                                .previewFrameSlotWaitMilliseconds =
                                    draw.frameSlotWaitMilliseconds,
                                .drawFrameCpuMilliseconds =
                                    draw.totalMilliseconds,
                                .acquireCallMilliseconds =
                                    draw.acquireCallMilliseconds,
                                .presentCallMilliseconds =
                                    draw.presentCallMilliseconds,
                                .previewStreamUpdated =
                                    draw.previewStreamUpdated,
                                .swapchainRecreated =
                                    draw.swapchainRecreated,
                                .synchronousReadback =
                                    draw.synchronousReadback,
                                .blockingEvents = frameBlockingEvents,
                                .synchronization = draw.synchronization
                        };
                        editorUi.recordFramePerformance(performanceSample);
                        if (previewSmokeCollector.has_value())
                        {
                            previewSmokeCollector->record(performanceSample);
                            if (!simulationPreview.isAvailable())
                            {
                                previewSmokeFailure = true;
                                previewSmokeFailureMessage =
                                    simulationPreview.error();
                                simulationPreview.pause();
                                running = false;
                            }
                            else if (preview.renderPoseFailureCount > 0)
                            {
                                previewSmokeFailure = true;
                                previewSmokeFailureMessage =
                                    "Simulation preview interpolation failed.";
                                simulationPreview.pause();
                                running = false;
                            }
                            else if (simulationPreview.playbackState()
                                != quantum::editor::SimulationPreview::
                                    PlaybackState::Playing
                                && !(previewSmokeOptions->repeat
                                    && preview.boundaryStopped))
                            {
                                previewSmokeFailure = true;
                                previewSmokeFailureMessage =
                                    "Simulation playback stopped before the "
                                    "requested duration elapsed.";
                                running = false;
                            }
                            else
                            {
                                const double elapsedSeconds =
                                    std::chrono::duration<double>(
                                        PerformanceClock::now()
                                        - *previewSmokeStart).count();
                                if (previewSmokeOptions->repeat
                                    && preview.boundaryStopped)
                                {
                                    simulationPreview.reset();
                                    simulationPreview.play();
                                }
                                if (elapsedSeconds
                                    >= previewSmokeOptions->durationSeconds)
                                {
                                    previewSmokeDurationCompleted = true;
                                    simulationPreview.pause();
                                    running = false;
                                }
                            }
                        }
                    }
                }

                if (previewSmokeCollector.has_value())
                {
                    const double elapsedSeconds = previewSmokeStart
                        ? std::chrono::duration<double>(
                            PerformanceClock::now()
                            - *previewSmokeStart).count()
                        : 0.0;
                    const auto report = previewSmokeCollector->finish(
                        elapsedSeconds,
                        previewSmokeDurationCompleted && !previewSmokeFailure,
                        previewSmokeFailure,
                        previewSmokeFailureMessage);
                    const auto paths =
                        quantum::editor::writePreviewSmokeReports(
                            report, *previewSmokeOptions);
                    quantum::logging::logMessagef(
                        quantum::logging::LogLevel::Info,
                        "SMOKE",
                        "Preview smoke reports written to %s and %s",
                        paths.json.string().c_str(),
                        paths.text.string().c_str());
                    applicationExitCode =
                        report.playbackCompletedNormally
                            && !report.previewOrPhysicsFailure ? 0 : 2;
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

        return applicationExitCode;
    }
}
