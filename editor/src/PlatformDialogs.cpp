#include <quantum/editor/PlatformDialogs.hpp>
#include <quantum/coaster/TrackStyle.hpp>
#include <quantum/engine/Logging.hpp>

#include <system_error>

#ifdef _WIN32

#include <objbase.h>
#include <shobjidl.h>
#include <windows.h>

#include <SDL3/SDL.h>

namespace
{
    enum class FileDialogKind
    {
        OpenDocument,
        SaveDocument,
        OpenTrackHardware
    };

    [[nodiscard]] std::optional<std::filesystem::path> showFileDialog(
        const FileDialogKind kind)
    {
        HRESULT hr = CoInitializeEx(
            nullptr,
            COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE
        );

        if (FAILED(hr))
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Error,
                "FILE",
                "COM initialization failed for file dialog: 0x%08lx",
                static_cast<unsigned long>(hr)
            );
            return std::nullopt;
        }

        IFileDialog* dialog = nullptr;
        const bool isOpen = kind != FileDialogKind::SaveDocument;
        const CLSID clsid = isOpen ? CLSID_FileOpenDialog : CLSID_FileSaveDialog;
        hr = CoCreateInstance(
            clsid,
            nullptr,
            CLSCTX_ALL,
            IID_PPV_ARGS(&dialog)
        );

        if (FAILED(hr) || dialog == nullptr)
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Error,
                "FILE",
                "Failed to create file dialog: 0x%08lx",
                static_cast<unsigned long>(hr)
            );
            CoUninitialize();
            return std::nullopt;
        }

        const COMDLG_FILTERSPEC filter = kind
            == FileDialogKind::OpenTrackHardware
            ? COMDLG_FILTERSPEC{L"glTF Binary Mesh", L"*.glb"}
            : COMDLG_FILTERSPEC{L"QUANTUM Coaster Files", L"*.quantum"};
        dialog->SetFileTypes(1, &filter);
        dialog->SetDefaultExtension(kind == FileDialogKind::OpenTrackHardware
            ? L"glb" : L"quantum");

        if (!isOpen)
        {
            dialog->SetOptions(
                FOS_OVERWRITEPROMPT | FOS_STRICTFILETYPES
            );
        }

        hr = dialog->Show(nullptr);

        if (hr == HRESULT_FROM_WIN32(ERROR_CANCELLED))
        {
            dialog->Release();
            CoUninitialize();
            return std::nullopt;
        }

        if (FAILED(hr))
        {
            quantum::logging::logMessagef(
                quantum::logging::LogLevel::Error,
                "FILE",
                "File dialog failed: 0x%08lx",
                static_cast<unsigned long>(hr)
            );
            dialog->Release();
            CoUninitialize();
            return std::nullopt;
        }

        IShellItem* item = nullptr;
        hr = dialog->GetResult(&item);

        if (FAILED(hr) || item == nullptr)
        {
            dialog->Release();
            CoUninitialize();
            return std::nullopt;
        }

        PWSTR filePath = nullptr;
        hr = item->GetDisplayName(SIGDN_FILESYSPATH, &filePath);

        std::optional<std::filesystem::path> result;

        if (SUCCEEDED(hr) && filePath != nullptr)
        {
            result = std::filesystem::path(filePath);
            CoTaskMemFree(filePath);
        }

        item->Release();
        dialog->Release();
        CoUninitialize();

        return result;
    }
}

namespace quantum::editor
{
    std::optional<std::filesystem::path> openFileDialog(SDL_Window* window)
    {
        (void)window;
        return showFileDialog(FileDialogKind::OpenDocument);
    }

    std::optional<std::filesystem::path> saveFileDialog(SDL_Window* window)
    {
        (void)window;
        return showFileDialog(FileDialogKind::SaveDocument);
    }

    std::optional<std::filesystem::path>
    openTrackHardwareFileDialog(SDL_Window* window)
    {
        (void)window;
        return showFileDialog(FileDialogKind::OpenTrackHardware);
    }
}

#else

namespace quantum::editor
{
    std::optional<std::filesystem::path> openFileDialog(SDL_Window* window)
    {
        (void)window;
        return std::nullopt;
    }

    std::optional<std::filesystem::path> saveFileDialog(SDL_Window* window)
    {
        (void)window;
        return std::nullopt;
    }

    std::optional<std::filesystem::path>
    openTrackHardwareFileDialog(SDL_Window* window)
    {
        (void)window;
        return std::nullopt;
    }
}

#endif

namespace quantum::editor
{
    std::expected<std::string, std::string> trackHardwareAssetIdFromPath(
        const std::filesystem::path& selectedPath,
        const std::filesystem::path& runtimeRoot)
    {
        std::error_code error;
        const std::filesystem::path assetRoot =
            std::filesystem::weakly_canonical(runtimeRoot / "assets", error);
        if (error)
            return std::unexpected("Could not resolve the runtime asset root.");
        const std::filesystem::path selected =
            std::filesystem::weakly_canonical(selectedPath, error);
        if (error)
            return std::unexpected("Could not resolve the selected GLB path.");

        const std::filesystem::path relative =
            selected.lexically_relative(assetRoot);
        if (relative.empty() || relative.is_absolute())
            return std::unexpected("Select a GLB below the runtime assets folder.");
        for (const std::filesystem::path& part : relative)
        {
            if (part == "..")
                return std::unexpected(
                    "Select a GLB below the runtime assets folder.");
        }

        try
        {
            return coaster::normalizeTrackHardwareAssetIdentifier(
                "assets://" + relative.generic_string());
        }
        catch (const std::exception& exception)
        {
            return std::unexpected(exception.what());
        }
    }
}
