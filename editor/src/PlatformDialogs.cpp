#include <quantum/editor/PlatformDialogs.hpp>

#ifdef _WIN32

#include <objbase.h>
#include <shobjidl.h>
#include <windows.h>

#include <SDL3/SDL.h>

namespace
{
    [[nodiscard]] std::optional<std::filesystem::path> showFileDialog(
        const bool isOpen)
    {
        HRESULT hr = CoInitializeEx(
            nullptr,
            COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE
        );

        if (FAILED(hr))
        {
            SDL_LogError(
                SDL_LOG_CATEGORY_APPLICATION,
                "COM initialization failed for file dialog: 0x%08lx",
                static_cast<unsigned long>(hr)
            );
            return std::nullopt;
        }

        IFileDialog* dialog = nullptr;
        const CLSID clsid = isOpen ? CLSID_FileOpenDialog : CLSID_FileSaveDialog;
        hr = CoCreateInstance(
            clsid,
            nullptr,
            CLSCTX_ALL,
            IID_PPV_ARGS(&dialog)
        );

        if (FAILED(hr) || dialog == nullptr)
        {
            SDL_LogError(
                SDL_LOG_CATEGORY_APPLICATION,
                "Failed to create file dialog: 0x%08lx",
                static_cast<unsigned long>(hr)
            );
            CoUninitialize();
            return std::nullopt;
        }

        const COMDLG_FILTERSPEC filter{
            L"QUANTUM Coaster Files",
            L"*.quantum"
        };
        dialog->SetFileTypes(1, &filter);
        dialog->SetDefaultExtension(L"quantum");

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
            SDL_LogError(
                SDL_LOG_CATEGORY_APPLICATION,
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
        return showFileDialog(true);
    }

    std::optional<std::filesystem::path> saveFileDialog(SDL_Window* window)
    {
        (void)window;
        return showFileDialog(false);
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
}

#endif
