#include <filesystem>
#include <string>
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace quantum::editor
{
    bool replaceDocumentFile(const std::filesystem::path& source,
        const std::filesystem::path& destination, std::string& error)
    {
        if (MoveFileExW(source.c_str(), destination.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) return true;
        error = "Could not replace document (Windows error " + std::to_string(GetLastError()) + ").";
        return false;
    }
}
