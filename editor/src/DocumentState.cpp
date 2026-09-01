#include <quantum/editor/DocumentState.hpp>

namespace quantum::editor
{
    void DocumentState::newDocument()
    {
        currentPath_.clear();
        dirty_ = false;
    }

    void DocumentState::setOpenDocument(const std::filesystem::path& path)
    {
        currentPath_ = path;
        dirty_ = false;
    }

    void DocumentState::setCurrentPath(const std::filesystem::path& path)
    {
        currentPath_ = path;
    }

    void DocumentState::clearPath()
    {
        currentPath_.clear();
    }

    void DocumentState::markDirty()
    {
        dirty_ = true;
    }

    void DocumentState::clearDirty()
    {
        dirty_ = false;
    }

    bool DocumentState::isDirty() const noexcept
    {
        return dirty_;
    }

    const std::filesystem::path& DocumentState::currentPath() const noexcept
    {
        return currentPath_;
    }

    bool DocumentState::hasPath() const noexcept
    {
        return !currentPath_.empty();
    }

    std::string DocumentState::displayName() const
    {
        if (currentPath_.empty())
        {
            return "Untitled";
        }

        return currentPath_.filename().string();
    }

    std::string DocumentState::windowTitle() const
    {
        std::string title = "QUANTUM \xe2\x80\x94 ";
        title += displayName();

        if (dirty_)
        {
            title += " *";
        }

        return title;
    }
}
