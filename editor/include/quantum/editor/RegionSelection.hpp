#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace quantum::editor
{
    // Index transforms used after accepted document mutations. They preserve
    // the identity of an unaffected selected region while reflecting the
    // AuthoredTrack vector ordering rules.
    [[nodiscard]] inline std::size_t selectionAfterInsertion(
        const std::size_t selectedIndex,
        const std::size_t insertedIndex) noexcept
    {
        return insertedIndex <= selectedIndex
            ? selectedIndex + 1
            : selectedIndex;
    }

    [[nodiscard]] inline std::size_t selectionAfterRemoval(
        const std::size_t selectedIndex,
        const std::size_t removedIndex,
        const std::size_t remainingCount)
    {
        if (remainingCount == 0)
        {
            throw std::invalid_argument(
                "Region selection requires at least one remaining region."
            );
        }

        if (removedIndex < selectedIndex)
        {
            return selectedIndex - 1;
        }
        if (removedIndex == selectedIndex)
        {
            return std::min(removedIndex, remainingCount - 1);
        }
        return selectedIndex;
    }

    [[nodiscard]] inline std::size_t selectionAfterMove(
        const std::size_t selectedIndex,
        const std::size_t fromIndex,
        const std::size_t toIndex) noexcept
    {
        if (selectedIndex == fromIndex)
        {
            return toIndex;
        }
        if (fromIndex < toIndex
            && selectedIndex > fromIndex
            && selectedIndex <= toIndex)
        {
            return selectedIndex - 1;
        }
        if (toIndex < fromIndex
            && selectedIndex >= toIndex
            && selectedIndex < fromIndex)
        {
            return selectedIndex + 1;
        }
        return selectedIndex;
    }
}
