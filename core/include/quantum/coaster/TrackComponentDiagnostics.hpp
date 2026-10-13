#pragma once
#include <functional>
#include <string_view>

namespace quantum::coaster
{
    // Presentation diagnostics are separate from authored data and asset I/O.
    // An empty detail clears a previous slot-contract fallback after recovery.
    using TrackComponentDiagnosticSink = std::function<void(std::string_view, std::string_view)>;
    void setTrackComponentDiagnosticSink(TrackComponentDiagnosticSink sink);
}
