#pragma once

#include <quantum/coaster/AuthoredTrack.hpp>

#include <expected>
#include <string>

namespace quantum::coaster
{
    // Current document format version. The serialization and
    // deserialization functions enforce this exact version.
    inline constexpr int currentFormatVersion = 1;

    // Serializes an AuthoredTrack into a deterministic, human-readable
    // JSON string at the current document format version. The output is
    // byte-identical across calls for the same authored content.
    [[nodiscard]] std::string serializeCoasterDocument(
        const AuthoredTrack& track
    );

    // Deserializes a JSON string into an AuthoredTrack. The returned
    // track is fully validated against Core section invariants.
    //
    // Returns std::unexpected with a human-readable error when:
    //   - the JSON is malformed,
    //   - the format version is missing or unsupported,
    //   - required fields are missing or have wrong types,
    //   - unknown fields or enum values are present,
    //   - authored data violates Core validation constraints, or
    //   - nextSegmentId is inconsistent with segment ids.
    //
    // Never mutates an existing AuthoredTrack; the caller receives a
    // complete new track only on success.
    [[nodiscard]] std::expected<AuthoredTrack, std::string>
    deserializeCoasterDocument(const std::string& jsonString);
}
