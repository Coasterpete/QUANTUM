#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace quantum::audio
{
// Owns QUANTUM's SDL audio-subsystem reference. Audio M0 only initializes
// the subsystem, captures basic driver/playback-device information, and
// balances the SDL_INIT_AUDIO reference it takes. No playback device, stream,
// mixer, or playback path exists yet.
//
// Ownership: initialize() takes one SDL_INIT_AUDIO subsystem reference;
// shutdown() (and therefore the destructor) releases exactly that reference.
// It never calls SDL_Quit(). Re-initializing shuts down first so the SDL
// subsystem reference count is never incremented twice.
class AudioEngine
{
public:
    AudioEngine() noexcept = default;
    ~AudioEngine();

    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    // Initializes the SDL audio subsystem and snapshots basic audio
    // information. Audio failure is non-fatal: on failure the engine is left
    // uninitialized with lastError() populated and initialize() returns false.
    [[nodiscard]] bool initialize() noexcept;

    // Idempotent. Safe to call on an uninitialized engine. Releasing the SDL
    // audio-subsystem reference happens before the application's SDL_Quit().
    void shutdown() noexcept;

    [[nodiscard]] bool isInitialized() const noexcept;

    [[nodiscard]] std::string_view lastError() const noexcept;
    [[nodiscard]] std::string_view audioDriverName() const noexcept;

    [[nodiscard]] std::size_t playbackDeviceCount() const noexcept;
    [[nodiscard]] std::string_view playbackDeviceName(
        std::size_t index) const noexcept;

    [[nodiscard]] std::string_view
    defaultPlaybackDeviceName() const noexcept;

private:
    bool initialized_ = false;
    std::string lastError_;
    std::string audioDriverName_;
    std::string defaultPlaybackDeviceName_;
    std::vector<std::string> playbackDeviceNames_;
};
}