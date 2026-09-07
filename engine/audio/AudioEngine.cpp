#include "AudioEngine.hpp"

#include <SDL3/SDL_init.h>
#include <SDL3/SDL_audio.h>
#include <SDL3/SDL_stdinc.h>

namespace quantum::audio
{
    AudioEngine::~AudioEngine()
    {
        shutdown();
    }

    bool AudioEngine::initialize() noexcept
    {
        // Re-initializing releases the previous subsystem reference first so
        // SDL's ref-counted subsystem initialization is never unbalanced.
        shutdown();

        if (!SDL_InitSubSystem(SDL_INIT_AUDIO))
        {
            if (const char* const error = SDL_GetError())
            {
                lastError_ = error;
            }
            return false;
        }

        initialized_ = true;
        lastError_.clear();

        if (const char* const driver = SDL_GetCurrentAudioDriver())
        {
            audioDriverName_ = driver;
        }

        int deviceCount = 0;
        SDL_AudioDeviceID* const deviceIds =
            SDL_GetAudioPlaybackDevices(&deviceCount);
        if (deviceIds != nullptr)
        {
            playbackDeviceNames_.clear();
            playbackDeviceNames_.reserve(
                static_cast<std::size_t>(deviceCount));
            for (int index = 0; index < deviceCount; ++index)
            {
                if (const char* const name =
                    SDL_GetAudioDeviceName(deviceIds[index]))
                {
                    playbackDeviceNames_.emplace_back(name);
                }
            }
            SDL_free(deviceIds);
        }
        else
        {
            // Enumeration failure is not subsystem-initialization failure.
            // Preserve the diagnostic while keeping the engine initialized.
            playbackDeviceNames_.clear();
            if (const char* const error = SDL_GetError())
            {
                lastError_ = error;
            }
        }

        if (const char* const defaultName =
            SDL_GetAudioDeviceName(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK))
        {
            defaultPlaybackDeviceName_ = defaultName;
        }

        return true;
    }

    void AudioEngine::shutdown() noexcept
    {
        if (initialized_)
        {
            SDL_QuitSubSystem(SDL_INIT_AUDIO);
            initialized_ = false;
        }
        lastError_.clear();
        audioDriverName_.clear();
        defaultPlaybackDeviceName_.clear();
        playbackDeviceNames_.clear();
    }

    bool AudioEngine::isInitialized() const noexcept
    {
        return initialized_;
    }

    std::string_view AudioEngine::lastError() const noexcept
    {
        return lastError_;
    }

    std::string_view AudioEngine::audioDriverName() const noexcept
    {
        return audioDriverName_;
    }

    std::size_t AudioEngine::playbackDeviceCount() const noexcept
    {
        return playbackDeviceNames_.size();
    }

    std::string_view AudioEngine::playbackDeviceName(
        const std::size_t index) const noexcept
    {
        if (index >= playbackDeviceNames_.size())
        {
            return {};
        }
        return playbackDeviceNames_[index];
    }

    std::string_view AudioEngine::defaultPlaybackDeviceName() const noexcept
    {
        return defaultPlaybackDeviceName_;
    }
}