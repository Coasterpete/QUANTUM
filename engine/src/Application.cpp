#include <quantum/engine/Application.hpp>
#include <quantum/renderer/VulkanContext.hpp>

#include <SDL3/SDL.h>

#include <stdexcept>
#include <string>

namespace quantum::engine
{
    int Application::run()
    {
        if (!SDL_Init(SDL_INIT_VIDEO))
        {
            throw std::runtime_error(
                std::string("SDL_Init failed: ") + SDL_GetError()
            );
        }

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

        {
            quantum::renderer::VulkanContext vulkan;
            vulkan.initialize(window);

            bool running = true;

            while (running)
            {
                SDL_Event event{};

                while (SDL_PollEvent(&event))
                {
                    if (event.type == SDL_EVENT_QUIT)
                    {
                        running = false;
                    }
                }

                SDL_Delay(1);
            }
        }

        SDL_DestroyWindow(window);
        SDL_Quit();

        return 0;
    }
}