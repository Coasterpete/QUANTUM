#include <quantum/engine/Application.hpp>

#include <print>

namespace quantum::engine
{
    int Application::run()
    {
        std::println("QUANTUM CoasterWorks Engine");
        std::println("Initializing Vulkan...");
        std::println("GLFW initialized.");
        std::println("Application started successfully.");

        return 0;
    }
}