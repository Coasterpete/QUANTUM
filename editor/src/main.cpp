#include <quantum/engine/Application.hpp>

#include <SDL3/SDL_main.h>

#include <cstdio>
#include <exception>
#include <print>

int main(int argc, char* argv[])
{
    try
    {
        quantum::engine::Application application;
        return application.run();
    }
    catch (const std::exception& exception)
    {
        std::println(stderr, "Fatal Error: {}", exception.what());
        return 1;
    }
}