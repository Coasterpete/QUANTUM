#include <quantum/engine/Application.hpp>

#include <cstdio>
#include <exception>
#include <print>

int main()
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