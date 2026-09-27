#include <quantum/renderer/EnvironmentAssets.hpp>
#include <quantum/renderer/EnvironmentMap.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <typeinfo>
#include <vector>

namespace
{
    using namespace quantum::renderer;

    void require(const bool condition, const char* const message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void require(const bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    [[nodiscard]] std::string messageFrom(const std::function<void()>& callable)
    {
        try
        {
            callable();
        }
        catch (const std::exception& exception)
        {
            return exception.what();
        }
        return {};
    }

    [[nodiscard]] float maximumComponent(const EnvironmentPixels& pixels)
    {
        float maximum = 0.0F;
        for (const float value : pixels.rgba)
        {
            maximum = std::max(maximum, value);
        }
        return maximum;
    }

    [[nodiscard]] double meanLuma(const EnvironmentPixels& pixels)
    {
        if (pixels.rgba.empty())
        {
            return 0.0;
        }
        double total = 0.0;
        const std::size_t texels = pixels.rgba.size() / 4;
        for (std::size_t index = 0; index < texels; ++index)
        {
            total += 0.2126 * pixels.rgba[4 * index]
                + 0.7152 * pixels.rgba[4 * index + 1]
                + 0.0722 * pixels.rgba[4 * index + 2];
        }
        return total / static_cast<double>(texels);
    }

    std::string messageFrom(const std::function<void()>& callable);

    // "assets://environment/x.hdr" resolves to <root>/assets/environment/x.hdr,
    // matching VulkanContext::environmentFilePath.
    [[nodiscard]] std::filesystem::path resolveAsset(
        const std::filesystem::path& root, const std::string& identifier)
    {
        constexpr std::string_view scheme = "assets://";
        return root / "assets"
            / std::filesystem::path(identifier.substr(scheme.size()));
    }

    void bundledRegistryIsConsistent()
    {
        const auto assets = bundledEnvironmentAssets();
        require(assets.size() >= 2,
            "Ground Surface M0 requires at least the original and the selected "
            "sky to be selectable.");
        std::vector<std::string> identifiers;
        for (const EnvironmentAsset& asset : assets)
        {
            require(asset.identifier.starts_with("assets://environment/"),
                "Bundled environments use package-relative identities.");
            require(asset.identifier.ends_with(".hdr"),
                "The pipeline decodes Radiance .hdr panoramas only.");
            require(!asset.displayName.empty(), "Every sky needs a display name.");
            require(!asset.credit.empty(),
                "Every bundled sky must record its provenance and license.");
            identifiers.push_back(asset.identifier);
        }
        for (std::size_t a = 0; a < identifiers.size(); ++a)
            for (std::size_t b = a + 1; b < identifiers.size(); ++b)
                require(identifiers[a] != identifiers[b],
                    "Bundled environment identities must be unique.");

        require(findBundledEnvironmentAsset(identifiers.front()) != nullptr,
            "The first bundled environment resolves.");
        require(findBundledEnvironmentAsset("") == nullptr,
            "The empty selection is not a bundled environment.");
        require(findBundledEnvironmentAsset("assets://environment/nope.hdr")
                == nullptr,
            "An unknown environment does not resolve.");

        require(validateEnvironmentAssetIdentifier("").empty(),
            "None is a valid selection.");
        require(validateEnvironmentAssetIdentifier(identifiers.front())
                == identifiers.front(),
            "A canonical bundled identifier round-trips.");
        require(findBundledEnvironmentAsset(
                    defaultEnvironmentAssetIdentifier) != nullptr,
            "The explicit new-session environment is bundled.");
        require(defaultEnvironmentAssetIdentifier
                == "assets://environment/dayskyhdri027b_1k.hdr",
            "The new-session environment remains the neutral DaySky asset.");
        require(!messageFrom([] {
            static_cast<void>(validateEnvironmentAssetIdentifier(
                "C:/skies/mine.hdr"));
        }).empty(), "An absolute path is rejected.");
        require(!messageFrom([] {
            static_cast<void>(validateEnvironmentAssetIdentifier(
                "assets://ground/grass.png"));
        }).empty(), "Another package root is rejected.");

        // The lookup hands out a pointer into the shared registry, so it must
        // stay valid after the lookup returns and across repeated calls.
        const EnvironmentAsset* first = findBundledEnvironmentAsset(
            identifiers.front());
        const EnvironmentAsset* again = findBundledEnvironmentAsset(
            identifiers.front());
        require(first != nullptr && first == again
            && first->identifier == identifiers.front(),
            "A bundled-environment lookup returns a stable registry pointer.");
    }

    void everyBundledEnvironmentDecodes(const std::filesystem::path& root)
    {
        for (const EnvironmentAsset& asset : bundledEnvironmentAssets())
        {
            const std::filesystem::path path =
                resolveAsset(root, asset.identifier);
            require(std::filesystem::is_regular_file(path),
                "A bundled environment is missing from the source tree: "
                    + path.string());
            const ProcessedEnvironment processed = preprocessEnvironment(path);

            require(processed.sky.width == 256 && processed.sky.height == 256
                && processed.sky.layers == 6 && processed.sky.mipLevels == 1,
                "The sky cubemap has the documented shape.");
            require(processed.irradiance.width == 24
                && processed.irradiance.layers == 6,
                "The irradiance cubemap has the documented shape.");
            require(processed.specular.width == 128
                && processed.specular.mipLevels == 6,
                "The prefiltered specular chain has the documented shape.");
            require(processed.brdf.width == 128 && processed.brdf.layers == 1,
                "The BRDF LUT has the documented shape.");

            // A real HDR panorama keeps values well above display white, and
            // the diffuse irradiance estimate must be strictly positive.
            require(maximumComponent(processed.sky) > 8.0F,
                "The decoded sky retains high-dynamic-range values.");
            require(meanLuma(processed.irradiance) > 0.001,
                "The decoded sky produces usable diffuse irradiance.");
            // The BRDF LUT is environment independent, so a second decode must
            // reproduce it exactly.
            require(processed.brdf.rgba
                    == preprocessEnvironment(path).brdf.rgba,
                "The BRDF integration LUT is deterministic.");
        }
    }

    void selectedSkyIsVisuallyDistinct(
        const std::filesystem::path& root)
    {
        const auto assets = bundledEnvironmentAssets();
        const ProcessedEnvironment first =
            preprocessEnvironment(resolveAsset(root, assets.at(0).identifier));
        const ProcessedEnvironment second =
            preprocessEnvironment(resolveAsset(root, assets.at(1).identifier));

        // Selecting a different sky must actually change the rendered sky and
        // the image-based lighting, otherwise the selector would be cosmetic.
        require(first.sky.rgba != second.sky.rgba,
            "The two bundled skies decode to different panoramas.");
        require(std::abs(meanLuma(first.irradiance) - meanLuma(second.irradiance))
                > 1.0e-4,
            "The two bundled skies produce different diffuse irradiance, so "
            "switching between them changes image-based lighting.");
    }

    void malformedEnvironmentsAreRejected(
        const std::filesystem::path& root)
    {
        const std::filesystem::path temporary =
            root / "environment-map-tests";
        std::filesystem::create_directories(temporary);

        // The loader must reject anything it cannot decode rather than
        // producing a degenerate environment.
        const auto rejects = [&](const std::string& name,
            const std::string& contents, const char* const why)
        {
            const std::filesystem::path path = temporary / name;
            std::ofstream(path, std::ios::binary) << contents;
            const std::string message = messageFrom([&] {
                static_cast<void>(preprocessEnvironment(path));
            });
            require(!message.empty(), why);
        };

        rejects("no-format.hdr", "#?RADIANCE\n\n-Y 8 +X 16\n",
            "A header without FORMAT is rejected.");
        rejects("bad-orientation.hdr",
            "#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n+Y 8 +X 16\n",
            "An unsupported orientation is rejected.");
        rejects("truncated.hdr",
            "#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 8 +X 16\n\x02\x02",
            "A truncated scanline is rejected.");
        std::error_code error;
        std::filesystem::remove_all(temporary, error);
    }
}

int main(int argc, char* argv[])
{
    try
    {
        require(argc == 2, "pass the QUANTUM source root");
        const std::filesystem::path root = argv[1];
        bundledRegistryIsConsistent();
        everyBundledEnvironmentDecodes(root);
        selectedSkyIsVisuallyDistinct(root);
        malformedEnvironmentsAreRejected(root);
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Environment asset test failure ["
            << typeid(exception).name() << "]: "
            << exception.what() << '\n';
        return 1;
    }

    std::cout << "Environment asset tests passed.\n";
    return 0;
}
