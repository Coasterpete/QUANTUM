#include <quantum/renderer/EnvironmentMap.hpp>

#include <glm/geometric.hpp>
#include <glm/common.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using quantum::renderer::EnvironmentPixels;
    constexpr float pi = std::numbers::pi_v<float>;

    struct Panorama
    {
        int width = 0;
        int height = 0;
        std::vector<glm::vec3> pixels;
    };

    [[nodiscard]] Panorama readRadiance(const std::filesystem::path& path)
    {
        std::ifstream input(path, std::ios::binary);
        if (!input)
            throw std::runtime_error("Cannot open HDR environment: " + path.string());

        std::string line;
        bool formatFound = false;
        while (std::getline(input, line) && !line.empty() && line != "\r")
            formatFound |= line == "FORMAT=32-bit_rle_rgbe" || line == "FORMAT=32-bit_rle_rgbe\r";
        if (!formatFound || !std::getline(input, line))
            throw std::runtime_error("Unsupported Radiance HDR header: " + path.string());

        Panorama image;
        char ySign = 0, xSign = 0, yAxis = 0, xAxis = 0;
        if (std::sscanf(line.c_str(), "%c%c %d %c%c %d",
                &ySign, &yAxis, &image.height,
                &xSign, &xAxis, &image.width) != 6
            || ySign != '-' || yAxis != 'Y' || xSign != '+' || xAxis != 'X'
            || image.width < 8 || image.width > 32767
            || image.height < 1 || image.height > 16384)
        {
            throw std::runtime_error("Unsupported HDR orientation or dimensions: "
                + path.string());
        }

        image.pixels.resize(static_cast<std::size_t>(image.width) * image.height);
        std::vector<std::uint8_t> scanline(static_cast<std::size_t>(image.width) * 4);
        for (int y = 0; y < image.height; ++y)
        {
            std::array<unsigned char, 4> header{};
            input.read(reinterpret_cast<char*>(header.data()), 4);
            if (!input || header[0] != 2 || header[1] != 2
                || (static_cast<int>(header[2]) << 8 | header[3]) != image.width)
                throw std::runtime_error("Invalid HDR RLE scanline: " + path.string());

            for (int channel = 0; channel < 4; ++channel)
            {
                int x = 0;
                while (x < image.width)
                {
                    const int countCode = input.get();
                    if (countCode == EOF || countCode == 0)
                        throw std::runtime_error("Truncated HDR scanline: " + path.string());
                    const bool run = countCode > 128;
                    const int count = run ? countCode - 128 : countCode;
                    if (count > image.width - x)
                        throw std::runtime_error("HDR RLE run exceeds scanline: " + path.string());
                    if (run)
                    {
                        const int value = input.get();
                        if (value == EOF)
                            throw std::runtime_error("Truncated HDR RLE run: " + path.string());
                        std::fill_n(scanline.data() + channel * image.width + x,
                            count, static_cast<std::uint8_t>(value));
                    }
                    else
                    {
                        input.read(reinterpret_cast<char*>(scanline.data()
                            + channel * image.width + x), count);
                        if (!input)
                            throw std::runtime_error("Truncated HDR RLE literal: " + path.string());
                    }
                    x += count;
                }
            }
            for (int x = 0; x < image.width; ++x)
            {
                const int exponent = scanline[3 * image.width + x];
                const float scale = exponent == 0 ? 0.0F
                    : std::ldexp(1.0F, exponent - 136);
                image.pixels[static_cast<std::size_t>(y) * image.width + x] = {
                    scanline[x] * scale,
                    scanline[image.width + x] * scale,
                    scanline[2 * image.width + x] * scale};
            }
        }
        return image;
    }

    [[nodiscard]] glm::vec3 panoramaSample(const Panorama& panorama,
        const glm::vec3 direction)
    {
        const float u = std::atan2(direction.y, direction.x) / (2.0F * pi)
            + 0.5F;
        const float v = std::acos(std::clamp(direction.z, -1.0F, 1.0F)) / pi;
        const float x = u * panorama.width - 0.5F;
        const float y = v * panorama.height - 0.5F;
        const int x0 = static_cast<int>(std::floor(x));
        const int y0 = std::clamp(static_cast<int>(std::floor(y)), 0,
            panorama.height - 1);
        const int y1 = std::min(y0 + 1, panorama.height - 1);
        const int x1 = x0 + 1;
        const auto pixel = [&](const int px, const int py)
        {
            const int wrapped = (px % panorama.width + panorama.width)
                % panorama.width;
            return panorama.pixels[static_cast<std::size_t>(py)
                * panorama.width + wrapped];
        };
        const float tx = x - std::floor(x);
        const float ty = std::clamp(y - std::floor(y), 0.0F, 1.0F);
        return glm::mix(glm::mix(pixel(x0, y0), pixel(x1, y0), tx),
            glm::mix(pixel(x0, y1), pixel(x1, y1), tx), ty);
    }

    [[nodiscard]] glm::vec3 cubeDirection(const int face,
        const int x, const int y, const int size)
    {
        const float u = 2.0F * (x + 0.5F) / size - 1.0F;
        const float v = 2.0F * (y + 0.5F) / size - 1.0F;
        const std::array<glm::vec3, 6> directions{{
            {1.0F, -v, -u}, {-1.0F, -v, u}, {u, 1.0F, v},
            {u, -1.0F, -v}, {u, -v, 1.0F}, {-u, -v, -1.0F}}};
        return glm::normalize(directions[face]);
    }

    [[nodiscard]] float radicalInverse(std::uint32_t bits)
    {
        bits = (bits << 16) | (bits >> 16);
        bits = ((bits & 0x55555555u) << 1) | ((bits & 0xAAAAAAAAu) >> 1);
        bits = ((bits & 0x33333333u) << 2) | ((bits & 0xCCCCCCCCu) >> 2);
        bits = ((bits & 0x0F0F0F0Fu) << 4) | ((bits & 0xF0F0F0F0u) >> 4);
        bits = ((bits & 0x00FF00FFu) << 8) | ((bits & 0xFF00FF00u) >> 8);
        return static_cast<float>(bits) * 2.3283064365386963e-10F;
    }

    [[nodiscard]] glm::vec3 orient(const glm::vec3& local,
        const glm::vec3 normal)
    {
        const glm::vec3 up = std::abs(normal.z) < 0.999F
            ? glm::vec3{0.0F, 0.0F, 1.0F}
            : glm::vec3{0.0F, 1.0F, 0.0F};
        const glm::vec3 tangent = glm::normalize(glm::cross(up, normal));
        return local.x * tangent + local.y * glm::cross(normal, tangent)
            + local.z * normal;
    }

    [[nodiscard]] glm::vec3 ggxHalfVector(const glm::vec2 sample,
        const float roughness, const glm::vec3 normal)
    {
        const float a = roughness * roughness;
        const float a2 = a * a;
        const float phi = 2.0F * pi * sample.x;
        const float cosine = std::sqrt((1.0F - sample.y)
            / (1.0F + (a2 - 1.0F) * sample.y));
        const float sine = std::sqrt(std::max(0.0F, 1.0F - cosine * cosine));
        return glm::normalize(orient({std::cos(phi) * sine,
            std::sin(phi) * sine, cosine}, normal));
    }

    void append(EnvironmentPixels& target, const glm::vec3 value)
    {
        target.rgba.insert(target.rgba.end(), {value.x, value.y, value.z, 1.0F});
    }

    [[nodiscard]] EnvironmentPixels makeSky(const Panorama& panorama)
    {
        EnvironmentPixels result{256, 256, 6, 1, {}};
        result.rgba.reserve(256u * 256u * 6u * 4u);
        for (int face = 0; face < 6; ++face)
            for (int y = 0; y < 256; ++y)
                for (int x = 0; x < 256; ++x)
                    append(result, panoramaSample(panorama,
                        cubeDirection(face, x, y, 256)));
        return result;
    }

    [[nodiscard]] EnvironmentPixels makeIrradiance(const Panorama& panorama)
    {
        constexpr int size = 24;
        constexpr int samples = 256;
        EnvironmentPixels result{size, size, 6, 1, {}};
        result.rgba.reserve(size * size * 6 * 4);
        for (int face = 0; face < 6; ++face)
            for (int y = 0; y < size; ++y)
                for (int x = 0; x < size; ++x)
                {
                    const glm::vec3 normal = cubeDirection(face, x, y, size);
                    glm::vec3 sum{0.0F};
                    for (int i = 0; i < samples; ++i)
                    {
                        const float phi = 2.0F * pi * radicalInverse(i);
                        const float cosine = std::sqrt(1.0F
                            - (i + 0.5F) / samples);
                        const float sine = std::sqrt(1.0F - cosine * cosine);
                        sum += panoramaSample(panorama, orient({
                            std::cos(phi) * sine,
                            std::sin(phi) * sine, cosine}, normal));
                    }
                    // Cosine-weighted samples estimate irradiance / pi.
                    append(result, sum / static_cast<float>(samples));
                }
        return result;
    }

    [[nodiscard]] EnvironmentPixels makeSpecular(const Panorama& panorama)
    {
        constexpr int size = 128;
        constexpr int mips = 6;
        constexpr int samples = 128;
        EnvironmentPixels result{size, size, 6, mips, {}};
        for (int face = 0; face < 6; ++face)
            for (int mip = 0; mip < mips; ++mip)
            {
                const int levelSize = size >> mip;
                const float roughness = static_cast<float>(mip) / (mips - 1);
                for (int y = 0; y < levelSize; ++y)
                    for (int x = 0; x < levelSize; ++x)
                    {
                        const glm::vec3 normal = cubeDirection(
                            face, x, y, levelSize);
                        if (mip == 0)
                        {
                            append(result, panoramaSample(panorama, normal));
                            continue;
                        }
                        glm::vec3 sum{0.0F};
                        float weight = 0.0F;
                        for (int i = 0; i < samples; ++i)
                        {
                            const glm::vec3 halfVector = ggxHalfVector({
                                (i + 0.5F) / samples, radicalInverse(i)},
                                roughness, normal);
                            const glm::vec3 light = glm::normalize(
                                2.0F * glm::dot(normal, halfVector)
                                    * halfVector - normal);
                            const float noL = std::max(glm::dot(normal, light),
                                0.0F);
                            if (noL > 0.0F)
                            {
                                sum += panoramaSample(panorama, light) * noL;
                                weight += noL;
                            }
                        }
                        append(result, weight > 0.0F ? sum / weight
                            : glm::vec3{0.0F});
                    }
            }
        return result;
    }

    [[nodiscard]] EnvironmentPixels makeBrdfLut()
    {
        constexpr int size = 128;
        constexpr int samples = 256;
        EnvironmentPixels result{size, size, 1, 1, {}};
        result.rgba.reserve(size * size * 4);
        const glm::vec3 normal{0.0F, 0.0F, 1.0F};
        for (int y = 0; y < size; ++y)
        {
            const float roughness = (y + 0.5F) / size;
            const float k = roughness * roughness * 0.5F;
            for (int x = 0; x < size; ++x)
            {
                const float noV = (x + 0.5F) / size;
                const glm::vec3 view{std::sqrt(1.0F - noV * noV), 0.0F, noV};
                float a = 0.0F, b = 0.0F;
                for (int i = 0; i < samples; ++i)
                {
                    const glm::vec3 halfVector = ggxHalfVector({
                        (i + 0.5F) / samples, radicalInverse(i)},
                        roughness, normal);
                    const glm::vec3 light = glm::normalize(
                        2.0F * glm::dot(view, halfVector) * halfVector - view);
                    const float noL = std::max(light.z, 0.0F);
                    if (noL <= 0.0F)
                        continue;
                    const float noH = std::max(halfVector.z, 0.0F);
                    const float voH = std::max(glm::dot(view, halfVector), 0.0F);
                    const float gv = noV / (noV * (1.0F - k) + k);
                    const float gl = noL / (noL * (1.0F - k) + k);
                    const float visible = gv * gl * voH
                        / std::max(noH * noV, 0.00001F);
                    const float fresnel = std::pow(1.0F - voH, 5.0F);
                    a += (1.0F - fresnel) * visible;
                    b += fresnel * visible;
                }
                append(result, {a / samples, b / samples, 0.0F});
            }
        }
        return result;
    }
}

namespace quantum::renderer
{
    ProcessedEnvironment preprocessEnvironment(
        const std::filesystem::path& hdrPath)
    {
        const Panorama panorama = readRadiance(hdrPath);
        return {makeSky(panorama), makeIrradiance(panorama),
            makeSpecular(panorama), makeBrdfLut()};
    }
}
