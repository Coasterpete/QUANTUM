#version 450

layout(location = 0) in vec3 worldPosition;
layout(location = 1) in vec2 surfaceUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform samplerCube irradianceMap;
layout(set = 0, binding = 1) uniform samplerCube specularMap;
layout(set = 0, binding = 2) uniform sampler2D brdfMap;

layout(set = 1, binding = 0) uniform sampler2D albedoMap;
layout(set = 1, binding = 1) uniform sampler2D normalMap;
layout(set = 1, binding = 2) uniform sampler2D roughnessMap;

layout(push_constant) uniform GroundDraw
{
    mat4 viewProjection;
    vec4 baseColor;
    vec4 cameraExposure;
    vec4 sunDirectionIntensity;
    vec4 surface;
} draw;

const float pi = 3.14159265359;

vec3 srgbToLinear(vec3 color)
{
    return mix(color / 12.92,
        pow((color + 0.055) / 1.055, vec3(2.4)),
        greaterThan(color, vec3(0.04045)));
}

vec3 toneMap(vec3 color)
{
    // ACES fitted curve, evaluated in linear light before the sRGB attachment.
    return clamp((color * (2.51 * color + 0.03)) /
        (color * (2.43 * color + 0.59) + 0.14), 0.0, 1.0);
}

vec3 rotateEnvironment(vec3 direction, float angle)
{
    float c = cos(angle);
    float s = sin(angle);
    return vec3(c * direction.x + s * direction.y,
        -s * direction.x + c * direction.y, direction.z);
}

// The albedo map is uploaded as an _SRGB image, so the sampler already returns
// linear values. The authored base color is still sRGB-encoded, exactly as in
// the track shader, so both surfaces tint identically.
vec3 resolveAlbedo()
{
    return srgbToLinear(draw.baseColor.rgb) * texture(albedoMap, surfaceUv).rgb;
}

void main()
{
    vec3 base = resolveAlbedo();
    // The M0 surface is a horizontal quad whose UVs run along +X and +Y, so its
    // tangent frame is the world frame: T = +X, B = +Y, N = +Z. That lets the
    // tangent-space normal map be applied without a per-vertex tangent
    // attribute. The 1x1 built-in fallback decodes to (0, 0, 1) and therefore
    // reproduces the flat geometric normal exactly.
    vec3 N = normalize(texture(normalMap, surfaceUv).xyz);
    vec3 V = normalize(draw.cameraExposure.xyz - worldPosition);
    vec3 L = normalize(draw.sunDirectionIntensity.xyz);
    vec3 H = normalize(V + L);
    float NoL = max(dot(N, L), 0.0);
    float NoV = max(dot(N, V), 0.001);
    float NoH = max(dot(N, H), 0.0);
    float VoH = max(dot(V, H), 0.0);
    float metallic = clamp(draw.surface.x, 0.0, 1.0);
    // The roughness map is authored as a multiplier on the surface's scalar
    // roughness factor; the 1x1 built-in fallback is white, so an unset map
    // leaves the authored roughness untouched.
    float roughness = clamp(draw.surface.y
        * texture(roughnessMap, surfaceUv).r, 0.045, 1.0);
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;

    vec3 F0 = mix(vec3(0.04), base, metallic);
    vec3 F = F0 + (1.0 - F0) * pow(1.0 - VoH, 5.0);
    float d = NoH * NoH * (alpha2 - 1.0) + 1.0;
    float D = alpha2 / max(pi * d * d, 0.000001);
    float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    float G = NoV / (NoV * (1.0 - k) + k)
        * NoL / (NoL * (1.0 - k) + k);
    vec3 specular = D * G * F / max(4.0 * NoV * NoL, 0.001);
    vec3 diffuse = (1.0 - F) * (1.0 - metallic) * base / pi;
    vec3 direct = (diffuse + specular) * NoL
        * draw.sunDirectionIntensity.w;
    vec3 ambient;
    if (draw.surface.w >= 0.0)
    {
        vec3 Fambient = F0 + (max(vec3(1.0 - roughness), F0) - F0)
            * pow(1.0 - NoV, 5.0);
        vec3 irradiance = texture(irradianceMap,
            rotateEnvironment(N, draw.surface.w)).rgb;
        vec3 reflection = reflect(-V, N);
        vec3 prefiltered = textureLod(specularMap,
            rotateEnvironment(reflection, draw.surface.w),
            roughness * 5.0).rgb;
        vec2 brdf = texture(brdfMap, vec2(NoV, roughness)).rg;
        ambient = ((1.0 - Fambient) * (1.0 - metallic) * base
            * irradiance + prefiltered * (Fambient * brdf.x + brdf.y))
            * draw.surface.z;
    }
    else
    {
        ambient = 0.18 * base * (1.0 - metallic) + 0.06 * F0;
    }
    outColor = vec4(toneMap((direct + ambient)
        * draw.cameraExposure.w), draw.baseColor.a);
}
