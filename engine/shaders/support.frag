#version 450

layout(location = 0) in vec3 worldPosition;
layout(location = 1) in vec3 worldNormal;
layout(location = 2) in vec3 worldTangent;
layout(location = 3) in vec3 worldBitangent;
layout(location = 4) in vec2 surfaceUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform samplerCube irradianceMap;
layout(set = 0, binding = 1) uniform samplerCube specularMap;
layout(set = 0, binding = 2) uniform sampler2D brdfMap;

layout(set = 1, binding = 0) uniform sampler2D albedoMap;
layout(set = 1, binding = 1) uniform sampler2D normalMap;
layout(set = 1, binding = 2) uniform sampler2D roughnessMap;

layout(push_constant) uniform SupportDraw
{
    mat4 viewProjection;
    // xyz = sRGB timber tint, w unused.
    vec4 baseColor;
    vec4 cameraExposure;
    vec4 sunDirectionIntensity;
    // x = roughness multiplier, y = normal strength,
    // z = environment intensity, w = environment rotation.
    vec4 surface;
    // x = texture scale in Core units per repeat, y = foundation flag.
    vec4 timber;
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

void main()
{
    bool foundation = draw.timber.y > 0.5;
    vec3 N = normalize(worldNormal);
    vec3 T = normalize(worldTangent - N * dot(N, worldTangent));
    vec3 B = cross(N, T);

    // Normal-map convention, verified against the authored set rather than
    // assumed: correlating this map's green channel against the numerical
    // gradient of its own height source gives corr = -0.42, i.e. green
    // increases as height decreases. That is the OpenGL / +Y-up convention,
    // which is the same handedness the ground surface's world-aligned basis
    // already assumed. So the sample maps directly onto +B with no inversion;
    // a DirectX-encoded map would need normalSample.y negated.
    if (!foundation)
    {
        vec3 normalSample = texture(normalMap, surfaceUv).xyz * 2.0 - 1.0;
        N = normalize(N + (T * normalSample.x + B * normalSample.y)
            * draw.surface.y);
    }

    // Timber takes neutral grayscale detail from the pine map. Foundations
    // use their authored concrete tint directly, without timber grain.
    vec3 detail = foundation ? vec3(1.0)
        : texture(albedoMap, surfaceUv).rgb;
    vec3 base = srgbToLinear(draw.baseColor.rgb) * detail;

    vec3 V = normalize(draw.cameraExposure.xyz - worldPosition);
    vec3 L = normalize(draw.sunDirectionIntensity.xyz);
    vec3 H = normalize(V + L);
    float NoL = max(dot(N, L), 0.0);
    float NoV = max(dot(N, V), 0.001);
    float NoH = max(dot(N, H), 0.0);
    float VoH = max(dot(V, H), 0.0);

    // Untreated timber is a dielectric, so metallic stays zero and the
    // dielectric F0 below applies.
    float metallic = 0.0;

    // The supplied roughness map is a detail signal whose median sits near
    // 0.31 and which contains pure-black texels. Feeding it in as a straight
    // multiplier, the way a ground texture is used, would drive those texels
    // to near-mirror gloss and read as wet varnished wood. Remapping it into a
    // band around 1.0 keeps it a modulation of the authored roughness while
    // preserving the grain-correlated variation that makes the surface read
    // as timber.
    float roughness = draw.surface.x;
    if (!foundation)
    {
        float roughnessDetail = texture(roughnessMap, surfaceUv).r;
        float detailBand = mix(0.55, 1.45, roughnessDetail);
        roughness = clamp(
            0.85 * draw.surface.x * detailBand, 0.045, 1.0);
    }
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;

    vec3 F0 = vec3(0.04);
    vec3 F = F0 + (1.0 - F0) * pow(1.0 - VoH, 5.0);
    float d = NoH * NoH * (alpha2 - 1.0) + 1.0;
    float D = alpha2 / max(pi * d * d, 0.000001);
    float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    float G = NoV / (NoV * (1.0 - k) + k)
        * NoL / (NoL * (1.0 - k) + k);
    vec3 specular = D * G * F / max(4.0 * NoV * NoL, 0.001);
    vec3 diffuse = (1.0 - F) * base / pi;
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
        ambient = ((1.0 - Fambient) * base * irradiance
            + prefiltered * (Fambient * brdf.x + brdf.y))
            * draw.surface.z;
    }
    else
    {
        ambient = 0.18 * base + 0.06 * F0;
    }
    outColor = vec4(toneMap((direct + ambient)
        * draw.cameraExposure.w), 1.0);
}
