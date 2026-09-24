#version 450

layout(location = 0) in vec3 worldPosition;
layout(location = 1) in vec3 worldNormal;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform TrackDraw
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

void main()
{
    vec3 base = srgbToLinear(draw.baseColor.rgb);
    if (draw.surface.w > 0.5)
    {
        // Wireframe and editor edges preserve their authored display colors.
        outColor = vec4(base, draw.baseColor.a);
        return;
    }

    vec3 N = normalize(worldNormal);
    vec3 V = normalize(draw.cameraExposure.xyz - worldPosition);
    vec3 L = normalize(draw.sunDirectionIntensity.xyz);
    vec3 H = normalize(V + L);
    float NoL = max(dot(N, L), 0.0);
    float NoV = max(dot(N, V), 0.001);
    float NoH = max(dot(N, H), 0.0);
    float VoH = max(dot(V, H), 0.0);
    float metallic = clamp(draw.surface.x, 0.0, 1.0);
    float roughness = clamp(draw.surface.y, 0.045, 1.0);
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
    // A modest sky fill keeps unlit faces readable until image lighting exists.
    vec3 ambient = 0.18 * base * (1.0 - metallic) + 0.06 * F0;
    outColor = vec4(toneMap((direct + ambient)
        * draw.cameraExposure.w), draw.baseColor.a);
}
