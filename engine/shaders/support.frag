#version 450

layout(location = 0) in vec3 worldPosition;
layout(location = 1) in vec3 worldNormal;
layout(location = 2) in vec3 worldTangent;
layout(location = 3) in vec3 worldBitangent;
layout(location = 4) in vec2 surfaceUv;
layout(location = 5) flat in int endFace;
layout(location = 6) in vec2 physicalUv;
layout(location = 7) flat in int pattern;
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
    // xyz = sRGB material tint, w = metallic.
    vec4 baseColor;
    vec4 cameraExposure;
    vec4 sunDirectionIntensity;
    // x = material roughness, y = normal strength,
    // z = environment intensity, w = environment rotation.
    vec4 surface;
    // x/z = longitudinal/transverse Core units per repeat, y = material,
    // w = meters per Core coordinate unit.
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
    int material = int(draw.timber.y + .5);
    bool timber = material == 0;
    vec3 N = normalize(worldNormal);
    vec3 T = normalize(worldTangent - N * dot(N, worldTangent));
    vec3 B = cross(N, T);
    if (pattern > 0)
    {
        // Grating/chain mesh is a perforated sheet, not individual wires.
        vec2 cell = pattern == 2 ? vec2(physicalUv.x + physicalUv.y, physicalUv.x - physicalUv.y)
            * 14 : physicalUv * 20;
        vec2 distanceToWire = abs(fract(cell + .5) - .5);
        vec2 aa = fwidth(cell);
        if (all(greaterThan(distanceToWire, vec2(.10) + aa))) discard;
    }
    vec3 detail = vec3(1);
    float roughness = draw.surface.x;
    if (timber && endFace == 0)
    {
        // The authored pine set is not seamless. Blend an offset copy near
        // its boundaries instead of repeating a hard rectangular edge.
        vec2 uv = surfaceUv;
        vec2 blend = smoothstep(vec2(.32), vec2(.5), abs(fract(uv) - .5));
        vec3 a = texture(albedoMap, uv).rgb;
        vec3 b = texture(albedoMap, uv + vec2(.5,0)).rgb;
        vec3 c = texture(albedoMap, uv + vec2(0,.5)).rgb;
        vec3 d = texture(albedoMap, uv + vec2(.5,.5)).rgb;
        detail = mix(mix(a,b,blend.x),mix(c,d,blend.x),blend.y);
        vec3 normalSample = mix(mix(texture(normalMap,uv).xyz,texture(normalMap,uv+vec2(.5,0)).xyz,blend.x),
            mix(texture(normalMap,uv+vec2(0,.5)).xyz,texture(normalMap,uv+vec2(.5,.5)).xyz,blend.x),blend.y)*2-1;
        N = normalize(T*normalSample.x*draw.surface.y + B*normalSample.y*draw.surface.y
            + N*max(normalSample.z,.1));
        float r = mix(mix(texture(roughnessMap,uv).r,texture(roughnessMap,uv+vec2(.5,0)).r,blend.x),
            mix(texture(roughnessMap,uv+vec2(0,.5)).r,texture(roughnessMap,uv+vec2(.5,.5)).r,blend.x),blend.y);
        roughness = clamp(roughness * mix(.75,1.25,r), .045, 1);
    }
    else if (timber)
    {
        // End grain in section space, distinct from longitudinal pine detail.
        float rings = sin(length(surfaceUv*vec2(1,1.61)+vec2(.3,.2))*130);
        detail = vec3(.85 + .08*rings);
    }
    else
    {
        // Conservative physical-scale material signals. Metals never sample
        // the pine maps. Concrete pores / galvanized mottling stay in local UVs.
        float noise = sin(surfaceUv.x*183.1 + sin(surfaceUv.y*91.7))
            * sin(surfaceUv.y*157.3 + sin(surfaceUv.x*77.1));
        float strength = material == 3 ? .10 : material == 2 ? .055 : .012;
        detail = vec3(1 + noise * strength);
        roughness = clamp(roughness + noise*strength*.4, .045, 1);
    }
    vec3 base = srgbToLinear(draw.baseColor.rgb) * detail;

    vec3 V = normalize(draw.cameraExposure.xyz - worldPosition);
    vec3 L = normalize(draw.sunDirectionIntensity.xyz);
    vec3 H = normalize(V + L);
    float NoL = max(dot(N, L), 0.0);
    float NoV = max(dot(N, V), 0.001);
    float NoH = max(dot(N, H), 0.0);
    float VoH = max(dot(V, H), 0.0);

    float metallic = draw.baseColor.a;
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
        ambient = ((1.0 - Fambient) * (1.0 - metallic) * base * irradiance
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
