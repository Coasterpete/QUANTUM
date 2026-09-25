#version 450

layout(location = 0) in vec2 ndc;
layout(location = 0) out vec4 outColor;
layout(set = 0, binding = 3) uniform samplerCube skyMap;

layout(push_constant) uniform SkyDraw
{
    mat4 inverseViewProjection;
    vec4 settings; // rotation (radians), exposure
} draw;

vec3 toneMap(vec3 color)
{
    return clamp((color * (2.51 * color + 0.03)) /
        (color * (2.43 * color + 0.59) + 0.14), 0.0, 1.0);
}

void main()
{
    vec4 nearPosition = draw.inverseViewProjection * vec4(ndc, 0.0, 1.0);
    vec4 farPosition = draw.inverseViewProjection * vec4(ndc, 1.0, 1.0);
    vec3 direction = normalize(farPosition.xyz / farPosition.w
        - nearPosition.xyz / nearPosition.w);
    float c = cos(draw.settings.x);
    float s = sin(draw.settings.x);
    direction = vec3(c * direction.x + s * direction.y,
        -s * direction.x + c * direction.y, direction.z);
    outColor = vec4(toneMap(texture(skyMap, direction).rgb
        * draw.settings.y), 1.0);
}
