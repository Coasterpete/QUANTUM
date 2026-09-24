#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(location = 0) out vec3 worldPosition;
layout(location = 1) out vec3 worldNormal;

layout(push_constant) uniform TrackDraw
{
    mat4 viewProjection;
    vec4 baseColor;
    vec4 cameraExposure;
    vec4 sunDirectionIntensity;
    vec4 surface;
} draw;

void main()
{
    gl_Position = draw.viewProjection * vec4(inPosition, 1.0);
    worldPosition = inPosition;
    worldNormal = normalize(inNormal);
}
