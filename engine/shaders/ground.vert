#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec2 inUv;

layout(location = 0) out vec3 worldPosition;
layout(location = 1) out vec2 surfaceUv;

layout(push_constant) uniform GroundDraw
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
    // UV tiling is already baked into the vertex data, so the fragment stage
    // needs no additional uniform beyond the shared 128-byte block.
    surfaceUv = inUv;
}
