#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(location = 0) out vec3 worldNormal;
layout(location = 1) out vec4 vertexColor;

layout(push_constant) uniform TrackDraw
{
    mat4 viewProjection;
    vec4 baseColor;
    vec4 colorOverride;
} draw;

void main()
{
    gl_Position = draw.viewProjection * vec4(inPosition, 1.0);
    worldNormal = normalize(inNormal);
    vertexColor = vec4(
        mix(draw.baseColor.rgb, draw.colorOverride.rgb,
            draw.colorOverride.a),
        draw.baseColor.a
    );
}
