#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 instanceTransform0;
layout(location = 3) in vec4 instanceTransform1;
layout(location = 4) in vec4 instanceTransform2;
layout(location = 5) in vec4 instanceTransform3;

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
    mat4 transform = mat4(
        instanceTransform0,
        instanceTransform1,
        instanceTransform2,
        instanceTransform3
    );
    vec4 worldPosition = transform * vec4(inPosition, 1.0);
    gl_Position = draw.viewProjection * worldPosition;
    worldNormal = normalize(transpose(inverse(mat3(transform))) * inNormal);
    vertexColor = vec4(
        mix(draw.baseColor.rgb, draw.colorOverride.rgb,
            draw.colorOverride.a),
        draw.baseColor.a
    );
}
