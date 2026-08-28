#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 vertexColor;

layout(push_constant) uniform CameraTransform
{
    mat4 viewProjection;
    vec4 highlightColor;
} camera;

void main()
{
    gl_Position = camera.viewProjection * vec4(inPosition, 1.0);
    vertexColor = vec4(
        mix(inColor.rgb, camera.highlightColor.rgb,
            camera.highlightColor.a),
        inColor.a
    );
}
