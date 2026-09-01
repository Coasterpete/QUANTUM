#version 450

layout(location = 0) in vec3 worldNormal;
layout(location = 1) in vec4 vertexColor;

layout(location = 0) out vec4 outColor;

void main()
{
    const vec3 lightDirection = normalize(vec3(-0.45, -0.35, 0.82));
    const float ambient = 0.28;
    const float diffuse = 0.72 * max(dot(normalize(worldNormal),
        lightDirection), 0.0);
    outColor = vec4(vertexColor.rgb * (ambient + diffuse), vertexColor.a);
}
