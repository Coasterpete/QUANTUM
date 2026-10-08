#version 450
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 instanceTransform0;
layout(location = 3) in vec4 instanceTransform1;
layout(location = 4) in vec4 instanceTransform2;
layout(location = 5) in vec4 instanceTransform3;
layout(location = 6) in vec4 instanceSection;
layout(location = 7) in vec2 inUv;
layout(location = 8) in float inInner;
layout(location = 0) out vec3 worldPosition;
layout(location = 1) out vec3 worldNormal;
layout(location = 2) out vec3 worldTangent;
layout(location = 3) out vec3 worldBitangent;
layout(location = 4) out vec2 surfaceUv;
layout(location = 5) flat out int endFace;
layout(location = 6) out vec2 physicalUv;
layout(location = 7) flat out int pattern;
layout(push_constant) uniform SupportDraw
{
    mat4 viewProjection;
    // RGB sRGB tint; A metallic.
    vec4 baseColor;
    vec4 cameraExposure;
    vec4 sunDirectionIntensity;
    vec4 surface;
    // x/z Core units per longitudinal/transverse repeat; y material; w SI scale.
    vec4 timber;
} draw;

void main()
{
    mat4 transform = mat4(instanceTransform0, instanceTransform1,
        instanceTransform2, instanceTransform3);
    mat3 basis = mat3(transform);
    vec3 extent = vec3(length(basis[0]), length(basis[1]), length(basis[2]));
    vec3 p = inPosition;
    if (inInner > .5) p.yz *= instanceSection.x;
    vec3 local = p * extent;
    vec4 position = transform * vec4(p, 1);
    worldPosition = position.xyz;
    gl_Position = draw.viewProjection * position;
    vec3 n = normalize(inNormal);
    bool side = abs(n.x) < .5;
    bool circular = instanceSection.y > .5;
    vec3 tangent = side ? vec3(1,0,0) : vec3(0,1,0);
    vec3 across = side ? (abs(n.y) > .5 ? vec3(0,0,-sign(n.y)) : vec3(0,sign(n.z),0))
        : vec3(0,0,sign(n.x));
    physicalUv = vec2(dot(local,tangent), dot(local,across));
    if (side && circular)
    {
        // Interpolate an unwrapped seam coordinate, never atan or per-face
        // projections. The inner wall has the same physical texture density.
        float radiusScale = inInner > .5 ? instanceSection.x : 1;
        physicalUv.y = inUv.y * 3.14159265359 * extent.y * radiusScale * (inInner > .5 ? 1 : -1);
    }
    surfaceUv = physicalUv / draw.timber.xz;
    physicalUv *= draw.timber.w;
    // Inverse transpose handles nonuniform section/connector placement scale.
    worldNormal = normalize(transpose(inverse(basis)) * n);
    worldTangent = normalize(basis * tangent);
    worldBitangent = cross(worldNormal, worldTangent);
    endFace = side ? 0 : 1;
    pattern = int(instanceSection.z + .5);
}
