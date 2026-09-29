#version 450

// Support members share one unit mesh per profile shape and differ only by
// their instance transform. That transform's basis columns already carry the
// member's length and cross-section, so its column lengths are the member's
// object-space extents in Core units. Deriving UVs from those extents is what
// keeps wood grain physically sized: a long post repeats the texture more
// than a short brace instead of stretching one copy from end to end.
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 instanceTransform0;
layout(location = 3) in vec4 instanceTransform1;
layout(location = 4) in vec4 instanceTransform2;
layout(location = 5) in vec4 instanceTransform3;

layout(location = 0) out vec3 worldPosition;
layout(location = 1) out vec3 worldNormal;
// Tangent-space basis in world space, recovered from the instance transform.
layout(location = 2) out vec3 worldTangent;
layout(location = 3) out vec3 worldBitangent;
// U runs along the member's grain, V across it, both in Core units divided by
// the authored texture scale.
layout(location = 4) out vec2 surfaceUv;

layout(push_constant) uniform SupportDraw
{
    mat4 viewProjection;
    // xyz = sRGB timber tint, w unused.
    vec4 baseColor;
    vec4 cameraExposure;
    vec4 sunDirectionIntensity;
    // x = roughness multiplier, y = normal strength,
    // z = environment intensity, w = environment rotation.
    vec4 surface;
    // x = texture scale in Core units per repeat, y = foundation flag.
    vec4 timber;
} draw;

void main()
{
    mat4 transform = mat4(
        instanceTransform0,
        instanceTransform1,
        instanceTransform2,
        instanceTransform3
    );

    // The basis columns are axis * extent, so their lengths recover the
    // member's object-space size without an extra instance attribute.
    vec3 extent = vec3(
        length(transform[0].xyz),
        length(transform[1].xyz),
        length(transform[2].xyz)
    );

    // The mesh is a unit cube/cylinder, so scaling by the recovered extents
    // puts the vertex in the member's own Core-unit object space. UVs come
    // from that space, which is why texture density does not depend on how
    // the mesh was tessellated.
    vec3 localPosition = inPosition * extent;
    vec4 position = transform * vec4(inPosition, 1.0);

    gl_Position = draw.viewProjection * position;
    worldPosition = position.xyz;

    // The basis is orthonormal apart from its per-axis extents, so removing
    // those extents leaves a pure rotation. Handling the non-uniform scale
    // this way keeps normals correct for a long post whose length is far
    // larger than its cross-section.
    vec3 rotation0 = transform[0].xyz / extent.x;
    vec3 rotation1 = transform[1].xyz / extent.y;
    vec3 rotation2 = transform[2].xyz / extent.z;

    vec3 objectNormal = normalize(inNormal);

    // Per-face projection. Side faces (normal along the cross-section axes)
    // take U from the member's longitudinal +X so grain runs along the
    // timber, and V from whichever cross-section axis lies in the face. The
    // end caps are projected across the cross-section instead, which reads as
    // end grain without needing a dedicated end-grain texture.
    bool sideFace = abs(objectNormal.x) < 0.5;
    vec3 grainAxis = sideFace ? vec3(1.0, 0.0, 0.0)
        : vec3(0.0, 1.0, 0.0);
    vec3 acrossAxis = sideFace
        ? (abs(objectNormal.y) > 0.5 ? vec3(0.0, 0.0, 1.0)
                                     : vec3(0.0, 1.0, 0.0))
        : vec3(0.0, 0.0, 1.0);

    worldNormal = normalize(
        rotation0 * objectNormal.x
        + rotation1 * objectNormal.y
        + rotation2 * objectNormal.z);
    worldTangent = normalize(
        rotation0 * grainAxis.x
        + rotation1 * grainAxis.y
        + rotation2 * grainAxis.z);
    worldBitangent = cross(worldNormal, worldTangent);

    float inverseScale = 1.0 / draw.timber.x;
    surfaceUv = vec2(dot(localPosition, grainAxis),
                     dot(localPosition, acrossAxis)) * inverseScale;
}
