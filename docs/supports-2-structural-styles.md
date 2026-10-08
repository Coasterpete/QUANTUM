# Supports 2: structural styles, surfaces, and infrastructure

This increment extends the accepted native graph and Avalonia editor. It does
not change coaster geometry, Train Physics, RideOperations, Track Start, or
the 1/240-second simulation policy. It supersedes the timber-only presentation
and infrastructure-hook limitations described in the earlier milestone docs.

## Audit of the previous pipeline

`AuthoredTrack` owns a value `SupportCollection`. A structure ID is document-wide;
nodes and members share a structure-local element allocator. A connection has
no separate ID: its identity is structure + member + directed end. History
stores authored track values, not renderer resources.

Track attachments store cumulative station and rider-frame offsets in Core
coordinate units. The native centerline resolves their presentation positions.
Foundation and free nodes retain their authored world positions, including
when Track Start moves. Foundations are centered on their nodes; this upgrade
preserves that convention and does not project foundations onto terrain.

`resolveSupportMemberPlacements` already handles physical mounting independently
from logical graph lines. Member-local +X follows length; projected orientation
reference supplies Y; Z completes the right-handed frame. That numerical
placement code remains intact. The box has independent hard face normals;
the 16-segment round mesh has radial side normals and separate cap normals.

The old vertex shader already multiplied unit-mesh positions by actual member
dimensions. The reported wood problem was therefore **not** a general 0–1
stretch over every member. The inspected defects were:

- Every member draw sampled pine, regardless of its profile or intended family.
- Cylinders selected different planar projections around their circumference.
- Opposite box faces used unsigned transverse mapping despite opposite tangent
  handedness, producing inconsistent normal-map orientation.
- Caps reused longitudinal pine grain.
- The 2240 × 1390 pine maps used equal longitudinal/transverse repeat distances,
  treating a non-square source as square.
- The bundled source maps have nonmatching repeated edges; the sampler already
  repeats, so repeating those edges alone does not remove their seams.

The renderer consumed pure solid snapshots as shared box/cylinder meshes plus
instance transforms. VulkanContext owned buffers, VMA allocations, descriptors,
and retirement. Picking consumed native node and member placement metadata;
serialization used strict optional JSON fields; edits used native history.
The older Wooden Run generator already emitted ordinary graph objects. Those
responsibility boundaries remain in place.

## Profiles, families, and material precedence

Profiles are Circular, Rectangular, Hss, and ISection. Circular with a positive
wall uses an actual tube; HSS uses four walls; I/H uses two flanges and a web.
Wall validation requires positive, possible dimensions. Existing Circular and
Rectangular identifiers retain their meaning; a rectangular profile may use
any of the four materials.

Families provide defaults: TraditionalTimber, TubularSteel, BoxSteel,
ISectionSteel, ConcretePier, and Custom. A surface is resolved in this order:
member override → structure override → family default. Legacy timber appearance
is honored when an explicit new surface does not replace it. Foundation
appearance remains independently authored.

Existing profiles remain explicit. New Avalonia-created members opt into family
profiles. Applying member dimensions makes an explicit override; “Use family
profile” opts back in. Generated frames have a separate `useFrameProfile` flag:
recipe dimensions update those members until an explicit profile gesture clears
the flag. An explicit choice equal to the old default remains an override.
Changing family never silently replaces explicit profiles or material overrides.

Surfaces contain sRGB tint, roughness, metallic, normal strength, and two SI
repeat distances. Timber uses the existing pine maps. Painted steel,
galvanized steel, and concrete use separate conservative procedural shading;
they do not sample pine. Normal-map strength currently affects timber detail.
The maintainer's untracked material directories are not adopted or modified.

## Physical UVs and meshes

Longitudinal U = member-local physical X / longitudinal repeat. Box transverse
V uses the signed coordinate consistent with `cross(normal, longitudinal)`.
Caps use section coordinates. Circular walls interpolate a duplicated unwrap
seam over the actual circumference, with opposite orientation for the bore.
The shader converts authored SI repeats to Core units once at the draw boundary.
The default timber transverse repeat preserves the source aspect ratio.

Timber caps have procedural end grain. Offset-map blending softens the existing
pine maps' repeated edges. This is predictable procedural mapping, not a
hand-authored unwrap or a promise of seamless photorealistic lumber. Members
at or below the existing 1e-6 Core-unit presentation cutoff are still skipped.

End caps, plates, flanges/base sleeves, splice straps, saddles/clamps, and
footing treatments now emit bounded geometry in the existing directed-end frame,
including local connector placement. MiteredCut and absent connections retain
their existing behavior. Track/foundation treatment validation remains intact.
Logical GLB references remain saved; procedural treatment geometry does not
load a new connector asset pipeline or model bolts and paired splice joints.

Footing and Block use box bodies; Pedestal uses two tiers inside the authored
bounds; Pier uses a circular column. Explicit foundation dimensions override
the existing derived size. Zero dimensions retain derived sizing.

## Generated frames and infrastructure

`generateSupportFrame` produces four nodes and four ordinary members: two posts,
crossbeam, and diagonal brace, with world foundations and two attached top nodes.
Parameters cover station, width/post spacing, foundation elevation, vertical
offset, three profiles, and footing dimensions. Regeneration retains IDs,
member overrides, and added manual objects. It rejects deleted/replaced recipe
topology instead of treating another object as a generated member. Regeneration
deliberately moves the recipe's four nodes and resets their recipe foundations.
The older Wooden Run remains available and now retains structure style/surface.

Native infrastructure presentation consumes the existing authored infrastructure
collection. Track-relative catwalks, platforms, landings, and guardrails use the
existing evaluated native frame, including pitch, bank, and curvature. Chords
are at most 0.75 m and refine when frames turn substantially. Rail posts are
spaced independently of deck tessellation. Left/right inner/outer edges use the
authored track side. World endpoints remain fixed during Track Start changes.

Stairs use explicit start/end points, level treads, stringers, and optional
handrails. They do not route around scenery or automatically connect a network.
They require horizontally separated endpoints. Budgets are 4096 deck frames,
1024 railing intervals, and 512 treads per object; split oversized intervals.
Grating and chain mesh are shader cutouts in modest sheets, not individual wires.

Access presentation reads existing authored Access dimensions/materials and the
existing runtime fraction by hardware ID. Two stationary posts and a five-part
leaf provide swing/slide presentation. Locking and interlocks remain entirely
in RideOperations. Infrastructure references existing Access IDs. It does not
author a second gate state or automatically cut a rail opening around a gate.

## Editor, ownership, and learning path

The C ABI's existing graph layouts remain intact. Optional bounded JSON snapshots
carry style/provenance display values. Session caches authored support arrays by
revision; Avalonia drafts send commands back to ApplicationService. One native
transaction validates the candidate, records history, and publishes a revision.
Undo restores authored values; selection remains transient and may need reselection
when a deleted object is restored. Renderer caches are never serialized.

NativeViewport rebuilds static infrastructure on document revision changes and
gate meshes when runtime fractions change. VulkanContext combines support,
infrastructure, and Access batches using three shared unit meshes. Candidate
uploads precede publication; old buffers use the existing retirement path.
Vulkan resources remain owned by VulkanContext and are destroyed before SDL's
window. C++ value semantics, `std::optional`, scoped enums, `std::span`, and
explicit ownership are the concepts to understand now. SPIR-V, descriptor,
and synchronization details can wait until the renderer reading step.

Supports preserve native node/member picking priority. Infrastructure and gates
use native transformed-box picking; selected infrastructure gets a bounding
outline. A picked gate opens the existing hardware inspector. Holes in grating,
HSS/I envelopes, and detailed treatments are conservatively picked; foundation
controls remain node-based. The new infrastructure panels and animated solid
presentation are integrated in the Avalonia/native-host path; the historical
ImGui editor does not gain matching new panels or infrastructure publication.

The schema remains version 1 with optional strict fields. This reader loads old
documents, retaining IDs and defaults. Older binaries that reject unknown fields
cannot open documents containing these new features. No saved documents are
automatically rewritten or migrated.

Read now: SupportSurface.hpp; SupportStyles.cpp; SupportFrameGenerator.cpp;
InfrastructurePresentation.cpp. Optional: EditorViewModel.SupportAppearance.cs
and EditorViewModel.Infrastructure.cs. Deep internal: SupportSolidGeometry.cpp,
support.vert/support.frag, and VulkanContext's support upload/draw functions.

## Boundaries for visual acceptance

CPU generation is tested; visual approval remains with the maintainer. These are
engineering approximations: no taper, structural analysis, manufacturer library,
automatic stair routing, automatic gate openings, bolt detail, or connector-GLB
loading. Gate motion currently uploads the combined instance stream; static
catwalk geometry is cached, but GPU frame impact during gate motion was not
benchmarked. Non-timber normal-map assets are not wired in this increment.

The one live smoke completed its functional checks, but had a Vulkan shader
capability error and terminated abnormally during shutdown. The new `discard`
compiled under Vulkan 1.3's default SPIR-V 1.6 requested an unenabled optional
demote feature. Only support.frag now targets SPIR-V 1.5/Vulkan 1.2, valid on the
existing Vulkan 1.3 device, with OpKill. Offline validation and a compiled-binary
regression check verify this. The requirement is documented by the
[Vulkan SPIR-V capability table](https://docs.vulkan.org/spec/latest/appendices/spirvenv.html).
The smoke harness now awaits MainWindow's asynchronous close boundary.
Shutdown's native abort cause remains unconfirmed. No second live smoke was
run; clean runtime shutdown and final visual approval remain unverified.
