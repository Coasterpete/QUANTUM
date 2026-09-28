# Wooden Support Generator M0

The Supports workspace can generate a wooden run over an authored track
station interval. Stations and dimensions use Core coordinate units. The
recipe sets bent spacing and width, a flat foundation elevation, a vertical
offset from the track reference, square timber size, and optional diagonal
longitudinal bracing.

Each bent has two foundation nodes, two track-attached upper nodes, two
posts, an upper cross member, and two diagonal members. Consecutive bents
connect along both upper and foundation edges. Optional diagonal members
brace each longitudinal bay. Upper nodes follow the track's banked rider
frame; the foundations use a horizontal plan-view lateral. The full run is
one `SupportStructure`, with the recipe stored as provenance in the document.

Selecting a generated structure and pressing **Regenerate Selected Wooden
Run** replaces that structure as one Undo/Redo edit. With any other selection,
**Generate New Wooden Run** adds a separate structure. Manual structures are
never selected for replacement. Regeneration replaces manual edits made
inside the selected generated structure; M0 does not merge them.

The generator produces a structural layout for visualization and editing. It
does not calculate loads, choose structurally adequate sizes, follow terrain,
avoid scenery, exclude stations, or generate manufacturer-specific styles.
The foundation elevation is an explicit plane and does not use Ground
Surface appearance. The range must fit the authored track attachment domain,
contain at least two bents, and put all upper points above the foundation
plane. Near-vertical track tangents are rejected because a horizontal bent
axis is undefined there.

## Editor captures

- [Overall curved run](images/wooden-support-m0/editor-overview.png)
- [Connected bents and longitudinal bracing](images/wooden-support-m0/modern-steel.png)
- [Curved, banked attachment segment](images/wooden-support-m0/track-start-gizmo.png)
