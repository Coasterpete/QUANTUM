# Wooden Support Generator M1

> Historical M1 milestone and captures. The line-only renderer, shared square
> profile, absent member roles and interpolated story levels described below
> have been superseded by [Supports M2A](supports-m2a-solid-rendering.md).
> Current generation assigns roles and role-specific sections in every family
> and matches interior stories by actual elevation. Hybrid additionally authors
> orientation/reference, face mounting, SimpleBent/ConnectedTowers, local panel
> choices and optional outer primary supports. Saved M1 geometry remains intact
> on load; explicit regeneration applies current rules.

QUANTUM generates real-world-informed procedural geometry based on observable
timber coaster support construction. The generator produces an editable
`SupportStructure`, not a structural analysis or an exact reproduction of a
manufacturer's engineering. Track Configuration describes the supported track;
the support structural family is an independent choice. In particular, the
Hybrid Timber Lattice can be used below an independently selected steel track.

## Structural families

Each generated run stores a `TimberSupportFamily` in its recipe. Stable textual
identifiers are saved in `.quantum` files. All families use the same sampled
track positions and rider frames, and retain the same authored controls.
Topology rules determine how the bents and neighboring bays are connected.

| Family | Procedural geometry | Observable construction cue |
| --- | --- | --- |
| Traditional Timber Bent | Regular two-post bents, caps and ledgers; M0 cross bracing for low runs and alternating single braces in tall stories | Conventional repeated bent framing with longitudinal ties |
| Modern Twister Timber | Closer bent spacing through direction or bank changes, at least two framing tiers, continuous ties on three post lines, and selected transverse and longitudinal braces | Nearby bents and several track-following horizontal lines form a dense continuous structure even along low curved runs |
| Prefabricated Timber Lattice | Evenly spaced three-post bents, aligned story ledgers, paired transverse panel braces, and braced longitudinal bays in repeating tower modules | Ordered two/three-leg bent construction, regular stacked panels, and longitudinal stays |
| Hybrid Timber Lattice | Broad two-post bents on individual foundations, a raised first-story ledger, upper cap, one repeated diagonal per bent/story, and mostly open longitudinal bays | Simple one-story framed units connect into a deeper lattice beneath independently chosen track |

These post arrangements, bracing patterns, spacing thresholds, spread factors
and story limits are QUANTUM approximations of visible construction language.
The post lanes represent primary framing lines, not an exact count of boards
in a real bent. They are not timber sizing, fastener schedules, foundation
designs or load ratings.

## Recipe and height behavior

The recipe retains start/end station, nominal bent spacing and width,
foundation elevation, attachment vertical offset, square member size and the
longitudinal bracing toggle. M1 adds family and maximum story height. Changing
family in the Editor applies recommended values to spacing, width, timber size
and story height; all remain editable afterward. Modern Twister and Hybrid
families reduce the next spacing to 60% of nominal where sampled tangent or
frame-up direction changes cross their family threshold. This is a geometric
density rule, not a load calculation.

Each bent uses the track rider frame for its upper nodes and the horizontal
track normal for foundation placement. Height above the explicit foundation
plane determines the number of stories. Intermediate levels split long posts,
add transverse framing and connect to neighboring bays. Posts, ledgers, caps
and longitudinal ties form the primary framework; diagonals are secondary
braces. Modern Twister uses a lower and upper framing tier even where its
height alone would require only one story. Hybrid's first-story ledger sits
one quarter of that story above the foundation, so its principal diagonal
starts at the ledger rather than at the footing. The document retains one
rectangular member profile for both roles.
The generator's separate member-addition sites leave a seam for a later solid
renderer without adding a member-role field in M1. A run is rejected if
any top lane reaches the foundation, the tangent has no horizontal direction,
or a bent would need more than 64 stories. The Ground Surface display is not
used as terrain geometry.

## Persistence and regeneration

The complete recipe remains on the generated `SupportStructure`. Documents
written by M0 without `family` or `storyHeight` load as Traditional Timber Bent
with the M0-compatible default story height of 24 Core units. Their saved nodes
and members are loaded unchanged. New generation or explicit regeneration
applies M1's story framing to tall runs. Textual family identifiers are
validated on load; unknown values are rejected. The document format version
is unchanged.

Each run is a separate structure, so one document can contain different
families in different station ranges. Regeneration replaces only the selected
generated structure. Other generated structures and manual structures stay
intact. Manual edits *inside* the regenerated structure are replaced. Generate
and regenerate each enter the existing transaction and Undo/Redo history path.

## Real-world reference notes

- [Great Coasters International service/design material](https://greatcoasters.com/service)
  describes twisted layouts with high-speed direction changes. The
  [Mystic Timbers construction photos](https://coasternation.com/exclusive-photos-details-construction-of-kings-islands-beloved-mystic-timbers/),
  [Gold Striker aerial construction view](https://www.coaster101.com/2013/03/06/gold-striker-construction-from-the-air/)
  and [Gold Striker close views](https://norcalthrills.com/parks/cga/coasters/gold-striker/)
  show closely repeated bents, upright post rhythms and multiple horizontal
  lines following curved track. This informs Modern Twister's station density,
  minimum two-tier framing and longitudinal continuity. Selected diagonals
  leave openings between the repeated frames.
- [Intamin's wooden coaster page](https://www.intamin.com/product/wooden-coaster/)
  establishes the prefabricated wooden-coaster context.
  [Cordes Holzbau's Colossos project](https://www.cordes-holzbau.de/en/projekt/colossos-wooden-roller-coaster-heide-park-soltau/),
  [ZÜBLIN Timber's Colossos project](https://www.zueblin-timber.com/en/projects/colossos-wooden-roller-coaster-soltau)
  and [El Toro construction journal](https://greatadventurehistory.com/ElToro.htm)
  document prepared components and erected two- and three-legged bents. The
  [Balder timber-framing account](https://www.swedishwood.com/inspiration/architecture/balder_rollercoaster/)
  describes stacked transverse trusses, vertical posts split at story levels,
  longitudinal stays and diagonals in selected directions. These observations
  inform Prefabricated's regular story sequence and repeating braced bays;
  its three post lines and brace interval remain simplified QUANTUM choices.
- [Rocky Mountain Construction's hybrid system description](https://rockymtnconstruction.com/hybrid-coasters/)
  explicitly permits wood lattice, steel lattice or columns below steel track.
  [Steel Vengeance](https://rockymtnconstruction.com/roller-coaster/steel-vengeance/),
  [Hersheypark's Wildcat's Revenge account](https://www.hersheypark.com/plan-your-visit/blog/hersheypark-roller-coaster-guide-2026)
  and [Wildcat's Revenge photos](https://themetography.com/photographing-coasters-at-hersheypark-august-2023/)
  show the steel-on-timber context. The user-supplied one-story 3D reference
  directly shows two-post bents on discrete foundations, a raised lower ledger,
  single diagonals with the same direction in neighboring bents, upper caps,
  longitudinal ties and open bays. Repeated bents and stacked stories create
  depth without full X bracing in every opening. This is not proprietary I-Box or
  support engineering.
- Traditional retains M0's simple two-post bent as its baseline. The visible
  repeated bents, caps, ties and selected tall-story diagonals follow
  conventional timber framing language rather than an added visual motif.

## Known limits and later work

The renderer still draws support members as lines while retaining rectangular
member profiles for future solid rendering. M1 does not compute structural
capacity, exact joint details, foundations, terrain following, scenery
clearances, station exclusion or selective preservation of edits within a
regenerated run. Story levels are evenly interpolated between foundation and
track attachment; they are not contractor fabrication levels. Geometry is
validated for document consistency and nondegenerate members only.

The comparison capture manifests disable the environment backdrop so the line
geometry is legible against a plain background. This branch changes no sky,
IBL, shader, cubemap, sampler, HDR upload or exposure implementation. The
black capture background is the capture setting, not a sky-rendering change.

## Windows Editor comparison captures

The first four captures use the same curved, banked ModernSteel track, station
range, recipe dimensions, camera framing and foundation plane. Only the support
family changes. The fifth uses a shorter, taller Traditional run to show its
intermediate stories. The sixth shows a low curved Modern run with the track
about 10–24 Core units above its foundation plane. The seventh isolates a
one-story Hybrid run against the supplied reference. All images were rendered
by the Windows Editor capture harness.

- [Traditional Timber Bent](images/wooden-support-m1/traditional/editor-overview.png)
- [Modern Twister Timber](images/wooden-support-m1/twister/editor-overview.png)
- [Prefabricated Timber Lattice](images/wooden-support-m1/prefabricated/editor-overview.png)
- [Hybrid Timber Lattice](images/wooden-support-m1/hybrid/editor-overview.png)
- [Tall multi-story example](images/wooden-support-m1/tall/editor-overview.png)
- [Low curved Modern Twister](images/wooden-support-m1/twister-low/editor-overview.png)
- [One-story Hybrid Timber](images/wooden-support-m1/hybrid-low/editor-overview.png)
