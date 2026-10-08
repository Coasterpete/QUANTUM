#ifndef QUANTUM_NATIVE_H
#define QUANTUM_NATIVE_H
#include <stdint.h>
#if defined(_WIN32)
#if defined(QUANTUM_NATIVE_EXPORTS)
#define QN_API __declspec(dllexport)
#else
#define QN_API __declspec(dllimport)
#endif
#define QN_CALL __cdecl
#else
#define QN_API __attribute__((visibility("default")))
#define QN_CALL
#endif
#ifdef __cplusplus
extern "C" {
#endif

/* ABI 4: 64-bit process, natural 8-byte alignment, UTF-8 strings.
   Tokens are non-reused IDs, never addresses. All buffers are caller-owned.
   Calls are serialized per session; viewport calls require its UI thread. */
typedef uint64_t QnSession;
typedef int32_t QnResult;
enum { QN_OK, QN_INVALID_HANDLE, QN_INVALID_ARGUMENT, QN_BUFFER_TOO_SMALL,
    QN_IO_ERROR, QN_UNAVAILABLE, QN_BUSY, QN_INTERNAL_ERROR, QN_CONFLICT };
typedef struct QnError { char message[512]; } QnError;
typedef struct QnRegion {
    uint32_t index, kind, construction, reserved;
    double start_station, length, end_station;
    double roll_degrees, pitch_degrees, yaw_degrees;
    double start_x, start_y, start_z, end_x, end_y, end_z;
} QnRegion;
typedef struct QnSnapshot {
    uint32_t size, version;
    uint64_t revision, tick;
    uint32_t region_count, sample_count, car_count, playback;
    uint32_t available, dirty, can_undo, can_redo;
    double speed, station, initial_speed, train_mass, total_length, physics_ms;
} QnSnapshot;
enum { QN_PLAY = 1, QN_PAUSE, QN_RESET, QN_SET_INITIAL_SPEED, QN_UNDO, QN_REDO, QN_SET_REGION_LENGTH };
typedef struct QnCommand {
    uint32_t type, region_index;
    uint64_t expected_revision;
    double value;
} QnCommand;

/* Position is SI metres; orientation is the canonical native quaternion.
   Inspector angles are degrees, Z(yaw)*Y(pitch)*X(roll), converted natively.
   Positive pitch turns +X toward -Z. Candidate is transient, never saved. */
typedef struct QnPose { double x, y, z, qw, qx, qy, qz; } QnPose;
typedef struct QnPlacementSnapshot {
    uint32_t size, version;
    uint64_t revision, selection_revision, presentation_revision;
    uint32_t selected, mode, active, preview;
    QnPose authored, candidate;
    double pitch, yaw, roll;
} QnPlacementSnapshot;
typedef struct QnTransformCommand {
    uint64_t expected_revision, selection_revision;
    uint32_t target, mode, axis, reserved; /* target 0 Track Start; mode 1 Move, 2 Rotate; axis 0/1/2 X/Y/Z */
} QnTransformCommand;
typedef struct QnTransformIntent {
    uint64_t id, revision, selection_revision;
    uint32_t phase, target, mode, axis; /* phase 0 none, 1 preview, 2 release, 3 cancel */
    double amount; /* cumulative SI metres or radians */
} QnTransformIntent;
QN_API QnResult QN_CALL qn_placement_snapshot(QnSession session, QnPlacementSnapshot* snapshot, QnError* error);
/* operation 0 position XYZ metres; 1 orientation Pitch/Yaw/Roll degrees. */
QN_API QnResult QN_CALL qn_placement_edit(QnSession session, uint64_t revision,
    uint32_t operation, double x, double y, double z, QnError* error);
QN_API QnResult QN_CALL qn_track_start_select(QnSession session, uint64_t revision,
    uint32_t selected, uint32_t mode, QnError* error);
QN_API QnResult QN_CALL qn_transform_begin(QnSession session, const QnTransformCommand* command, uint64_t* token, QnError* error);
QN_API QnResult QN_CALL qn_transform_preview(QnSession session, uint64_t token, double amount, QnError* error);
QN_API QnResult QN_CALL qn_transform_finish(QnSession session, uint64_t token, uint32_t commit, QnError* error);
/* UI-thread input is coalesced to the latest cumulative proposal plus its
   terminal phase. Even an entire drag in one frame retains a single ID. */
QN_API QnResult QN_CALL qn_viewport_transform_intent(QnSession session, QnTransformIntent* intent, QnError* error);
QN_API QnResult QN_CALL qn_viewport_cancel_transform(QnSession session, QnError* error);

/* Authored workspace values. Curves are sampled natively for display only;
   edits always change analytic profiles, never these samples. */
typedef struct QnProfileSegment {
    uint32_t owner_kind, channel, segment_id, transition_type, unit, reserved;
    uint64_t owner_id;
    double begin_meters, end_meters, value_begin, value_end;
    double samples[33];
} QnProfileSegment;
typedef struct QnDevice {
    uint64_t id;
    uint32_t kind, enabled, custom, reserved;
    double start_station, end_station, acceleration, maximum_force;
    char name[128];
} QnDevice;
typedef struct QnAuthoringSnapshot {
    uint32_t size, version;
    uint64_t revision;
    uint32_t profile_count, device_count;
} QnAuthoringSnapshot;
typedef struct QnTransitionType { uint32_t type; char name[64]; } QnTransitionType;
typedef struct QnProfileCommand {
    uint32_t owner_kind, channel, segment_id, operation, transition_type, reserved;
    uint64_t owner_id, expected_revision;
    double value;
} QnProfileCommand;

/* Support distances are SI metres at this boundary. IDs retain the native
   uint32_t structure/local-element namespaces. Connection treatment is
   0=None, 1..9=native treatments; flags: placement=1, mounting=2,
   asset display truncated=4. Commands preserve unedited native metadata. */
typedef struct QnSupportStructure {
    uint32_t id, node_count, member_count, generated;
    char name[256];
} QnSupportStructure;
typedef struct QnSupportNode {
    uint32_t structure_id, id, anchor, reserved; /* 0 free, 1 track, 2 foundation */
    double x, y, z, resolved_x, resolved_y, resolved_z, station, lateral, vertical;
} QnSupportNode;
typedef struct QnSupportConnection {
    uint32_t treatment, flags;
    char asset[512];
} QnSupportConnection;
typedef struct QnSupportMember {
    uint32_t structure_id, id, start_node, end_node, shape, role, orientation, has_reference;
    double width, height, wall;
    QnSupportConnection start_connection, end_connection;
} QnSupportMember;
typedef struct QnSupportSnapshot {
    uint32_t size, version;
    uint64_t revision, selection_revision, presentation_revision;
    uint32_t structure_count, node_count, member_count, active;
    uint32_t selection_kind, selection_structure, selection_element, reserved;
} QnSupportSnapshot;
typedef struct QnSupportCommand {
    uint64_t expected_revision;
    uint32_t operation, structure_id, element_id, start_node, end_node, shape, member_end, treatment;
    double x, y, z, width, height, wall;
} QnSupportCommand;
enum { QN_SUPPORT_CREATE_STRUCTURE, QN_SUPPORT_DELETE_STRUCTURE, QN_SUPPORT_RENAME_STRUCTURE,
    QN_SUPPORT_CREATE_NODE, QN_SUPPORT_DELETE_NODE, QN_SUPPORT_MOVE_NODE,
    QN_SUPPORT_SET_ATTACHMENT, QN_SUPPORT_CLEAR_ATTACHMENT, QN_SUPPORT_SET_FOUNDATION,
    QN_SUPPORT_CLEAR_FOUNDATION, QN_SUPPORT_CREATE_MEMBER, QN_SUPPORT_DELETE_MEMBER,
    QN_SUPPORT_SET_PROFILE, QN_SUPPORT_SET_TREATMENT, QN_SUPPORT_CLEAR_CONNECTION,
    QN_SUPPORT_SET_ASSET, QN_SUPPORT_CLEAR_PLACEMENT };

QN_API QnResult QN_CALL qn_support_snapshot(QnSession session, QnSupportSnapshot* summary,
    QnSupportStructure* structures, uint32_t structure_capacity,
    QnSupportNode* nodes, uint32_t node_capacity, QnSupportMember* members,
    uint32_t member_capacity, uint64_t known_revision, QnError* error);
/* text is a UTF-8 structure name or logical connector asset; may be null.
   For SetAttachment x/y/z mean station/lateral/vertical. Returns allocated ID. */
QN_API QnResult QN_CALL qn_support_command(QnSession session, const QnSupportCommand* command,
    const char* text, uint32_t* created, QnError* error);
/* Selection kind: 0 clear, 1 structure, 2 node, 3 member. Transient only. */
QN_API QnResult QN_CALL qn_select_support(QnSession session, uint64_t expected_revision,
    uint32_t kind, uint32_t structure, uint32_t element, QnError* error);
QN_API QnResult QN_CALL qn_support_workspace(QnSession session, uint32_t active, QnError* error);

QN_API uint32_t QN_CALL qn_abi_version(void);
QN_API QnResult QN_CALL qn_create(QnSession* session, QnError* error);
/* Busy while a viewport exists. Close it before releasing its platform host. */
QN_API QnResult QN_CALL qn_destroy(QnSession session, QnError* error);
QN_API QnResult QN_CALL qn_new(QnSession session, QnError* error);
QN_API QnResult QN_CALL qn_open(QnSession session, const char* path, QnError* error);
QN_API QnResult QN_CALL qn_save(QnSession session, const char* path, QnError* error);
/* On insufficient capacity fills only summary. When known_revision matches,
   only summary is copied; Region storage is untouched. 0 forces a Route copy.
   Index plus revision identifies a Region without changing the saved schema. */
QN_API QnResult QN_CALL qn_snapshot(QnSession session, QnSnapshot* summary,
    QnRegion* regions, uint32_t capacity, uint64_t known_revision, QnError* error);
QN_API QnResult QN_CALL qn_command(QnSession session, const QnCommand* command, QnError* error);
/* Owner: 0=Region index, 1=stable Track Device ID. Channel 0/1/2 maps the
   native Roll/Pitch/Yaw rows; device acceleration uses channel 3.
   Units: 0=deg/m, 1=G, 2=m/s^2. Cached per authored revision. */
QN_API QnResult QN_CALL qn_authoring_snapshot(QnSession session, QnAuthoringSnapshot* summary,
    QnProfileSegment* profiles, uint32_t profile_capacity,
    QnDevice* devices, uint32_t device_capacity, uint64_t known_revision, QnError* error);
QN_API QnResult QN_CALL qn_transition_types(QnTransitionType* types, uint32_t capacity,
    uint32_t* count, QnError* error);
/* Operations: 0 Begin value, 1 End value, 2 type, 3 split at local metres,
   4 remove, 5 move Begin boundary, 6 move End boundary. */
QN_API QnResult QN_CALL qn_profile_command(QnSession session, const QnProfileCommand* command, QnError* error);
QN_API QnResult QN_CALL qn_device_acceleration(QnSession session, uint64_t device,
    uint64_t expected_revision, uint32_t custom, double acceleration, QnError* error);
/* Preview is native analytical data only. Document revision/history and the
   simulation stay unchanged until finish(commit=1). Cancel is idempotent;
   tokens prevent late input from touching a later gesture. snap=0 disables
   snapping; otherwise it is in the displayed unit (metres for boundaries). */
QN_API QnResult QN_CALL qn_profile_drag_begin(QnSession session, const QnProfileCommand* command,
    uint64_t* token, QnError* error);
QN_API QnResult QN_CALL qn_profile_drag_preview(QnSession session, uint64_t token,
    double value, double snap, QnProfileSegment* profiles, uint32_t capacity,
    uint32_t* count, QnError* error);
QN_API QnResult QN_CALL qn_profile_drag_finish(QnSession session, uint64_t token,
    uint32_t commit, QnError* error);
QN_API QnResult QN_CALL qn_device_create(QnSession session, uint64_t expected_revision,
    uint32_t kind, uint64_t* device, QnError* error);
QN_API QnResult QN_CALL qn_device_delete(QnSession session, uint64_t expected_revision,
    uint64_t device, QnError* error);
QN_API QnResult QN_CALL qn_device_properties(QnSession session, uint64_t expected_revision,
    uint64_t device, uint32_t enabled, double start, double end, double force, QnError* error);
/* Transient viewport selection, zero clears. No authored/history changes. */
QN_API QnResult QN_CALL qn_select_device(QnSession session, uint64_t expected_revision,
    uint64_t device, QnError* error);
QN_API QnResult QN_CALL qn_update(QnSession session, double seconds, QnError* error);
/* Borrowed host: 1=HWND, 2=X11 XID; 0/0 standalone validation window.
   Avalonia positions/sizes the child. frame dimensions are physical pixels. */
QN_API QnResult QN_CALL qn_viewport_open(QnSession session, uint32_t platform, uint64_t window, QnError* error);
/* Busy means the controller is replacing state; retain the previous frame. */
QN_API QnResult QN_CALL qn_viewport_frame(QnSession session, uint32_t width, uint32_t height, QnError* error);
QN_API QnResult QN_CALL qn_viewport_close(QnSession session, QnError* error);
/* Bits: Undo=1, Redo=2, Save=4, Open=8, New=16, SaveAs=32,
   next pane=64, previous pane=128, synchronize host focus=256, cancel=512.
   Consumes pending native-child input intents. */
QN_API QnResult QN_CALL qn_viewport_shortcuts(QnSession session, uint32_t* shortcuts, QnError* error);
/* Display-only: 0 shaded, 1 wireframe, 2 shaded/wireframe, 3 reference curves. */
QN_API QnResult QN_CALL qn_viewport_presentation(QnSession session, uint32_t mode, QnError* error);
/* 1=orbit radians, 2=pan physical pixels, 3=wheel, 4=frame, 5=local move,
   6=focus borrowed child after managed keyboard navigation, 7=frame support selection,
   8=select support at normalized x/y (same native picking as SDL clicks),
   9=frame Track Start; 10/11/12=normalized transform down/move/up,
   13=cancel pointer input (same path used by SDL and bounded smoke). */
QN_API QnResult QN_CALL qn_viewport_input(QnSession session, uint32_t action,
    double x, double y, double z, double seconds, QnError* error);
#ifdef __cplusplus
}
#endif
#endif
