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

/* ABI 1: 64-bit process, natural 8-byte alignment, UTF-8 strings.
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
QN_API QnResult QN_CALL qn_update(QnSession session, double seconds, QnError* error);
/* Borrowed host: 1=HWND, 2=X11 XID; 0/0 standalone validation window.
   Avalonia positions/sizes the child. frame dimensions are physical pixels. */
QN_API QnResult QN_CALL qn_viewport_open(QnSession session, uint32_t platform, uint64_t window, QnError* error);
/* Busy means the controller is replacing state; retain the previous frame. */
QN_API QnResult QN_CALL qn_viewport_frame(QnSession session, uint32_t width, uint32_t height, QnError* error);
QN_API QnResult QN_CALL qn_viewport_close(QnSession session, QnError* error);
/* Bits: Undo=1, Redo=2, Save=4, Open=8, New=16, SaveAs=32,
   next pane=64, previous pane=128, synchronize host focus=256.
   Consumes pending native-child input intents. */
QN_API QnResult QN_CALL qn_viewport_shortcuts(QnSession session, uint32_t* shortcuts, QnError* error);
/* Display-only: 0 shaded, 1 wireframe, 2 shaded/wireframe, 3 reference curves. */
QN_API QnResult QN_CALL qn_viewport_presentation(QnSession session, uint32_t mode, QnError* error);
/* 1=orbit radians, 2=pan physical pixels, 3=wheel, 4=frame, 5=local move,
   6=focus borrowed child after managed keyboard navigation. */
QN_API QnResult QN_CALL qn_viewport_input(QnSession session, uint32_t action,
    double x, double y, double z, double seconds, QnError* error);
#ifdef __cplusplus
}
#endif
#endif
