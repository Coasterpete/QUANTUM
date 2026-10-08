namespace Quantum.Managed;

public enum TransformMode : uint { Select, Move, Rotate }
public enum TransformAxis : uint { X, Y, Z }
public sealed record TrackStartPose(double X, double Y, double Z, double Qw, double Qx, double Qy, double Qz);
public sealed record PlacementSnapshot(ulong Revision, ulong SelectionRevision, ulong PresentationRevision,
    bool Selected, TransformMode Mode, bool Active, bool Preview, TrackStartPose Authored, TrackStartPose Candidate,
    double Pitch, double Yaw, double Roll);

public sealed unsafe partial class Session
{
    private PlacementSnapshot ReadPlacement()
    {
        NativePlacement state = new() { Size = (uint)sizeof(NativePlacement), Version = 1 };
        Check(Native.qn_placement_snapshot(handle, ref state, out var error), error);
        static TrackStartPose Map(NativePose p) => new(p.X, p.Y, p.Z, p.Qw, p.Qx, p.Qy, p.Qz);
        return new(state.Revision, state.SelectionRevision, state.PresentationRevision, state.Selected != 0,
            (TransformMode)state.Mode, state.Active != 0, state.Preview != 0, Map(state.Authored), Map(state.Candidate),
            state.Pitch, state.Yaw, state.Roll);
    }
    public void EditTrackStart(ulong revision, bool orientation, double x, double y, double z)
    { RequireOpen(); Check(Native.qn_placement_edit(handle, revision, orientation ? 1u : 0u, x, y, z, out var error), error); }
    public void SelectTrackStart(ulong revision, bool selected, TransformMode mode = TransformMode.Select)
    { RequireOpen(); Check(Native.qn_track_start_select(handle, revision, selected ? 1u : 0u, (uint)mode, out var error), error); }
    public ulong BeginTransform(ulong revision, ulong selectionRevision, TransformMode mode, TransformAxis axis)
    {
        RequireOpen();
        NativeTransformCommand command = new() { Revision = revision, SelectionRevision = selectionRevision, Mode = (uint)mode, Axis = (uint)axis };
        Check(Native.qn_transform_begin(handle, in command, out var token, out var error), error); return token;
    }
    public void PreviewTransform(ulong token, double amount)
    { RequireOpen(); Check(Native.qn_transform_preview(handle, token, amount, out var error), error); }
    public void FinishTransform(ulong token, bool commit)
    { RequireOpen(); Check(Native.qn_transform_finish(handle, token, commit ? 1u : 0u, out var error), error); }
    public TransformIntent TakeTransformIntent()
    {
        RequireOpen(); var result = Native.qn_viewport_transform_intent(handle, out var intent, out var error);
        if (result == 6) return default;
        Check(result, error); return intent;
    }
    public void CancelViewportTransform()
    { RequireOpen(); Check(Native.qn_viewport_cancel_transform(handle, out var error), error); }
}
