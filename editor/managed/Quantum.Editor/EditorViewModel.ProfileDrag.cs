using System.Globalization;
using Quantum.Managed;

namespace Quantum.Editor;

public sealed partial class EditorViewModel
{
    private ProfileSegment? dragTarget;
    private ulong dragToken, dragRevision;
    private long dragGeneration;
    private ProfileOperation dragOperation;
    private Task dragStart = Task.CompletedTask, previewPump = Task.CompletedTask;
    private (double Value, double Snap)? pendingDrag;
    private IReadOnlyList<ProfileSegment>? previewProfiles;
    private bool endingDrag;
    private bool snapEnabled;
    private string distanceSnap = "0.5", valueSnap = "0.1";
    private bool ProfileInteractionActive => dragTarget is not null;
    public bool InteractionActive => ProfileInteractionActive || TransformInteractionActive;
    public event Action? ProfileDragCanceled;
    public IReadOnlyList<ProfileSegment> GraphProfiles => previewProfiles ?? VisibleProfiles;
    public string DragStatus => ProfileInteractionActive ? "Native profile preview · release to commit · Esc cancels"
        : "Circles: value · squares: distance · crossed squares: pinned · wheel: zoom · middle drag: pan";
    public bool SnapEnabled { get => snapEnabled; set { snapEnabled = value; Changed(); } }
    public string DistanceSnap { get => distanceSnap; set { distanceSnap = value; Changed(); } }
    public string ValueSnap { get => valueSnap; set { valueSnap = value; Changed(); } }
    private static bool SameTarget(ProfileSegment? a, ProfileSegment? b) => a is not null && b is not null
        && a.OwnerKind == b.OwnerKind && a.OwnerId == b.OwnerId && a.Channel == b.Channel && a.Id == b.Id;

    public Task BeginProfileDragAsync(ProfileSegment target, ProfileOperation operation)
    {
        if (Busy || InteractionActive || Snapshot is not { } snapshot) return Task.CompletedTask;
        SelectedProfile = VisibleProfiles.FirstOrDefault(p => SameTarget(p, target));
        if (SelectedProfile is null) return Task.CompletedTask;
        Error = ""; dragTarget = SelectedProfile; dragRevision = snapshot.Revision; dragOperation = operation;
        endingDrag = false; pendingDrag = null; dragToken = 0;
        previewPump = Task.CompletedTask;
        var generation = ++dragGeneration;
        NotifyDrag();
        dragStart = StartNativeDrag(target, snapshot.Revision, operation, generation);
        return dragStart;
    }
    private async Task StartNativeDrag(ProfileSegment target, ulong revision, ProfileOperation operation, long generation)
    {
        ulong token = 0;
        try
        {
            await Controller.Submit(s => token = s.BeginProfileDrag(target, revision, operation));
            if (generation != dragGeneration)
            { await Controller.Submit(s => s.FinishProfileDrag(token, false)); return; }
            dragToken = token;
            StartPreviewPump();
        }
        catch (Exception failure)
        {
            if (generation != dragGeneration) return;
            Error = failure.Message;
            await EndProfileDragAsync(false);
        }
    }
    public void QueueProfileDrag(double value)
    {
        if (!ProfileInteractionActive || endingDrag) return;
        var snap = 0.0;
        if (SnapEnabled)
        {
            var text = dragOperation is ProfileOperation.MoveBegin or ProfileOperation.MoveEnd ? DistanceSnap : ValueSnap;
            if (!double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out snap) || !double.IsFinite(snap) || snap <= 0)
            { Error = "Snapping increment must be finite and positive. Drag canceled."; CancelForSelection(); return; }
        }
        pendingDrag = (value, snap);
        StartPreviewPump();
    }
    private void StartPreviewPump()
    {
        // One in-flight native request plus the latest pointer position. Input
        // cannot accumulate an unbounded queue of obsolete preview commands.
        if (dragToken != 0 && pendingDrag is not null && previewPump.IsCompleted)
            previewPump = PumpPreviews(dragGeneration, dragToken);
    }
    private async Task PumpPreviews(long generation, ulong token)
    {
        try
        {
            while (generation == dragGeneration && pendingDrag is { } request)
            {
                pendingDrag = null;
                IReadOnlyList<ProfileSegment>? channel = null;
                await Controller.Submit(s => channel = s.PreviewProfileDrag(token, request.Value, request.Snap));
                if (generation != dragGeneration || dragTarget is not { } target) return;
                // Copied native samples are presentation. The native document
                // snapshot remains unchanged until release commits the gesture.
                previewProfiles = VisibleProfiles.Select(p => p.Channel == target.Channel
                    ? channel!.First(c => c.Id == p.Id) : p).ToArray();
                var candidate = channel!.First(p => p.Id == target.Id);
                ProfileBegin = candidate.ValueBegin.ToString("G12", CultureInfo.CurrentCulture);
                ProfileEnd = candidate.ValueEnd.ToString("G12", CultureInfo.CurrentCulture);
                if (dragOperation is ProfileOperation.MoveBegin or ProfileOperation.MoveEnd)
                    ProfileDistance = (dragOperation == ProfileOperation.MoveBegin ? candidate.Begin : candidate.End).ToString("G12", CultureInfo.CurrentCulture);
                Changed(nameof(GraphProfiles));
            }
        }
        catch (Exception failure)
        {
            if (generation != dragGeneration) return;
            Error = failure.Message;
            await EndProfileDragAsync(false);
        }
    }
    public async Task EndProfileDragAsync(bool commit)
    {
        if (!ProfileInteractionActive) return;
        var generation = dragGeneration;
        if (!commit)
        {
            var token = dragToken;
            ++dragGeneration; ClearDrag(); ProfileDragCanceled?.Invoke();
            // A pending begin cancels its own token when it resumes. Requests
            // already running are ordered before this cancellation by worker.
            if (token != 0)
                try { await Controller.Submit(s => s.FinishProfileDrag(token, false)); }
                catch (Exception failure) { Error = failure.Message; }
            return;
        }
        endingDrag = true;
        await dragStart;
        StartPreviewPump();
        await previewPump;
        if (generation != dragGeneration || !ProfileInteractionActive) return;
        Busy = true;
        try
        {
            var token = dragToken;
            await Controller.Submit(s => s.FinishProfileDrag(token, true));
            ClearDrag();
            if (Controller.Latest is { } latest) Publish(latest);
        }
        catch (Exception failure)
        {
            Error = failure.Message;
            await EndProfileDragAsync(false);
            if (Controller.Latest is { } latest) Publish(latest);
        }
        finally { Busy = false; }
    }
    private void CancelForSelection()
    {
        if (ProfileInteractionActive) _ = EndProfileDragAsync(false);
        if (TransformInteractionActive && !publishingSnapshot) _ = EndTransformAsync(false);
    }
    private void ClearDrag()
    {
        dragTarget = null; dragToken = 0; pendingDrag = null; previewProfiles = null; endingDrag = false;
        SelectedProfile = selectedProfile;
        NotifyDrag(); Changed(nameof(GraphProfiles));
    }
    private void NotifyDrag()
    {
        Changed(nameof(InteractionActive)); Changed(nameof(DragStatus)); Changed(nameof(Activity)); RefreshCommands();
    }
}
