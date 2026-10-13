using System.Globalization;
using Quantum.Managed;

namespace Quantum.Editor;

public sealed partial class EditorViewModel
{
    private string layoutX = "", layoutY = "", layoutZ = "", layoutPitch = "", layoutYaw = "", layoutRoll = "";
    private bool transformActive, endingTransform, publishingSnapshot;
    private ulong transformToken, transformRevision, transformInputId;
    private long transformGeneration;
    private double? pendingTransform;
    private Task transformStart = Task.CompletedTask, transformPump = Task.CompletedTask;
    public bool TransformInteractionActive => transformActive;
    public event Action? TransformCanceled;
    public EditorCommand ApplyTrackStartPositionCommand { get; private set; } = null!;
    public EditorCommand ApplyTrackStartOrientationCommand { get; private set; } = null!;
    public EditorCommand SelectTrackStartCommand { get; private set; } = null!;
    public EditorCommand MoveTrackStartCommand { get; private set; } = null!;
    public EditorCommand RotateTrackStartCommand { get; private set; } = null!;
    public EditorCommand FrameTrackStartCommand { get; private set; } = null!;
    public string LayoutX { get => layoutX; set { layoutX = value; Changed(); } }
    public string LayoutY { get => layoutY; set { layoutY = value; Changed(); } }
    public string LayoutZ { get => layoutZ; set { layoutZ = value; Changed(); } }
    public string LayoutPitch { get => layoutPitch; set { layoutPitch = value; Changed(); } }
    public string LayoutYaw { get => layoutYaw; set { layoutYaw = value; Changed(); } }
    public string LayoutRoll { get => layoutRoll; set { layoutRoll = value; Changed(); } }
    public string TrackStartStatus => TransformInteractionActive ? "Placement preview · release to commit · Esc cancels"
        : Snapshot?.Placement is { Selected: true } p ? $"Track Start selected · {p.Mode}" : "Select Track Start here or its diamond in the viewport";
    public string PlacementPreview => Snapshot?.Placement is { Preview: true } p
        ? $"Preview XYZ: {p.Candidate.X:F3}, {p.Candidate.Y:F3}, {p.Candidate.Z:F3} m" : "";

    private void InitializePlacementCommands()
    {
        ApplyTrackStartPositionCommand = Add(() => ApplyPlacement(false), () => Snapshot is not null);
        ApplyTrackStartOrientationCommand = Add(() => ApplyPlacement(true), () => Snapshot is not null);
        SelectTrackStartCommand = Add(() => ChooseTrackStart(TransformMode.Select));
        MoveTrackStartCommand = Add(() => ChooseTrackStart(TransformMode.Move));
        RotateTrackStartCommand = Add(() => ChooseTrackStart(TransformMode.Rotate));
        FrameTrackStartCommand = Add(() => Run(() => { Controller.Session.Camera(ViewportAction.FrameTrackStart); return Task.CompletedTask; }));
    }
    private static double ParsePlacement(string text, string name)
    {
        if (!double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out var value) || !double.IsFinite(value))
            throw new ArgumentException($"{name} must be a finite number.");
        return value;
    }
    private Task ApplyPlacement(bool orientation) => Run(async () =>
    {
        if (Snapshot is not { } state) return;
        var x = ParsePlacement(orientation ? LayoutPitch : LayoutX, orientation ? "Pitch" : "X");
        var y = ParsePlacement(orientation ? LayoutYaw : LayoutY, orientation ? "Yaw" : "Y");
        var z = ParsePlacement(orientation ? LayoutRoll : LayoutZ, orientation ? "Roll" : "Z");
        await Controller.Submit(s =>
        {
            s.EditTrackStart(state.Revision, orientation, x, y, z);
            s.SelectTrackStart(s.Snapshot().Revision, true, state.Placement.Mode);
        });
    });
    private Task ChooseTrackStart(TransformMode mode) => Run(async () =>
    {
        if (Snapshot is { } state) await Controller.Submit(s => s.SelectTrackStart(state.Revision, true, mode));
    });
    private void PublishPlacement(PlacementSnapshot next, bool changedDocument)
    {
        if (changedDocument)
        {
            LayoutX = next.Authored.X.ToString("G12", CultureInfo.CurrentCulture);
            LayoutY = next.Authored.Y.ToString("G12", CultureInfo.CurrentCulture);
            LayoutZ = next.Authored.Z.ToString("G12", CultureInfo.CurrentCulture);
            LayoutPitch = next.Pitch.ToString("G12", CultureInfo.CurrentCulture);
            LayoutYaw = next.Yaw.ToString("G12", CultureInfo.CurrentCulture);
            LayoutRoll = next.Roll.ToString("G12", CultureInfo.CurrentCulture);
        }
        Changed(nameof(TrackStartStatus)); Changed(nameof(PlacementPreview));
    }
    public Task BeginTransformAsync(TransformMode mode, TransformAxis axis, ulong? revision = null, ulong? selectionRevision = null)
    {
        if (Busy || InteractionActive || Snapshot is not { } state) return Task.CompletedTask;
        Error = ""; transformActive = true; endingTransform = false; transformToken = 0; pendingTransform = null;
        transformRevision = revision ?? state.Revision; transformPump = Task.CompletedTask;
        var generation = ++transformGeneration; NotifyTransform();
        transformStart = StartTransform(mode, axis, transformRevision, selectionRevision ?? state.Placement.SelectionRevision, generation);
        return transformStart;
    }
    private async Task StartTransform(TransformMode mode, TransformAxis axis, ulong revision, ulong selectionRevision, long generation)
    {
        ulong token = 0;
        try
        {
            await Controller.Submit(s => token = s.BeginTransform(revision, selectionRevision, mode, axis));
            if (generation != transformGeneration)
            { await Controller.Submit(s => s.FinishTransform(token, false)); return; }
            transformToken = token; StartTransformPump();
        }
        catch (Exception failure)
        { if (generation == transformGeneration) { Error = failure.Message; await EndTransformAsync(false); } }
    }
    public void QueueTransform(double amount)
    {
        if (!transformActive || endingTransform) return;
        pendingTransform = amount; StartTransformPump();
    }
    private void StartTransformPump()
    {
        if (transformToken != 0 && pendingTransform is not null && transformPump.IsCompleted)
            transformPump = PumpTransforms(transformGeneration, transformToken);
    }
    private async Task PumpTransforms(long generation, ulong token)
    {
        try
        {
            while (generation == transformGeneration && pendingTransform is { } amount)
            {
                pendingTransform = null;
                await Controller.Submit(s => s.PreviewTransform(token, amount));
            }
        }
        catch (Exception failure)
        { if (generation == transformGeneration) { Error = failure.Message; await EndTransformAsync(false); } }
    }
    public async Task EndTransformAsync(bool commit)
    {
        if (!transformActive) return;
        var generation = transformGeneration;
        if (!commit)
        {
            var token = transformToken; ++transformGeneration; ClearTransform(); TransformCanceled?.Invoke();
            if (token != 0)
                try { await Controller.Submit(s => s.FinishTransform(token, false)); }
                catch (Exception failure) { Error = failure.Message; }
            return;
        }
        if (endingTransform) return;
        endingTransform = true;
        await transformStart; StartTransformPump(); await transformPump;
        if (generation != transformGeneration || !transformActive) return;
        Busy = true;
        try
        {
            await Controller.Submit(s => s.FinishTransform(transformToken, true));
            ClearTransform(); if (Controller.Latest is { } latest) Publish(latest);
        }
        catch (Exception failure)
        { Error = failure.Message; await EndTransformAsync(false); if (Controller.Latest is { } latest) Publish(latest); }
        finally { Busy = false; }
    }
    public async Task ReceiveTransformIntentAsync(TransformIntent intent)
    {
        if (intent.Phase == 3)
        { if (intent.Id == transformInputId) await EndTransformAsync(false); return; }
        if (intent.Id != transformInputId)
        {
            transformInputId = intent.Id;
            if (Busy || InteractionActive) { TransformCanceled?.Invoke(); return; }
            _ = BeginTransformAsync((TransformMode)intent.Mode, (TransformAxis)intent.Axis, intent.Revision, intent.SelectionRevision);
        }
        QueueTransform(intent.Amount);
        if (intent.Phase == 2) await EndTransformAsync(true);
        else await transformStart;
    }
    private void ClearTransform()
    { transformActive = false; transformToken = 0; pendingTransform = null; endingTransform = false; NotifyTransform(); }
    private void NotifyTransform()
    { Changed(nameof(InteractionActive)); Changed(nameof(TransformInteractionActive)); Changed(nameof(Activity)); Changed(nameof(TrackStartStatus)); RefreshCommands(); }
}
