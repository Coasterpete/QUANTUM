using Avalonia.Controls.ApplicationLifetimes;
using Quantum.Managed;
using System.Diagnostics;
using System.Globalization;
using System.Text.Json;

namespace Quantum.Editor;

internal static partial class RuntimeChecks
{
    // One bounded integration smoke, synthetic input only. Physical mouse
    // feel, hit targets and visual approval remain the maintainer's decision.
    public static async Task RunPlacementSmoke(MainWindow window, string fixture, string report,
        IClassicDesktopStyleApplicationLifetime desktop)
    {
        var checks = new List<string>(); var watch = Stopwatch.StartNew(); string? failure = null;
        var model = window.Model;
        void Require(bool condition, string name)
        { if (!condition) throw new InvalidOperationException($"{name}: {model.Error}"); checks.Add(name); }
        async Task Command(EditorCommand command, string name)
        { Require(command.CanExecute(null), $"{name} enabled"); await command.ExecuteAsync(); Require(model.Error.Length == 0, name); }
        async Task Settle()
        { await Task.Delay(200); await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!); }
        try
        {
            await Task.Delay(300);
            Require(window.HostedViewport.IsReady && window.HostedViewport.FrameCount > 0, "Embedded Vulkan presents");
            Require(window.Icon is not null && window.TimelineBounds.Height >= 200, "Canonical icon and Timeline retained");
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            model.PickFile = _ => Task.FromResult<string?>(fixture);
            await Command(model.OpenCommand, "Open real native document");
            var original = model.Snapshot!.Placement.Authored;
            model.LayoutZ = (original.Z + 15).ToString("G12", CultureInfo.CurrentCulture);
            await Command(model.ApplyTrackStartPositionCommand, "Raise numerically");
            await Command(model.UndoCommand, "Placement Undo"); await Command(model.RedoCommand, "Placement Redo");
            await Command(model.SelectTrackStartCommand, "Select Track Start");
            await Command(model.FrameTrackStartCommand, "Frame Track Start"); await Settle();
            var session = model.Controller.Session;
            session.SelectTrackStart(model.Snapshot!.Revision, false);
            await Settle(); session.Camera(ViewportAction.Focus);
            session.Camera(ViewportAction.TransformDown, .5, .5);
            session.Camera(ViewportAction.TransformUp, .5, .5);
            await Settle();
            Require(model.Snapshot!.Placement.Selected, "Native marker picking synchronizes Inspector");
            await Command(model.MoveTrackStartCommand, "Activate Move"); await Settle();
            var height = window.HostedViewport.Bounds.Height;
            var width = window.HostedViewport.Bounds.Width;
            var beforeMove = model.Snapshot!.Placement.Authored;
            session.Camera(ViewportAction.TransformDown, .5, .5 - 45 / height);
            session.Camera(ViewportAction.TransformMove, .5, .5 - 65 / height);
            await Settle();
            Require(model.Snapshot!.Placement.Preview && model.Snapshot.Placement.Authored == beforeMove,
                "Native Z handle shows staged preview while authored pose stays fixed");
            session.Camera(ViewportAction.TransformUp, .5, .5 - 65 / height);
            await Settle();
            Require(!model.InteractionActive && model.Snapshot!.Placement.Authored.Z > beforeMove.Z, "Native Z handle release raises track");
            await Command(model.RotateTrackStartCommand, "Activate Rotate");
            await Command(model.FrameTrackStartCommand, "Frame relocated anchor"); await Settle();
            var beforeRotate = model.Snapshot!.Placement.Authored;
            session.Camera(ViewportAction.TransformDown, .5 + 64 / width, .5);
            session.Camera(ViewportAction.TransformMove, .5 + 64 * Math.Cos(.08) / width, .5 + 64 * Math.Sin(.08) / height);
            session.Camera(ViewportAction.TransformUp, .5 + 64 * Math.Cos(.08) / width, .5 + 64 * Math.Sin(.08) / height);
            await Settle();
            Require(!model.InteractionActive && model.Snapshot!.Placement.Authored != beforeRotate, "Native Rotate control commits placement");
            Require(model.Snapshot.Available && model.Error.Length == 0 && window.HostedViewport.FrameCount > 5,
                "Train remains available and Vulkan continues presenting after relocation");
        }
        catch (Exception error) { failure = error.ToString(); }
        finally
        {
            await model.EndTransformAsync(false);
            await File.WriteAllTextAsync(report, JsonSerializer.Serialize(new
            {
                passed = failure is null, checks, elapsed_ms = watch.Elapsed.TotalMilliseconds,
                viewport_frames = window.HostedViewport.FrameCount,
                physical_pointer_testing = false, maintainer_visual_approval = "pending", failure
            }, new JsonSerializerOptions { WriteIndented = true }));
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            window.Close(); await Task.Delay(150); desktop.Shutdown(failure is null ? 0 : 1);
        }
    }
}
