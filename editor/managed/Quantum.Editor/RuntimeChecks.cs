using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.ApplicationLifetimes;
using System.Diagnostics;
using System.Globalization;
using System.Text.Json;

namespace Quantum.Editor;

// Opt-in live application regression check. It exercises the real commands,
// controller and child host. It cannot approve visuals or physical input/DPI.
internal static class RuntimeChecks
{
    public static async Task Run(MainWindow window, string fixture, string report,
        IClassicDesktopStyleApplicationLifetime desktop)
    {
        var checks = new List<string>();
        var measurements = new Dictionary<string, double>();
        string? failure = null;
        var renderScaling = window.RenderScaling;
        var model = window.Model;
        var temporary = Path.Combine(Path.GetTempPath(), $"quantum-editor-{Guid.NewGuid():N}.quantum");
        model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
        void Require(bool condition, string name)
        {
            if (!condition) throw new InvalidOperationException($"{name}: {model.Error}");
            checks.Add(name);
        }
        async Task Command(EditorCommand command, string name)
        {
            Require(command.CanExecute(null), $"{name} enabled");
            await command.ExecuteAsync();
            Require(model.Error.Length == 0, name);
        }
        try
        {
            await Task.Delay(400);
            renderScaling = window.RenderScaling;
            Require(window.HostedViewport.IsReady && window.HostedViewport.FrameCount > 0, "Embedded Vulkan startup");
            Require(window.Icon is not null && window.TimelineBounds.Height >= 200, "Canonical icon and first-class Timeline pane");
            model.PickFile = _ => Task.FromResult<string?>(fixture);
            var watch = Stopwatch.StartNew();
            await Command(model.OpenCommand, "Open real document");
            measurements["open_ms"] = watch.Elapsed.TotalMilliseconds;
            Require(model.Snapshot!.Route.Count > 0, "Native Route populated");
            var steadyFrames = window.HostedViewport.FrameCount;
            var steadyWatch = Stopwatch.StartNew();
            await Task.Delay(1000);
            measurements["steady_viewport_interval_ms"] = steadyWatch.Elapsed.TotalMilliseconds
                / (window.HostedViewport.FrameCount - steadyFrames);
            watch.Restart();
            for (var i = 0; i < 1000; ++i)
                model.Selected = model.Snapshot.Route[i % model.Snapshot.Route.Count];
            measurements["route_selection_ms"] = watch.Elapsed.TotalMilliseconds / 1000;
            measurements["route_count"] = model.Snapshot.Route.Count;
            model.Selected = model.Snapshot.Route[Math.Min(1, model.Snapshot.Route.Count - 1)];
            Require(model.HasSelection && model.Length.Length > 0, "Route selection populates Inspector");
            Require(model.HasProfile && model.VisibleProfiles.Count > 0, "Native Timeline profiles populated");
            var originalProfileEnd = model.SelectedProfile!.ValueEnd;
            model.ProfileEnd = (originalProfileEnd + .01).ToString("G12", CultureInfo.CurrentCulture);
            await Command(model.ApplyProfileEndCommand, "Accepted Transition Editor endpoint edit");
            Require(Math.Abs(model.SelectedProfile!.ValueEnd - originalProfileEnd - .01) < 1e-10, "Native profile snapshot updated");
            await Command(model.UndoCommand, "Undo profile edit");
            Require(!model.Snapshot!.Dirty && model.SelectedProfile!.ValueEnd == originalProfileEnd, "Profile history restored baseline");
            model.SelectedTransition = model.TransitionTypes.Last();
            await Command(model.ApplyTransitionCommand, "Apply existing transition type");
            Require(model.SelectedProfile!.TransitionType == model.TransitionTypes.Last().Type, "Transition catalog roundtrip");
            await Command(model.UndoCommand, "Undo transition type");
            await Command(model.SplitProfileCommand, "Split native profile");
            await Command(model.UndoCommand, "Undo native split");
            if (model.Snapshot!.Devices.Count > 0)
            {
                model.TimelineSource = 1;
                model.DeviceMode = model.SelectedDevice!.Custom ? 0 : 1;
                await Command(model.ApplyDeviceCommand, "Switch Constant/Custom acceleration");
                Require(model.SelectedDevice!.Custom == (model.DeviceMode == 1), "Device authored mode snapshot");
                await Command(model.UndoCommand, "Undo device mode");
                model.TimelineSource = 0;
            }
            var originalLength = model.Selected.Length;
            model.Length = (originalLength + .25).ToString("G12", CultureInfo.CurrentCulture);
            watch.Restart();
            await Command(model.ApplyLengthCommand, "Accepted Inspector length edit");
            measurements["edit_ms"] = watch.Elapsed.TotalMilliseconds;
            Require(model.Snapshot!.Dirty && Math.Abs(model.Selected!.Length - originalLength - .25) < 1e-8, "Edited snapshot and dirty state");
            await Command(model.UndoCommand, "Undo");
            Require(!model.Snapshot!.Dirty && model.Selected!.Length == originalLength, "Undo restored baseline");
            await Command(model.RedoCommand, "Redo");
            model.PickFile = _ => Task.FromResult<string?>(temporary);
            await Command(model.SaveAsCommand, "Save As");
            Require(File.Exists(temporary) && !model.Snapshot!.Dirty, "Saved exact revision");
            await Command(model.SaveCommand, "Save to current path");
            var beforeFailedOpen = model.Snapshot.Revision;
            model.PickFile = _ => Task.FromResult<string?>(temporary + ".missing");
            await model.OpenCommand.ExecuteAsync();
            Require(model.Error.Length != 0 && model.Snapshot.Revision == beforeFailedOpen, "Failed Open preserves valid document");
            model.PickFile = _ => Task.FromResult<string?>(temporary);
            await Command(model.NewCommand, "New");
            Require(!model.Snapshot!.CanUndo && !model.Snapshot.Dirty, "New resets document/history");
            await Command(model.OpenCommand, "Reopen saved edit");
            await Command(model.PlayCommand, "Play");
            await Task.Delay(250);
            await Command(model.PauseCommand, "Pause");
            Require(model.Snapshot!.Tick > 0 && model.Snapshot.Playback == 2, "Native simulation telemetry");
            await Command(model.ResetCommand, "Reset");
            Require(model.Snapshot!.Tick == 0 && model.Snapshot.Playback == 0, "Reset native preview");
            var frames = window.HostedViewport.FrameCount;
            var viewportWatch = Stopwatch.StartNew();
            var width = window.Width; var height = window.Height;
            for (var i = 0; i < 8; ++i)
            {
                window.Width = width - (i % 3) * 50;
                window.Height = height - (i % 2) * 50;
                await Task.Delay(100);
            }
            window.Width = width; window.Height = height;
            Require(window.HostedViewport.FrameCount > frames, "Repeated live resize");
            Require(window.TimelineBounds.Height >= 200, "Timeline survives workbench resize");
            measurements["viewport_interval_ms"] = viewportWatch.Elapsed.TotalMilliseconds / (window.HostedViewport.FrameCount - frames);
            window.WindowState = WindowState.Minimized;
            await Task.Delay(120);
            window.WindowState = WindowState.Normal;
            var position = window.Position;
            window.Position = new PixelPoint(position.X + 24, position.Y + 24);
            await Task.Delay(150);
            window.Position = position;
            Require(window.HostedViewport.FrameCount > frames, "Minimize/restore and window movement");
            window.RecreateViewport();
            await Task.Delay(250);
            Require(window.HostedViewport.IsReady && window.HostedViewport.FrameCount > 0, "Host recreation");
            model.Length = (model.Selected!.Length + .1).ToString("G12", CultureInfo.CurrentCulture);
            await Command(model.ApplyLengthCommand, "Dirty close setup");
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Cancel);
            Require(!await model.ConfirmClose(), "Cancel dirty close");
            var beforeCancelledNew = model.Snapshot!.Revision;
            await Command(model.NewCommand, "Cancelled dirty New");
            Require(model.Snapshot.Revision == beforeCancelledNew, "Cancelled replacement preserves document");
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Save);
            Require(await model.ConfirmClose(), "Save dirty close");
            Require(model.Controller.Latest?.Dirty == false, "Close save marks history baseline");
        }
        catch (Exception error) { failure = error.ToString(); }
        finally
        {
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            window.Close();
            // Closing owns GPU -> worker -> host teardown. Let its asynchronous
            // confirmation barrier finish before exiting the desktop lifetime.
            await Task.Delay(250);
            if (window.IsVisible) failure ??= "Window did not close.";
            else checks.Add("Clean window/controller/viewport shutdown");
            var directory = Path.GetDirectoryName(Path.GetFullPath(report))!;
            Directory.CreateDirectory(directory);
            await File.WriteAllTextAsync(report, JsonSerializer.Serialize(new
            {
                platform = Environment.OSVersion.ToString(), renderScaling,
                passed = failure is null, checks, measurements, failure,
                scope = "Live commands and child lifetime; visual, physical input and mixed-monitor DPI approval are separate."
            }, new JsonSerializerOptions { WriteIndented = true }));
            desktop.Shutdown(failure is null ? 0 : 1);
            File.Delete(temporary);
        }
    }
}
