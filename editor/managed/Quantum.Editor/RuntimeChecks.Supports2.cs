using Avalonia.Controls.ApplicationLifetimes;
using Quantum.Managed;
using System.Diagnostics;
using System.Text.Json;

namespace Quantum.Editor;

internal static partial class RuntimeChecks
{
    public static async Task RunSupports2Smoke(MainWindow window, string fixture, string report,
        IClassicDesktopStyleApplicationLifetime desktop)
    {
        var checks = new List<string>(); var watch = Stopwatch.StartNew();
        string? failure = null; var model = window.Model;
        void Require(bool condition, string name)
        { if (!condition) throw new InvalidOperationException($"{name}: {model.Error}"); checks.Add(name); }
        async Task Command(EditorCommand command, string name)
        { Require(command.CanExecute(null), $"{name} enabled"); await command.ExecuteAsync(); Require(model.Error.Length == 0,name); }
        try
        {
            await Task.Delay(300);
            Require(window.HostedViewport.IsReady && window.HostedViewport.FrameCount > 0, "Embedded Vulkan presents");
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            model.PickFile = _ => Task.FromResult<string?>(fixture);
            await Command(model.OpenCommand,"Open Supports 2 acceptance document");
            model.WorkspaceMode = 1;
            await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!);
            Require(model.SupportStructures.Count == 5 && model.InfrastructureItems.Count == 4,
                "All five structural families and infrastructure reach the editor");
            await model.SelectSupportAsync(new(SupportSelectionKind.Member, model.SupportStructures[0].Id,
                model.Snapshot!.Supports.Members.First().Id));
            await Command(model.FrameSupportCommand,"Frame timber member");
            model.MemberMaterial = 1; model.MemberRepeat = "2";
            await Command(model.ApplyMemberMaterialCommand,"Apply physical timber repeat");
            await Command(model.UndoCommand,"Undo material gesture");
            await Command(model.RedoCommand,"Redo material gesture");
            model.WorkspaceMode = 0; model.TimelineSource = 1;
            model.SelectedDevice = model.Snapshot!.Devices.First(d => d.Kind == (uint)DeviceKind.Access);
            model.GateDuration = ".2"; await Command(model.ApplyHardwareCommand,"Set bounded gate movement duration");
            await Command(model.PlayCommand,"Play native simulation");
            await Command(model.OpenGateCommand,"Open existing Access runtime");
            await Task.Delay(450); await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!);
            Require(model.SelectedHardwareRuntime is { Position: 1, State: "Open" }, "Gate reaches open position");
            await Command(model.CloseGateCommand,"Close existing Access runtime");
            await Task.Delay(450); await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!);
            Require(model.SelectedHardwareRuntime is { Position: 0, State: "Closed" }, "Gate reaches closed position");
            await Command(model.PauseCommand,"Pause native simulation");
            Require(window.HostedViewport.FrameCount > 10 && model.Snapshot!.Tick > 0,
                "Vulkan continues presenting styles, infrastructure and gate motion");
        }
        catch (Exception error) { failure = error.ToString(); }
        finally
        {
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            // MainWindow closes asynchronously after draining the controller
            // and destroying the native child. Await that lifetime boundary.
            var closed = new TaskCompletionSource();
            window.Closed += (_, _) => closed.TrySetResult();
            window.Close();
            try { await closed.Task.WaitAsync(TimeSpan.FromSeconds(5)); }
            catch (Exception error) { failure ??= error.ToString(); }
            await File.WriteAllTextAsync(report, JsonSerializer.Serialize(new
            {
                passed = failure is null, checks, elapsed_ms = watch.Elapsed.TotalMilliseconds,
                viewport_frames = window.HostedViewport.FrameCount, physical_pointer_testing = false,
                maintainer_visual_approval = "pending", failure
            }, new JsonSerializerOptions { WriteIndented = true }));
            desktop.Shutdown(failure is null ? 0 : 1);
        }
    }
}
