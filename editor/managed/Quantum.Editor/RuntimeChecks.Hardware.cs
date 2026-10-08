using Avalonia.Controls.ApplicationLifetimes;
using Quantum.Managed;
using System.Diagnostics;
using System.Text.Json;
namespace Quantum.Editor;
internal static partial class RuntimeChecks
{
    public static async Task RunHardwareSmoke(MainWindow window, string fixture, string report,
        IClassicDesktopStyleApplicationLifetime desktop)
    {
        var checks = new List<string>(); var watch = Stopwatch.StartNew(); string? failure = null;
        var model = window.Model;
        void Require(bool condition, string name)
        { if (!condition) throw new InvalidOperationException($"{name}: {model.Error}"); checks.Add(name); }
        async Task Execute(EditorCommand command, string name)
        { Require(command.CanExecute(null), $"{name} enabled"); await command.ExecuteAsync(); Require(model.Error.Length == 0, name); }
        try
        {
            await Task.Delay(300);
            Require(window.HostedViewport.IsReady && window.HostedViewport.FrameCount > 0, "Embedded Vulkan presents frames");
            Require(window.Icon is not null, "Canonical application icon present");
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            model.PickFile = _ => Task.FromResult<string?>(fixture);
            await Execute(model.OpenCommand, "Open existing hybrid document");
            model.TimelineSource = 1;
            model.SelectedHardwareChoice = model.HardwareChoices.First(c => c.Kind == DeviceKind.Transport);
            await Execute(model.CreateHardwareCommand, "Create transport in live workspace");
            model.HardwareSpeed = "1.25"; await Execute(model.ApplyHardwareCommand, "Apply native transport speed");
            Require(model.ShowHardwareState && !model.ShowProfileEditor, "Transport uses configuration/state content");
            await Execute(model.UndoCommand, "Hardware Undo"); await Execute(model.RedoCommand, "Hardware Redo");
            model.SelectedHardwareChoice = model.HardwareChoices.First(c => c.Kind == DeviceKind.Access);
            await Execute(model.CreateHardwareCommand, "Create reusable gate");
            model.GateMotion = 1; model.GateTravel = "2"; model.GateDuration = "0.2";
            await Execute(model.ApplyHardwareCommand, "Apply sliding gate configuration");
            var revision = model.Snapshot!.Revision;
            await Execute(model.PlayCommand, "Start native fixed stepping");
            await Execute(model.UnlockGateCommand, "Unlock gate runtime"); await Execute(model.OpenGateCommand, "Open gate runtime");
            await Task.Delay(400); await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!);
            Require(model.SelectedHardwareRuntime?.Position == 1 && model.SelectedHardwareRuntime.State == "Open", "Gate reaches open state during native playback");
            Require(model.Snapshot!.Revision == revision && model.Snapshot.Tick > 0, "Runtime mechanism steps without authored revision changes");
            await Execute(model.PauseCommand, "Pause"); await Execute(model.ResetCommand, "Reset");
            Require(model.SelectedHardwareRuntime?.Locked == true && model.SelectedHardwareRuntime.Position == 0, "Reset restores closed/locked authored gate");
            Require(window.HostedViewport.FrameCount > 10, "Vulkan continues presenting through hardware edits and playback");
        }
        catch (Exception error) { failure = error.ToString(); }
        finally
        {
            var result = JsonSerializer.Serialize(new { success = failure is null, seconds = watch.Elapsed.TotalSeconds,
                checks, failure, physicalInputVerified = false, maintainerApproval = "Pending" }, new JsonSerializerOptions { WriteIndented = true });
            await File.WriteAllTextAsync(report, result);
            desktop.Shutdown(failure is null ? 0 : 1);
        }
    }
}
