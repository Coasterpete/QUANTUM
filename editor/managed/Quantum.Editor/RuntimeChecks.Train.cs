using Avalonia.Controls.ApplicationLifetimes;
using Quantum.Managed;
using System.Diagnostics;
using System.Text.Json;

namespace Quantum.Editor;
internal static partial class RuntimeChecks
{
    public static async Task RunTrain2Smoke(MainWindow window, string fixture, string report,
        IClassicDesktopStyleApplicationLifetime desktop)
    {
        var checks = new List<string>(); var watch = Stopwatch.StartNew(); var model = window.Model;
        string? failure = null;
        void Require(bool condition, string name)
        { if (!condition) throw new InvalidOperationException($"{name}: {model.Error}"); checks.Add(name); }
        async Task Command(EditorCommand command, string name)
        { Require(command.CanExecute(null), $"{name} enabled"); await command.ExecuteAsync(); Require(model.Error.Length == 0, name); }
        try
        {
            await Task.Delay(300);
            Require(window.HostedViewport.IsReady && window.HostedViewport.FrameCount > 0, "Embedded Vulkan presents");
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            model.PickFile = _ => Task.FromResult<string?>(fixture);
            await Command(model.OpenCommand, "Open heterogeneous Train 2 fixture");
            model.WorkspaceMode = 2; await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!);
            Require(model.TrainCars.Count == 4 && model.TrainCars.Select(c => c.Physical.DryMassKilograms).Distinct().Count() == 4,
                "Train workspace binds heterogeneous native definitions");
            await Command(model.FrameTrainCommand, "Frame complete train");
            await Command(model.FrameTrainFrontCommand, "Frame front reference envelope");
            var first = model.TrainCars[0].Id;
            model.SelectedCar = model.TrainCars[^1]; await model.Controller.Submit(_ => { });
            model.Publish(model.Controller.Latest!); await Task.Delay(200);
            // The production native pick path receives one synthetic click.
            model.Controller.Session.Camera(ViewportAction.PickTrainCar, .5, .5);
            await Task.Delay(200); await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!);
            Require(model.SelectedCar!.Id == first, "Native car-envelope click synchronizes back to Train selection");
            var id = model.SelectedCar.Id;
            await Command(model.MoveCarDownCommand, "Reorder a physical/visual car");
            Require(model.TrainCars[1].Id == id, "Bound consist order follows native reorder");
            await Command(model.UndoCommand, "Undo reorder");
            model.CarAlignmentFields.Single(f => f.Path == "visualTranslationMeters.2").Value = ".75";
            await Command(model.ApplyCarCommand, "Apply explicit shell alignment");
            await Command(model.PlayCommand, "Play authored train"); await Task.Delay(180);
            await Command(model.PauseCommand, "Pause authored train");
            Require(model.Snapshot!.Tick > 0, "Native fixed ticks advance");
            await Command(model.ResetCommand, "Reset current authored train");
            Require(model.Snapshot!.Tick == 0 && model.SelectedCar!.VisualTranslationMeters[2] == .75, "Reset retains edited consist");
            await Task.Delay(150);
            Require(window.HostedViewport.FrameCount > 5, "Vulkan keeps presenting train overlays and GLBs");
        }
        catch (Exception error) { failure = error.ToString(); }
        finally
        {
            await File.WriteAllTextAsync(report, JsonSerializer.Serialize(new {
                passed = failure is null, checks, elapsed_ms = watch.Elapsed.TotalMilliseconds,
                viewport_frames = window.HostedViewport.FrameCount, physical_pointer_testing = false,
                maintainer_visual_approval = "pending", failure }, new JsonSerializerOptions { WriteIndented = true }));
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard); window.Close();
            await Task.Delay(150); desktop.Shutdown(failure is null ? 0 : 1);
        }
    }
}
