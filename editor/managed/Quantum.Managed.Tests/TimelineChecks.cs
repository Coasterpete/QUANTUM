using Avalonia;
using Quantum.Editor;
using Quantum.Managed;
using System.Diagnostics;
using System.Globalization;

internal static class TimelineChecks
{
    private static int checks;
    private static void Require(bool condition, string name)
    { ++checks; if (!condition) throw new Exception(name); }
    private static async Task Execute(EditorCommand command, EditorViewModel vm)
    {
        Require(command.CanExecute(null), "Authoring command enabled");
        await command.ExecuteAsync(); Require(vm.Error.Length == 0, vm.Error);
    }
    public static async Task Run()
    {
        using var controller = new SessionController();
        await controller.Submit(_ => { });
        var vm = new EditorViewModel(controller); vm.Publish(controller.Latest!);
        var initial = vm.SelectedProfile!;
        var plot = new ProfilePlot { Profiles = vm.VisibleProfiles.Where(p => p.Channel == 0).ToArray(), Selected = initial };
        plot.Measure(new Size(600, 240)); plot.Arrange(new Rect(0, 0, 600, 240));
        var hit = plot.HitSegment(new Point(319, 108.5));
        Require(hit?.Id == initial.Id && hit.Channel == 0, "Graph hit resolves native analytical segment without list preselection");
        plot.SegmentSelected += segment => { vm.SelectedProfile = vm.VisibleProfiles.First(p => p.Channel == segment.Channel && p.Id == segment.Id); plot.Selected = vm.SelectedProfile; };
        vm.SelectedProfile = vm.VisibleProfiles.First(p => p.Channel == 1);
        plot.SelectAt(new Point(319, 108.5));
        Require(vm.SelectedProfile!.Channel == 0 && plot.Selected == vm.SelectedProfile && vm.ProfileEnd == "0",
            "Pointer selection path synchronizes graph, segment selection and numeric form");
        Require(plot.HitHandle(new Point(54, 108.5)) is { Operation: ProfileOperation.BeginValue, Pinned: false }, "Outer value remains draggable");
        Require(plot.HitHandle(new Point(54, 207)) is { Operation: ProfileOperation.MoveBegin, Pinned: true }, "Outer distance handle visibly pinned");
        Require(!vm.MoveProfileBeginCommand.CanExecute(null) && !vm.MoveProfileEndCommand.CanExecute(null)
            && !vm.RemoveProfileCommand.CanExecute(null), "Impossible moves/remove disabled before native command");
        var revision = vm.Snapshot!.Revision;
        var listChanges = 0;
        vm.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(vm.VisibleProfiles)) ++listChanges; };
        await vm.BeginProfileDragAsync(initial, ProfileOperation.EndValue);
        Require(!vm.UndoCommand.CanExecute(null) && vm.InteractionActive, "Other authoring commands blocked during active gesture");
        var watch = Stopwatch.StartNew();
        for (var i = 0; i < 100; ++i) vm.QueueProfileDrag(i * .001);
        await vm.EndProfileDragAsync(true);
        Require(!vm.InteractionActive && vm.Error.Length == 0 && vm.SelectedProfile!.ValueEnd == .099,
            "Coalesced pointer candidates commit latest position");
        Require(vm.Snapshot!.Revision == revision + 1, "Entire managed gesture causes one document revision");
        Require(listChanges <= 5, "Pointer previews do not rebuild segment ItemsSource on each event");
        Console.WriteLine($"Controller burst (100 pointer candidates, including full commit): {watch.Elapsed.TotalMilliseconds:F3} ms");
        await Execute(vm.UndoCommand, vm);
        Require(vm.SelectedProfile!.ValueEnd == initial.ValueEnd && !vm.Snapshot!.CanUndo, "Value drag Undo is entire gesture");
        await vm.BeginProfileDragAsync(vm.SelectedProfile, ProfileOperation.EndValue); vm.QueueProfileDrag(.2);
        await vm.EndProfileDragAsync(false); await controller.Submit(_ => { });
        Require(controller.Latest!.CanRedo && !controller.Latest.Dirty, "Cancel preserves existing Redo and clean-state marker");
        await Execute(vm.RedoCommand, vm); Require(vm.SelectedProfile!.ValueEnd == .099, "Value drag Redo");

        vm.ProfileDistance = "30"; await Execute(vm.SplitProfileCommand, vm);
        Require(vm.CanMoveProfileEnd && !vm.CanMoveProfileBegin, "After split selected left segment has movable end only");
        var left = vm.SelectedProfile!;
        var right = vm.VisibleProfiles.First(p => p.Channel == left.Channel && p.Id != left.Id);
        vm.SelectedProfile = right;
        Require(vm.ProfileBegin == right.ValueBegin.ToString("G12", CultureInfo.CurrentCulture) && vm.CanMoveProfileBegin && !vm.CanMoveProfileEnd,
            "List selection synchronizes numeric form and pinned button state");
        vm.SelectedProfile = left;
        await vm.BeginProfileDragAsync(left, ProfileOperation.MoveEnd); vm.QueueProfileDrag(25); await vm.EndProfileDragAsync(true);
        Require(vm.SelectedProfile!.End == 25 && vm.VisibleProfiles.First(p => p.Channel == left.Channel && p.Id == right.Id).Begin == 25,
            "Boundary gesture refreshes native contiguous samples");
        await Execute(vm.UndoCommand, vm); Require(vm.SelectedProfile!.End == 30, "Boundary Undo");
        await Execute(vm.RedoCommand, vm); Require(vm.SelectedProfile!.End == 25, "Boundary Redo");
        vm.SelectedTransition = vm.TransitionTypes.First(t => t.Type != vm.SelectedProfile!.TransitionType);
        var type = vm.SelectedTransition.Type; await Execute(vm.ApplyTransitionCommand, vm);
        Require(vm.SelectedProfile!.TransitionType == type, "All type choices come from native catalog");
        await Execute(vm.UndoCommand, vm); await Execute(vm.RedoCommand, vm);
        Require(vm.SelectedProfile!.TransitionType == type, "Transition history roundtrip");
        await Execute(vm.RemoveProfileCommand, vm); await Execute(vm.UndoCommand, vm); await Execute(vm.RedoCommand, vm);
        Require(vm.VisibleProfiles.Count(p => p.Channel == 0) == 1, "Remove Undo/Redo and selection recovery");
        vm.SelectedProfile = vm.VisibleProfiles.First(p => p.Channel == 2);
        vm.ProfileDistance = "30"; await Execute(vm.SplitProfileCommand, vm);
        vm.SelectedProfile = vm.VisibleProfiles.First(p => p.Channel == 2 && p.Id != 1);
        await Execute(vm.RemoveProfileCommand, vm);
        Require(vm.SelectedProfile is { Channel: 2, Id: 1 }, "Removing a Yaw segment preserves channel selection");
        await Execute(vm.UndoCommand, vm); await Execute(vm.RedoCommand, vm);
        Require(vm.SelectedProfile is { Channel: 2, Id: 1 }, "Yaw removal history keeps a valid selected segment in that channel");

        var cancelBefore = vm.Snapshot!.Revision;
        await vm.BeginProfileDragAsync(vm.SelectedProfile!, ProfileOperation.EndValue); vm.QueueProfileDrag(.2);
        await vm.EndProfileDragAsync(false); await controller.Submit(_ => { });
        Require(controller.Latest!.Revision == cancelBefore && !vm.InteractionActive, "Escape/capture loss cancellation changes no history");
        await vm.BeginProfileDragAsync(vm.SelectedProfile!, ProfileOperation.EndValue); vm.QueueProfileDrag(.2);
        vm.SelectedProfile = vm.VisibleProfiles.First(p => p.Channel == 1);
        await controller.Submit(_ => { });
        Require(!vm.InteractionActive && vm.GraphProfiles == vm.VisibleProfiles && controller.Latest!.Revision == cancelBefore,
            "Selection change cancels preview and restores accepted graph");
        await vm.BeginProfileDragAsync(vm.SelectedProfile!, ProfileOperation.EndValue); vm.QueueProfileDrag(.2);
        await controller.Submit(s => s.Command(SessionCommand.SetInitialSpeed, vm.Snapshot!.Revision, .1));
        vm.Publish(controller.Latest!); await controller.Submit(_ => { });
        Require(!vm.InteractionActive && vm.Error.Contains("changed"), "New revision safely aborts gesture");

        vm.TimelineSource = 1;
        await Execute(vm.CreateLaunchCommand, vm);
        var id = vm.SelectedDevice!.Id;
        Require(vm.SelectedDevice.Kind == 0 && !vm.SelectedDevice.Custom && vm.VisibleProfiles.Count == 0, "Create/select Launch in Constant mode");
        await Execute(vm.UndoCommand, vm); Require(vm.Snapshot!.Devices.Count == 0, "Create Undo");
        await Execute(vm.RedoCommand, vm); Require(vm.SelectedDevice!.Id == id, "Create Redo restores stable ID");
        vm.DeviceStart = "3"; vm.DeviceEnd = "20"; vm.DeviceForce = "8000"; vm.DeviceEnabled = false;
        await Execute(vm.ApplyDevicePropertiesCommand, vm);
        Require(vm.SelectedDevice!.Start == 3 && vm.SelectedDevice.End == 20 && !vm.SelectedDevice.Enabled && vm.SelectedDevice.MaximumForce == 8000,
            "Native numeric interval/force/enable editing");
        await Execute(vm.UndoCommand, vm); await Execute(vm.RedoCommand, vm);
        var valid = vm.Snapshot!.Revision;
        vm.DeviceEnd = vm.DeviceStart; await vm.ApplyDevicePropertiesCommand.ExecuteAsync();
        Require(vm.Error.Length > 0 && vm.Snapshot!.Revision == valid && vm.DeviceEnd == "20", "Rejected numeric candidate restores native form");
        vm.DeviceMode = 1; vm.DeviceAcceleration = "2"; await Execute(vm.ApplyDeviceCommand, vm);
        Require(vm.SelectedDevice!.Custom && vm.SelectedProfile!.Channel == 3, "Custom mode seeds native SI profile");
        await Execute(vm.UndoCommand, vm); Require(!vm.SelectedDevice!.Custom, "Custom mode Undo");
        await Execute(vm.RedoCommand, vm); Require(vm.SelectedDevice!.Custom, "Custom mode Redo");
        vm.ProfileDistance = "8"; await Execute(vm.SplitProfileCommand, vm);
        await vm.BeginProfileDragAsync(vm.SelectedProfile!, ProfileOperation.EndValue); vm.QueueProfileDrag(3); await vm.EndProfileDragAsync(true);
        Require(vm.Error.Length == 0 && vm.SelectedProfile!.ValueEnd == 3, "Custom acceleration vertical gesture");
        await Execute(vm.UndoCommand, vm); await Execute(vm.RedoCommand, vm); Require(vm.SelectedProfile!.ValueEnd == 3, "Custom gesture history");
        await vm.BeginProfileDragAsync(vm.SelectedProfile!, ProfileOperation.MoveEnd); vm.QueueProfileDrag(6); await vm.EndProfileDragAsync(true);
        Require(vm.SelectedProfile!.End == 6, "Custom acceleration boundary gesture");
        await Execute(vm.UndoCommand, vm); await Execute(vm.RedoCommand, vm);
        valid = vm.Snapshot!.Revision;
        var acceptedValue = vm.SelectedProfile!.ValueBegin;
        await vm.BeginProfileDragAsync(vm.SelectedProfile, ProfileOperation.BeginValue); vm.QueueProfileDrag(-1); await vm.EndProfileDragAsync(true);
        Require(!vm.InteractionActive && vm.Error.Length > 0 && controller.Latest!.Revision == valid
            && vm.ProfileBegin == acceptedValue.ToString("G12", CultureInfo.CurrentCulture) && vm.GraphProfiles == vm.VisibleProfiles,
            "Rejected acceleration preview restores graph/form without history");
        vm.DeviceEnabled = true; await Execute(vm.ApplyDevicePropertiesCommand, vm);
        var path = Path.Combine(Path.GetTempPath(), $"quantum-integrated-authoring-{Guid.NewGuid():N}.quantum");
        try
        {
            vm.TimelineSource = 0; vm.SelectedProfile = vm.VisibleProfiles.First(p => p.Channel == 0);
            vm.ProfileBegin = (.05).ToString("G12", CultureInfo.CurrentCulture); await Execute(vm.ApplyProfileBeginCommand, vm);
            vm.PickFile = _ => Task.FromResult<string?>(path); await Execute(vm.SaveAsCommand, vm);
            vm.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            await Execute(vm.NewCommand, vm); await Execute(vm.OpenCommand, vm);
            vm.TimelineSource = 1;
            Require(vm.SelectedDevice!.Id == id && vm.SelectedDevice.Custom && vm.SelectedDevice.Start == 3 && vm.SelectedDevice.End == 20
                && vm.VisibleProfiles.First(p => p.Id == 1).End == 6 && vm.VisibleProfiles.First(p => p.Id == 1).ValueEnd == 3,
                "Coherent device + Region authoring save/reload");
            Require(Math.Abs(vm.Snapshot!.Profiles.First(p => p.OwnerKind == 0 && p.Channel == 0).ValueBegin - .05) < 1e-12, "Region edit also restored");
            await Execute(vm.PlayCommand, vm);
            await controller.Submit(s => s.Update(.05));
            vm.Publish(controller.Latest!); Require(vm.Snapshot!.Tick > 0, "Authored device remains consumable by native fixed-step preview");
            await Execute(vm.PauseCommand, vm); await Execute(vm.ResetCommand, vm);
            await Execute(vm.DeleteDeviceCommand, vm); Require(vm.Snapshot!.Devices.Count == 0, "Device deletion");
            await Execute(vm.UndoCommand, vm); Require(vm.SelectedDevice!.Id == id, "Delete Undo restores selection");
            await Execute(vm.RedoCommand, vm); Require(vm.Snapshot!.Devices.Count == 0, "Delete Redo");
            await Execute(vm.CreateBrakeCommand, vm); Require(vm.SelectedDevice!.Kind == 1 && vm.SelectedDevice.Id > id, "Create Brake without ID reuse");
        }
        finally { File.Delete(path); }
        Console.WriteLine($"Managed timeline/controller: {checks} checks passed.");
    }
}
