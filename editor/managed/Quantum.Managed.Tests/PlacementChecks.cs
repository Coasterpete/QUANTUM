using System.Globalization;
using System.Text.Json;
using Quantum.Editor;
using Quantum.Managed;

internal static class PlacementChecks
{
    private static int checks;
    private static void Require(bool value, string message) { ++checks; if (!value) throw new Exception(message); }
    private static async Task Execute(EditorCommand command, EditorViewModel vm)
    {
        Require(command.CanExecute(null), "Placement command enabled");
        await command.ExecuteAsync(); Require(vm.Error.Length == 0, vm.Error);
    }
    public static async Task Run(string fixture)
    {
        using var controller = new SessionController();
        await controller.Submit(s => s.Open(fixture));
        var vm = new EditorViewModel(controller); vm.Publish(controller.Latest!);
        await Execute(vm.MoveTrackStartCommand, vm);
        Require(vm.Snapshot!.Placement is { Selected: true, Mode: TransformMode.Move }, "Track Start selection/tool comes from native snapshot");
        var original = vm.Snapshot.Placement.Authored;
        var revisions = new List<TrackStartPose> { original };
        vm.LayoutX = (original.X + 2).ToString("G12", CultureInfo.CurrentCulture);
        await Execute(vm.ApplyTrackStartPositionCommand, vm); revisions.Add(vm.Snapshot!.Placement.Authored);
        vm.LayoutY = (original.Y - 3).ToString("G12", CultureInfo.CurrentCulture);
        await Execute(vm.ApplyTrackStartPositionCommand, vm); revisions.Add(vm.Snapshot!.Placement.Authored);
        vm.LayoutZ = (original.Z + 15).ToString("G12", CultureInfo.CurrentCulture);
        await Execute(vm.ApplyTrackStartPositionCommand, vm); revisions.Add(vm.Snapshot!.Placement.Authored);
        Require(vm.Snapshot.Placement.Authored.Z == original.Z + 15 && vm.Snapshot.Route[0].StartZ == original.Z + 15,
            "Inspector Z raises whole native geometry");
        vm.LayoutYaw = (vm.Snapshot.Placement.Yaw + 12).ToString("G12", CultureInfo.CurrentCulture);
        await Execute(vm.ApplyTrackStartOrientationCommand, vm); revisions.Add(vm.Snapshot!.Placement.Authored);
        for (var i = revisions.Count - 2; i >= 0; --i)
        { await Execute(vm.UndoCommand, vm); Require(vm.Snapshot!.Placement.Authored == revisions[i], "Numeric Undo restores exact stored pose"); }
        for (var i = 1; i < revisions.Count; ++i)
        { await Execute(vm.RedoCommand, vm); Require(vm.Snapshot!.Placement.Authored == revisions[i], "Numeric Redo restores exact stored pose"); }
        var before = vm.Snapshot!;
        await vm.BeginTransformAsync(TransformMode.Move, TransformAxis.Z);
        for (var i = 0; i < 100; ++i) vm.QueueTransform(i * .1);
        vm.QueueTransform(5); await vm.EndTransformAsync(true);
        Require(!vm.InteractionActive && vm.Snapshot!.Revision == before.Revision + 1
            && vm.Snapshot.Placement.Authored.Z == before.Placement.Authored.Z + 5, "Coalesced Move commits final cumulative proposal once");
        await Execute(vm.UndoCommand, vm); Require(vm.Snapshot!.Placement.Authored == before.Placement.Authored, "One Undo restores entire Move");
        await Execute(vm.RedoCommand, vm);
        await Execute(vm.RotateTrackStartCommand, vm);
        before = vm.Snapshot!;
        // The native child can deliver down/move/up in one rendered frame.
        await vm.ReceiveTransformIntentAsync(new TransformIntent { Id = 100, Revision = before.Revision,
            SelectionRevision = before.Placement.SelectionRevision, Phase = 2, Mode = 2, Axis = 2, Amount = .15 });
        Require(vm.Snapshot!.Revision == before.Revision + 1 && vm.Snapshot.Placement.Authored != before.Placement.Authored,
            "Single-frame rotation intent uses begin/preview/one commit");
        await Execute(vm.UndoCommand, vm); Require(vm.Snapshot!.Placement.Authored == before.Placement.Authored, "One Undo restores rotation");
        await Execute(vm.RedoCommand, vm);
        before = vm.Snapshot!;
        await vm.BeginTransformAsync(TransformMode.Move, TransformAxis.X); vm.QueueTransform(8); await vm.EndTransformAsync(false);
        await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require(vm.Snapshot!.Revision == before.Revision && vm.Snapshot.Placement.Authored == before.Placement.Authored && !vm.Snapshot.Placement.Preview,
            "Managed cancellation discards preview with exact authored state");
        await vm.BeginTransformAsync(TransformMode.Move, TransformAxis.Z); vm.QueueTransform(0); await vm.EndTransformAsync(true);
        Require(vm.Snapshot!.Revision == before.Revision, "No-op managed drag makes no edit");
        await vm.BeginTransformAsync(TransformMode.Move, TransformAxis.X); vm.QueueTransform(3); vm.WorkspaceMode = 1;
        await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require(!vm.InteractionActive && vm.Snapshot!.Placement.Selected && !vm.Snapshot.Placement.Active, "Workspace switch cancels and preserves anchor identity");
        vm.WorkspaceMode = 0; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        await vm.BeginTransformAsync(TransformMode.Move, TransformAxis.Z);
        vm.Selected = vm.Snapshot!.Route.Skip(1).FirstOrDefault();
        await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require(!vm.InteractionActive && !vm.Snapshot!.Placement.Preview, "Region selection change cancels transform");
        await Execute(vm.MoveTrackStartCommand, vm);
        before = vm.Snapshot!;
        await vm.BeginTransformAsync(TransformMode.Move, TransformAxis.Z);
        await controller.Submit(s => s.EditTrackStart(before.Revision, false, before.Placement.Authored.X + 1, before.Placement.Authored.Y, before.Placement.Authored.Z));
        vm.Publish(controller.Latest!); await controller.Submit(_ => { });
        Require(!vm.InteractionActive && vm.Error.Contains("Document changed"), "Managed stale revision cancels without overwriting newer state");
        vm.Error = ""; vm.Publish(controller.Latest!);

        vm.TimelineSource = 1; await Execute(vm.CreateLaunchCommand, vm);
        vm.DeviceMode = 1; vm.DeviceAcceleration = "2"; await Execute(vm.ApplyDeviceCommand, vm);
        var device = vm.SelectedDevice!;
        var custom = vm.SelectedProfile!;
        await vm.BeginProfileDragAsync(custom, ProfileOperation.EndValue); vm.QueueProfileDrag(3); await vm.EndProfileDragAsync(true);
        Require(vm.SelectedDevice!.Custom && vm.SelectedProfile!.ValueEnd == 3, "Device custom profile dragging works after world placement");
        var profiles = JsonSerializer.Serialize(vm.Snapshot!.Profiles); var devices = vm.Snapshot.Devices.ToArray();
        vm.WorkspaceMode = 1; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        await Execute(vm.CreateSupportStructureCommand, vm);
        await Execute(vm.CreateSupportNodeCommand, vm); var attached = vm.SelectedSupportNode!.Id;
        vm.AttachmentStation = "10"; vm.AttachmentLateral = "1"; vm.AttachmentVertical = "-.5"; await Execute(vm.AttachSupportNodeCommand, vm);
        await Execute(vm.CreateSupportNodeCommand, vm); var free = vm.SelectedSupportNode!.Id;
        await Execute(vm.CreateSupportNodeCommand, vm); var foundation = vm.SelectedSupportNode!.Id;
        await Execute(vm.SetSupportFoundationCommand, vm);
        var sid = vm.SelectedStructure!.Id;
        var nodes = vm.SupportNodes.ToArray();
        vm.WorkspaceMode = 0; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        vm.LayoutZ = (vm.Snapshot!.Placement.Authored.Z + 5).ToString("G12", CultureInfo.CurrentCulture);
        await Execute(vm.ApplyTrackStartPositionCommand, vm);
        vm.LayoutYaw = (vm.Snapshot!.Placement.Yaw + 5).ToString("G12", CultureInfo.CurrentCulture);
        await Execute(vm.ApplyTrackStartOrientationCommand, vm);
        var after = vm.Snapshot!;
        Require(JsonSerializer.Serialize(after.Profiles) == profiles && after.Devices.SequenceEqual(devices), "Placement preserves Timeline and all custom-device authoring exactly");
        var updated = after.Supports.Nodes.Where(n => n.Structure == sid).ToArray();
        Require(updated.First(n => n.Id == free) == nodes.First(n => n.Id == free)
            && updated.First(n => n.Id == foundation) == nodes.First(n => n.Id == foundation), "Free and foundation XYZ remain world fixed");
        Require(updated.First(n => n.Id == attached).ResolvedZ != nodes.First(n => n.Id == attached).ResolvedZ
            && updated.First(n => n.Id == attached).Station == 10, "Attached support moves with unchanged station");
        Require(after.Supports.Selection.Structure == sid, "Route placement does not destroy support workspace selection");
        await Execute(vm.PlayCommand, vm); await Task.Delay(60); await Execute(vm.PauseCommand, vm);
        Require(vm.Snapshot!.Tick > 0, "Relocated native train plays and pauses");
        await Execute(vm.ResetCommand, vm); Require(vm.Snapshot!.Tick == 0, "Relocated train resets");
        var temporary = Path.Combine(Path.GetTempPath(), $"quantum-placement-managed-{Guid.NewGuid():N}.quantum");
        try
        {
            var finalPose = vm.Snapshot.Placement.Authored;
            await controller.Submit(s => s.Save(temporary)); await controller.Submit(s => s.New()); await controller.Submit(s => s.Open(temporary));
            vm.Publish(controller.Latest!);
            Require(vm.Snapshot!.Placement.Authored == finalPose && vm.Snapshot.Devices.Any(d => d.Id == device.Id && d.Custom), "Native save/reload through wrapper restores placement and custom device");
        }
        finally { File.Delete(temporary); }
        vm.TimelineSource = 0; vm.Selected = vm.Snapshot!.Route.First();
        var region = vm.SelectedProfile!;
        await vm.BeginProfileDragAsync(region, ProfileOperation.EndValue); vm.QueueProfileDrag(region.ValueEnd + .001); await vm.EndProfileDragAsync(true);
        Require(vm.Error.Length == 0 && vm.SelectedProfile!.ValueEnd == region.ValueEnd + .001, "Direct Region profile dragging still works after placement/reload");
        Console.WriteLine($"Managed placement: {checks} checks passed.");
    }
}
