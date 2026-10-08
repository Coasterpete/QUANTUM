using Quantum.Editor;
using Quantum.Managed;
internal static class HardwareChecks
{
    private static int checks;
    private static void Require(bool value, string text) { ++checks; if (!value) throw new Exception(text); }
    public static async Task Run()
    {
        using var controller = new SessionController(); await controller.Submit(_ => { });
        var vm = new EditorViewModel(controller); vm.Publish(controller.Latest!); vm.TimelineSource = 1;
        async Task Execute(EditorCommand command)
        { Require(command.CanExecute(null), "Hardware form command enabled"); await command.ExecuteAsync(); Require(vm.Error.Length == 0, vm.Error); }
        foreach (var choice in vm.HardwareChoices)
        {
            vm.SelectedHardwareChoice = choice; await Execute(vm.CreateHardwareCommand);
            Require(vm.SelectedDevice!.Kind == (uint)choice.Kind && vm.SelectedHardwareRuntime?.Configuration.ValueKind == System.Text.Json.JsonValueKind.Object,
                "Native typed settings and selection map for every hardware kind");
            Require(vm.HasAccelerationProfileHardware == (choice.Kind is DeviceKind.Launch or DeviceKind.Brake or DeviceKind.CableLift), "Only appropriate families expose profiles");
            if (choice.Kind == DeviceKind.Transport) { vm.HardwareSpeed = "1.5"; vm.HardwareDirection = 1; await Execute(vm.ApplyHardwareCommand);
                Require(vm.SelectedHardwareRuntime!.Settings(new TransportSettings()).Direction == -1, "Reverse tire command authored natively"); }
            if (choice.Kind == DeviceKind.ChainLift) { vm.HardwareSpeed = "4"; await Execute(vm.ApplyHardwareCommand);
                await Execute(vm.HardwareUnavailableCommand); Require(vm.SelectedHardwareRuntime!.State == "Unavailable", "Lift telemetry maps runtime availability"); }
            if (choice.Kind == DeviceKind.CableLift) { vm.HardwareReturn = "3"; await Execute(vm.ApplyHardwareCommand); }
            if (choice.Kind == DeviceKind.Station) { vm.HardwareStop = "20"; await Execute(vm.ApplyHardwareCommand);
                Require(vm.IsStationHardware && vm.ShowHardwareState && !vm.ShowProfileEditor, "Station form has state/configuration content"); }
            if (choice.Kind == DeviceKind.Transfer) { await Execute(vm.AddAlignmentCommand); vm.AlignmentName = "Maintenance"; vm.AlignmentKind = 2;
                await Execute(vm.ApplyAlignmentCommand); Require(vm.TransferAlignments.Last().Name == "Maintenance", "Transfer destination authoring reaches native snapshot");
                var deletedId = vm.SelectedAlignment!.Id;
                await Execute(vm.DeleteAlignmentCommand); await Execute(vm.AddAlignmentCommand);
                Require(vm.SelectedAlignment!.Id > deletedId, "Deleted alignment IDs are never reused"); }
        }
        Require(vm.IsAccessHardware && vm.ShowHardwareState, "Gate form uses mechanism content");
        vm.GateMotion = 1; vm.GateTravel = "2"; vm.GateMesh = "assets/gates/mesh.glb"; await Execute(vm.ApplyHardwareCommand);
        var revision = vm.Snapshot!.Revision;
        await Execute(vm.UnlockGateCommand); await Execute(vm.OpenGateCommand);
        Require(vm.Snapshot!.Revision == revision && vm.HardwareTelemetry.Contains("Opening"), "Managed runtime gate commands have no document revision");
        await Execute(vm.CloseGateCommand); await Execute(vm.LockGateCommand);
        var gate = vm.SelectedDevice!.Id;
        var station = vm.Snapshot!.Devices.First(d => d.Kind == (uint)DeviceKind.Station);
        vm.SelectedDevice = station; vm.HardwareGates = gate.ToString(); vm.HardwareBlock = vm.Snapshot.Devices.First(d => d.Kind == (uint)DeviceKind.Block).Id.ToString();
        await Execute(vm.ApplyHardwareCommand); await Execute(vm.DispatchCommand);
        Require(vm.SelectedHardwareRuntime!.DispatchRequested && !vm.SelectedHardwareRuntime.DispatchPermitted, "Native occupied block/unaligned station denies dispatch");
        var config = vm.SelectedHardwareRuntime.Settings(new StationSettings(0));
        var badRevision = vm.Snapshot!.Revision - 1; bool stale = false;
        try { await controller.Submit(s => s.SetHardwareConfiguration(badRevision, station.Id, config)); } catch (QuantumException e) { stale = e.Code == 8; }
        Require(stale, "Stale managed hardware config rejected");
        var temp = Path.Combine(Path.GetTempPath(), $"quantum-hardware-{Guid.NewGuid():N}.quantum");
        try
        {
            var snapshot = vm.Snapshot!;
            var infrastructure = new InfrastructureSettings(1, "Lift catwalk", 0, 0, 30, AccessGates: [gate]);
            await controller.Submit(s => s.SetInfrastructure(snapshot.Revision, [infrastructure], 2));
            await controller.Submit(s => { s.Save(temp); s.New(); s.Open(temp); }); vm.Publish(controller.Latest!);
            Require(vm.Snapshot!.Hardware.Infrastructure.Single().AccessGates![0] == gate, "Track-relative infrastructure persists through managed seam");
            var reopened = vm.Snapshot.Hardware.Hardware.First(h => h.Id == gate).Settings(new AccessSettings());
            Require(reopened.Motion == 1 && reopened.TravelDistanceMeters == 2 && reopened.MeshAsset.EndsWith("mesh.glb"), "Gate motion/appearance persists");
        }
        finally { File.Delete(temp); }
        Console.WriteLine($"Managed hardware forms/commands/telemetry/persistence: {checks} checks passed.");
    }
}
