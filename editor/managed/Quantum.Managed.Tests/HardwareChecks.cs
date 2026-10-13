using Quantum.Editor;
using Quantum.Managed;
internal static class HardwareChecks
{
    private static int checks;
    private static void Require(bool value, string text) { ++checks; if (!value) throw new Exception(text); }
    public static async Task Run()
    {
        await VelocityControlPath();
        await LaunchTargetComparisons();
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

    private static async Task LaunchTargetComparisons()
    {
        using var controller = new SessionController();
        await controller.Submit(s =>
        {
            s.Command(SessionCommand.SetRegionLength, s.Snapshot().Revision, 300, 0);
            s.Command(SessionCommand.SetInitialSpeed, s.Snapshot().Revision, 5);
        });
        var vm = new EditorViewModel(controller);
        vm.Publish(controller.Latest!); vm.TimelineSource = 1;
        vm.SelectedHardwareChoice = vm.HardwareChoices.First(c => c.Kind == DeviceKind.Launch);
        await vm.CreateHardwareCommand.ExecuteAsync();
        vm.DeviceStart = "0"; vm.DeviceEnd = "70"; vm.DeviceForce = "100000";
        await vm.ApplyDevicePropertiesCommand.ExecuteAsync();
        vm.DeviceMode = 0; vm.DeviceAcceleration = "4"; await vm.ApplyDeviceCommand.ExecuteAsync();
        vm.DeviceMode = 1; await vm.ApplyDeviceCommand.ExecuteAsync();
        var exitSpeeds = new List<double>();
        foreach (double? target in new double?[] { null, 10, 25 })
        {
            vm.HardwareTarget = target?.ToString(System.Globalization.CultureInfo.CurrentCulture) ?? "";
            await vm.ApplyHardwareCommand.ExecuteAsync();
            Require(vm.Error.Length == 0 && vm.SelectedHardwareRuntime!.Settings(new LaunchSettings()).TargetSpeedMetersPerSecond == target,
                "Blank/numeric Target Speed travels through view model, controller, ABI and authored configuration");
            Require(vm.VisibleProfiles.All(p => p.ValueBegin == 4 && p.ValueEnd == 4), "Target comparison preserves Custom profile");
            Require(vm.Snapshot!.Speed == 5 && vm.Snapshot.InitialSpeed == 5, "Applying a target never assigns initial train velocity");
            bool exited = false, limited = false;
            await controller.Submit(s =>
            {
                s.Command(SessionCommand.Play);
                for (int i = 0; i < 80; ++i)
                {
                    s.Update(.25);
                    var h = s.Snapshot().Hardware.Hardware.Single();
                    limited |= h.TargetSpeedLimited;
                    if (h.OccupiedBogieCount == 0) { exited = true; break; }
                }
                s.Command(SessionCommand.Pause);
            });
            vm.Publish(controller.Latest!);
            Require(exited, "Managed short-launch comparison exits interval");
            exitSpeeds.Add(vm.Snapshot!.Speed);
            Require(limited == (target == 10), "Copied native telemetry distinguishes active/unreached/unset target limiting");
            Require(vm.HardwareTelemetry.Contains(target is null ? "Target unset" : $"Target {target:F2}"),
                "Telemetry shows accepted authored target including unset");
            if (target == 10) Require(vm.Snapshot.Speed <= 10.02 && vm.Snapshot.Speed > 9.5, "10 m/s target constrains native Custom-profile playback");
            if (target == 25) Require(vm.Snapshot.Speed < 25, "High target cannot make a short launch reach it instantly");
            Console.WriteLine($"Managed launch comparison: target={target?.ToString() ?? "unset"}, exit={vm.Snapshot.Speed:F6} m/s, limited={limited}");
        }
        Require(exitSpeeds[0] > exitSpeeds[1] + 2 && exitSpeeds[0] == exitSpeeds[2],
            "Changing only launch target changes physical speed only when reachable");
        vm.HardwareTarget = "10"; await vm.ApplyHardwareCommand.ExecuteAsync();
        foreach (double initial in new[] { 5.0, 20.0 })
        {
            await controller.Submit(s =>
            {
                s.Command(SessionCommand.SetInitialSpeed, s.Snapshot().Revision, initial);
                s.Command(SessionCommand.Play); s.Update(1.0 / 240); s.Command(SessionCommand.Reset);
            });
            vm.Publish(controller.Latest!);
            Require(vm.Snapshot!.Tick == 0 && vm.Snapshot.Speed == initial && vm.Snapshot.InitialSpeed == initial,
                "Managed Initial Speed edit and Reset retain authored start speed");
            Require(vm.SelectedHardwareRuntime!.Settings(new LaunchSettings()).TargetSpeedMetersPerSecond == 10,
                "Initial Speed edit leaves hardware target unchanged");
        }
        await controller.Submit(s => { s.Command(SessionCommand.Play); s.Update(1.0 / 240); s.Command(SessionCommand.Pause); });
        vm.Publish(controller.Latest!);
        var above = vm.SelectedHardwareRuntime!;
        Require(above.State == "Complete" && above.TargetSpeedLimited && above.ProfileAcceleration == 4
            && above.CommandedAcceleration == 0 && above.AppliedForce == 0 && vm.Snapshot!.Speed > 10,
            "Above-target launch telemetry distinguishes profile, released propulsion and actual overspeed");
    }

    private static async Task VelocityControlPath()
    {
        using var controller = new SessionController();
        await controller.Submit(s =>
        {
            s.Command(SessionCommand.SetRegionLength, s.Snapshot().Revision, 300, 0);
            s.Command(SessionCommand.SetInitialSpeed, s.Snapshot().Revision, 3);
        });
        var vm = new EditorViewModel(controller);
        vm.Publish(controller.Latest!); vm.TimelineSource = 1;
        vm.SelectedHardwareChoice = vm.HardwareChoices.First(c => c.Kind == DeviceKind.Transport);
        await vm.CreateHardwareCommand.ExecuteAsync();
        vm.DeviceEnd = "280"; vm.DeviceForce = "1000000";
        await vm.ApplyDevicePropertiesCommand.ExecuteAsync();
        var speeds = new List<double>();
        foreach (var target in new[] { 1.0, 6.0 })
        {
            var revision = vm.Snapshot!.Revision;
            vm.HardwareSpeed = target.ToString(System.Globalization.CultureInfo.CurrentCulture);
            await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
            Require(double.Parse(vm.HardwareSpeed) == target, "Telemetry publication preserves the pending velocity edit");
            vm.DeviceAcceleration = target == 1 ? "5" : "4";
            await vm.ApplyDeviceCommand.ExecuteAsync();
            Require(vm.Error.Length == 0 && double.Parse(vm.HardwareSpeed) == target,
                "Applying the response limit preserves the pending velocity edit before Apply hardware configuration");
            vm.DeviceAcceleration = "4"; await vm.ApplyDeviceCommand.ExecuteAsync();
            vm.DeviceEnd = target == 1 ? "281" : "282";
            await vm.ApplyDevicePropertiesCommand.ExecuteAsync();
            Require(vm.Error.Length == 0 && double.Parse(vm.HardwareSpeed) == target,
                "Applying placement preserves the pending velocity edit");
            await vm.ApplyHardwareCommand.ExecuteAsync();
            Require(vm.Error.Length == 0 && vm.Snapshot!.Revision > revision, "Managed velocity field commits an authored revision");
            var hardware = vm.SelectedHardwareRuntime!;
            Require(hardware.Settings(new TransportSettings()).SpeedMetersPerSecond == target
                && vm.Snapshot!.Hardware.Revision == vm.Snapshot.Revision, "Velocity configuration and ABI snapshot agree");
            Require(hardware.OccupiedBogieCount > 0 && hardware.CommandedSpeed == target, "RideOperations consumes the committed velocity");
            await controller.Submit(s =>
            {
                s.Command(SessionCommand.Play);
                for (var i = 0; i < 12; ++i) s.Update(.25);
                s.Command(SessionCommand.Pause);
            });
            vm.Publish(controller.Latest!);
            speeds.Add(vm.Snapshot!.Speed);
            Require(Math.Abs(vm.Snapshot.Speed - target) < .08, $"Velocity target {target} reaches native train physics (actual {vm.Snapshot.Speed})");
            Console.WriteLine($"Velocity path: authored={target}, command={vm.SelectedHardwareRuntime!.CommandedSpeed}, actual={vm.Snapshot.Speed:F6}, force={vm.SelectedHardwareRuntime.AppliedForce:F1} N");
        }
        Require(speeds[1] - speeds[0] > 4.8, "Changing only velocity changes the actual train response");
        var acceptedRevision = vm.Snapshot!.Revision;
        vm.HardwareSpeed = "7"; vm.DeviceAcceleration = "-1";
        await vm.ApplyDeviceCommand.ExecuteAsync();
        Require(vm.Error.Length > 0 && vm.Snapshot!.Revision == acceptedRevision && double.Parse(vm.HardwareSpeed) == 6,
            "Rejected response edit retains the existing accepted-form restoration policy");
        vm.DeviceAcceleration = "4"; await vm.ApplyDeviceCommand.ExecuteAsync();

        await vm.DeleteDeviceCommand.ExecuteAsync();
        foreach (var kind in new[] { DeviceKind.Launch, DeviceKind.Brake })
        {
            await controller.Submit(s => s.Command(SessionCommand.SetInitialSpeed, s.Snapshot().Revision, kind == DeviceKind.Launch ? 3 : 10));
            vm.Publish(controller.Latest!);
            vm.SelectedHardwareChoice = vm.HardwareChoices.First(c => c.Kind == kind);
            await vm.CreateHardwareCommand.ExecuteAsync();
            vm.DeviceStart = "0"; vm.DeviceEnd = "280"; vm.DeviceForce = "1000000";
            await vm.ApplyDevicePropertiesCommand.ExecuteAsync();
            foreach (var target in kind == DeviceKind.Launch ? new[] { 4.0, 8.0 } : new[] { 1.0, 6.0 })
            {
                vm.HardwareTarget = target.ToString(System.Globalization.CultureInfo.CurrentCulture);
                vm.DeviceMode = 0; vm.DeviceAcceleration = "4"; await vm.ApplyDeviceCommand.ExecuteAsync();
                vm.DeviceMode = 1; await vm.ApplyDeviceCommand.ExecuteAsync();
                Require(vm.Error.Length == 0 && double.Parse(vm.HardwareTarget) == target,
                    "Applying Constant/Custom mode preserves the pending launch/brake target");
                Require(vm.VisibleProfiles.All(p => p.ValueBegin == 4 && p.ValueEnd == 4), "Velocity comparison retains the same Custom acceleration profile");
                await vm.ApplyHardwareCommand.ExecuteAsync();
                var hardware = vm.SelectedHardwareRuntime!;
                var authored = kind == DeviceKind.Launch ? hardware.Settings(new LaunchSettings()).TargetSpeedMetersPerSecond
                    : hardware.Settings(new BrakeSettings()).TargetSpeedMetersPerSecond;
                Require(vm.Error.Length == 0 && authored == target && hardware.CommandedSpeed == target,
                    "Launch/brake target reaches configuration snapshot and RideOperations");
                await controller.Submit(s =>
                {
                    s.Command(SessionCommand.Play);
                    if (kind == DeviceKind.Brake)
                    {
                        // A trim brake releases at its target; resistance then
                        // keeps slowing the train. Check the first target crossing.
                        for (var i = 0; i < 720; ++i)
                        {
                            s.Update(1.0 / 240);
                            if (s.Snapshot().Speed <= target) break;
                        }
                        s.Update(1.0 / 240);
                    }
                    else for (var i = 0; i < 12; ++i) s.Update(.25);
                    s.Command(SessionCommand.Pause);
                });
                vm.Publish(controller.Latest!);
                Require(Math.Abs(vm.Snapshot!.Speed - target) < .08,
                    $"{kind} target {target} limits the Custom-profile force path (actual {vm.Snapshot.Speed})");
                if (kind == DeviceKind.Brake)
                    Require(vm.SelectedHardwareRuntime!.State == "At target" && vm.SelectedHardwareRuntime.AppliedForce == 0,
                        "Trim brake releases its force at the authored target");
                Console.WriteLine($"{kind} velocity path: authored={target}, command={vm.SelectedHardwareRuntime!.CommandedSpeed}, actual={vm.Snapshot.Speed:F6}");
            }
            await vm.UndoCommand.ExecuteAsync();
            Require(double.Parse(vm.HardwareTarget) == (kind == DeviceKind.Launch ? 4 : 1), "Velocity Undo refreshes the authored target field");
            await vm.RedoCommand.ExecuteAsync();
            Require(double.Parse(vm.HardwareTarget) == (kind == DeviceKind.Launch ? 8 : 6), "Velocity Redo refreshes the authored target field");
            await vm.DeleteDeviceCommand.ExecuteAsync();
        }
    }
}
