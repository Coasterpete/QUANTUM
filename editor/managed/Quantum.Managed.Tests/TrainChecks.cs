using Quantum.Editor;
using Quantum.Managed;
using System.Diagnostics;

internal static class TrainChecks
{
    private static int checks;
    private static void Require(bool condition, string message) { ++checks; if (!condition) throw new Exception(message); }
    public static async Task Run()
    {
        using var controller = new SessionController(); await controller.Submit(_ => { });
        var vm = new EditorViewModel(controller); vm.Publish(controller.Latest!);
        vm.WorkspaceMode = 2; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require(vm.IsTrainWorkspace && !vm.IsRouteWorkspace && vm.Snapshot!.Train.Active, "Train is a separate native-backed workspace");
        Require(vm.TrainCars.Count == 4 && vm.Snapshot!.Train.Summary.BogieCount == 8, "Ordered snapshot and native summary mapping");
        SessionSnapshot? cached = null; await controller.Submit(session => cached = session.Snapshot());
        Require(ReferenceEquals(vm.Snapshot!.Train.Configuration, cached!.Train.Configuration), "Train authored snapshot is revision cached");
        async Task Execute(EditorCommand command)
        { Require(command.CanExecute(null), "Train command enabled"); await command.ExecuteAsync(); Require(vm.Error.Length == 0, vm.Error); }
        async Task History(string before)
        {
            var after = string.Join(',', vm.TrainCars.Select(c => $"{c.Id}:{c.Role}:{c.Physical.DryMassKilograms}:{c.VisualAsset}"));
            await Execute(vm.UndoCommand);
            Require(string.Join(',', vm.TrainCars.Select(c => $"{c.Id}:{c.Role}:{c.Physical.DryMassKilograms}:{c.VisualAsset}")) == before, "Managed gesture native Undo");
            await Execute(vm.RedoCommand);
            Require(string.Join(',', vm.TrainCars.Select(c => $"{c.Id}:{c.Role}:{c.Physical.DryMassKilograms}:{c.VisualAsset}")) == after, "Managed gesture native Redo");
        }
        string Authored() => string.Join(',', vm.TrainCars.Select(c => $"{c.Id}:{c.Role}:{c.Physical.DryMassKilograms}:{c.VisualAsset}"));
        var first = vm.SelectedCar!.Id; var before = Authored();
        vm.TrainPrototypeRole = "Zero"; await Execute(vm.InsertCarCommand);
        Require(vm.TrainCars.Count == 5 && vm.TrainCars[0].Role == "Zero" && vm.TrainCars[0].Physical.LoadMassKilograms == 0, "Zero prototype has native distinct mass and no passengers");
        var zeroId = vm.SelectedCar!.Id; await History(before);
        vm.SelectedCar = vm.TrainCars.Single(c => c.Id == zeroId); await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require(vm.Snapshot!.Train.SelectedCar == zeroId, "Managed car selection routes to native preview");
        before = Authored(); await Execute(vm.MoveCarDownCommand);
        Require(vm.TrainCars[1].Id == zeroId && vm.TrainCars[0].Id == first, "Reorder changes native physical and visual order"); await History(before);
        vm.SelectedCar = vm.TrainCars.Single(c => c.Id == zeroId); await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        before = Authored(); await Execute(vm.DuplicateCarCommand);
        Require(vm.SelectedCar!.Id != zeroId && vm.SelectedCar.Physical.DryMassKilograms == 350, "Duplicate gets new stable identity with same definition"); await History(before);
        vm.SelectedCar = vm.TrainCars.Single(c => c.Id == zeroId); await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        vm.TrainPrototypeRole = "Trailer"; before = Authored(); await Execute(vm.ApplyCarPrototypeCommand);
        Require(vm.SelectedCar!.Id == zeroId && vm.SelectedCar.Role == "Trailer" && vm.SelectedCar.Physical.DryMassKilograms == 550, "Preset changes values while preserving car identity"); await History(before);
        vm.SelectedCar = vm.TrainCars.Single(c => c.Id == zeroId); await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        const double preciseCoordinate = .12345678901234566;
        await controller.Submit(session => {
            var snapshot = session.Snapshot(); var configuration = snapshot.Train.Configuration;
            var cars = configuration.Cars.ToArray(); var index = Array.FindIndex(cars, c => c.Id == zeroId);
            cars[index] = cars[index] with { Physical = cars[index].Physical with { AerodynamicCenterMeters = [0, preciseCoordinate, 0] } };
            session.ApplyTrain(snapshot.Revision, configuration with { Cars = cars });
        }); vm.Publish(controller.Latest!);
        TrainField Field(string path) => vm.CarFields.Single(f => f.Path == path);
        Field("physical.dryMassKilograms").Value = "765";
        Field("physical.dryCogMeters.2").Value = ".62";
        Field("physical.loadMassKilograms").Value = "99";
        Field("physical.loadCogMeters.0").Value = ".15";
        Field("physical.bogies.0.positionMeters.2").Value = "-.12";
        Field("physical.bogies.1.positionMeters.2").Value = "-.12";
        Field("physical.frontHitchMeters.0").Value = "1.7";
        Field("physical.bodyDimensionsMeters.0").Value = "3.4";
        vm.CarAlignmentFields.Single(f => f.Path == "visualTranslationMeters.2").Value = ".8";
        vm.TrainCarAsset = "assets://train/test-custom.glb"; vm.TrainCarRole = "Custom";
        before = Authored(); var applyWatch = Stopwatch.StartNew(); await Execute(vm.ApplyCarCommand);
        Console.WriteLine($"Train atomic Apply + native validation/rebuild/snapshot: {applyWatch.Elapsed.TotalMilliseconds:F3} ms");
        var authored = vm.SelectedCar!;
        Require(authored.Physical.AerodynamicCenterMeters[1] == preciseCoordinate, "Apply preserves untouched native double precision through form text");
        Require(authored.Physical.DryMassKilograms == 765 && authored.Physical.DryCogMeters[2] == .62
            && authored.Physical.LoadMassKilograms == 99 && authored.Physical.LoadCogMeters[0] == .15
            && authored.Physical.Bogies[0].PositionMeters[2] == -.12 && authored.Physical.FrontHitchMeters[0] == 1.7
            && authored.Physical.BodyDimensionsMeters[0] == 3.4, "Form fields route to native per-car physics");
        Require(authored.VisualAsset == "assets://train/test-custom.glb" && authored.VisualTranslationMeters[2] == .8,
            "GLB assignment and visual alignment persist on matching physical car");
        Require(vm.TrainDiagnostics.Any(d => d.Severity == "WARNING" && d.CarId == zeroId && d.Message.Contains("GLB")), "Missing GLB warning maps from native"); await History(before);
        vm.SelectedCar = vm.TrainCars.Single(c => c.Id == zeroId); await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Field("physical.dryMassKilograms").Value = "-1"; var revision = vm.Snapshot!.Revision;
        await Execute(vm.ValidateCarCommand);
        Require(vm.TrainDiagnostics.Any(d => d.Severity == "ERROR") && vm.Snapshot.Revision == revision, "Proposal errors display without mutating document");
        await vm.ApplyCarCommand.ExecuteAsync(); Require(vm.HasError && vm.Snapshot.Revision == revision, "Invalid Apply preserves native history and revision");
        Field("physical.dryMassKilograms").Value = "765";
        await Execute(vm.BoxInertiaCommand);
        Require(vm.SelectedCar!.Physical.InertiaKgM2[0][0] > 0, "Explicit inertia approximation routes to C++");
        var accepted = vm.Snapshot!;
        vm.ShowTrainCog = false; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require((vm.Snapshot!.Train.Overlays & 8) == 0 && vm.Snapshot.Revision == accepted.Revision, "Preview checkbox is transient native state");
        before = Authored(); await Execute(vm.RemoveCarCommand); await History(before);
        var staged = vm.CarFields; staged[0].Value = "812"; vm.Publish(controller.Latest!);
        Require(ReferenceEquals(staged, vm.CarFields) && vm.CarFields[0].Value == "812", "Heartbeat preserves staged numeric fields");
        vm.WorkspaceMode = 0; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require(!vm.Snapshot!.Train.Active && vm.IsRouteWorkspace, "Leaving Train restores ordinary workspace selection");
        Console.WriteLine($"Train workspace checks passed: {checks}; C# only maps forms and native proposals.");
    }
}
