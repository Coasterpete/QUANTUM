using System.Text.Json;
using Quantum.Editor;
using Quantum.Managed;

internal static class Supports2Checks
{
    private static int checks;
    private static void Require(bool value, string message)
    { ++checks; if (!value) throw new Exception(message); }
    public static async Task Run()
    {
        using var controller = new SessionController();
        await controller.Submit(_ => { });
        var vm = new EditorViewModel(controller); vm.Publish(controller.Latest!);
        vm.WorkspaceMode = 1; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        async Task Execute(EditorCommand command)
        {
            Require(command.CanExecute(null), "Supports 2 command enabled");
            await command.ExecuteAsync(); Require(vm.Error.Length == 0, vm.Error);
        }
        async Task Gesture(EditorCommand command)
        {
            var revision = vm.Snapshot!.Revision;
            await Execute(command);
            Require(vm.Snapshot!.Revision == revision+1, "One form gesture publishes one native revision");
            var support = vm.Snapshot.Supports.Selection;
            var infrastructure = vm.SelectedInfrastructure?.Id;
            await Execute(vm.UndoCommand); await Execute(vm.RedoCommand);
            // Selection is transient: deletion Undo restores data, not a
            // formerly selected object. Reselect restored objects explicitly.
            if (support.Kind != SupportSelectionKind.None
                && vm.SupportStructures.Any(s => s.Id == support.Structure)) await vm.SelectSupportAsync(support);
            if (infrastructure is { } id && vm.InfrastructureItems.Any(i => i.Id == id))
            {
                await controller.Submit(s => s.SelectInfrastructure(s.Snapshot().Revision,id));
                vm.Publish(controller.Latest!);
            }
        }
        vm.SupportFamily = 2; await Gesture(vm.GenerateFrameCommand);
        var structure = vm.SelectedStructure!; var member = vm.SupportMembers[0];
        Require(structure.Appearance is { Family: 2, Frame: not null } && vm.SupportMembers.All(m => m.Shape == 2),
            "HSS frame arrives through native style and resolved-profile snapshots");
        await vm.SelectSupportAsync(new(SupportSelectionKind.Member, structure.Id, member.Id));
        vm.MemberMaterial = 3; vm.MemberRepeat = "2.5"; vm.MemberAcross = ".5";
        await Gesture(vm.ApplyMemberMaterialCommand);
        Require(vm.SelectedSupportMember!.Appearance!.Surface is { Material: "GalvanizedSteel" } surface
            && surface.RepeatMeters[0] == 2.5f, "Member material and physical repeats are native authored values");
        vm.SupportFamily = 3; await Gesture(vm.ApplySupportStyleCommand);
        Require(vm.SelectedSupportMember!.Shape == 2 && vm.SelectedSupportMember.Appearance!.Resolved.Material == "GalvanizedSteel",
            "Family changes preserve explicit profiles and material overrides");
        await Gesture(vm.UseStyleProfileCommand);
        Require(vm.SelectedSupportMember!.Shape == 3, "Explicit opt-in resolves the I-section family profile");
        vm.MemberShape = 0; vm.MemberWidth = ".45"; vm.MemberHeight = ".45"; vm.MemberWall = ".02";
        await Gesture(vm.ApplySupportProfileCommand);
        Require(vm.SelectedSupportMember!.Shape == 0 && !vm.SelectedSupportMember.Appearance!.UseStyleProfile,
            "Profile form restores an explicit circular override");
        vm.FrameWidth = "5"; await Gesture(vm.RegenerateFrameCommand);
        Require(vm.SupportMembers.First(m => m.Id == member.Id).Shape == 0, "Regeneration preserves edited profile and member ID");
        await vm.SelectSupportAsync(new(SupportSelectionKind.Node, structure.Id, vm.SupportNodes[0].Id));
        vm.FoundationKind = 2; vm.FoundationWidth = "1"; vm.FoundationDepth = "1"; vm.FoundationHeight = "2";
        await Gesture(vm.ApplyFoundationShapeCommand);
        Require(vm.SelectedStructure!.Appearance!.Foundations[0] is { Kind: 2 }, "Foundation form persists pier type");
        vm.WorkspaceMode = 0; vm.TimelineSource = 1;
        vm.SelectedHardwareChoice = vm.HardwareChoices.First(c => c.Kind == DeviceKind.Access);
        await Execute(vm.CreateHardwareCommand); var gate = vm.SelectedDevice!.Id;
        vm.GateWidth = "1.4"; vm.GateHeight = "1.6"; vm.GateInfill = 1; vm.GateInitiallyLocked = false;
        await Gesture(vm.ApplyHardwareCommand);
        Require(vm.SelectedHardwareRuntime!.Settings(new AccessSettings()) is { WidthMeters: 1.4, HeightMeters: 1.6, Infill: 1 },
            "Gate physical settings remain on the existing Access hardware form");
        vm.WorkspaceMode = 1;
        vm.InfrastructureGates = gate.ToString(); await Gesture(vm.CreateInfrastructureCommand);
        var infrastructure = vm.SelectedInfrastructure!.Id;
        Require(vm.SelectedInfrastructure.AccessGates!.Single() == gate && vm.SelectedInfrastructure.Surface!.Material == "GalvanizedSteel",
            "Catwalk form stores existing Access references and its independent material");
        vm.InfrastructureWidth = "1.2"; await Gesture(vm.ApplyInfrastructureCommand);
        Require(vm.SelectedInfrastructure!.WidthMeters == 1.2, "Catwalk parameter edit survives Undo/Redo");
        vm.InfrastructureKind = 1; vm.InfrastructureWorld = true; vm.InfrastructureWorldStart = "2, 4, 0";
        vm.InfrastructureWorldEnd = "6, 4, 0"; await Gesture(vm.CreateInfrastructureCommand);
        Require(vm.SelectedInfrastructure is { Kind: 1, WorldPlacement: true }, "World platform creation");
        vm.InfrastructureKind = 3; vm.InfrastructureWorldEnd = "6, 4, 2"; await Gesture(vm.CreateInfrastructureCommand);
        Require(vm.SelectedInfrastructure!.Kind == 3, "Explicit endpoint stair creation");
        var deleted = vm.SelectedInfrastructure.Id; await Gesture(vm.DeleteInfrastructureCommand);
        Require(vm.InfrastructureItems.All(i => i.Id != deleted), "Deleted infrastructure selection clears");
        await Execute(vm.UndoCommand); Require(vm.InfrastructureItems.Any(i => i.Id == deleted), "Delete Undo restores authored infrastructure ID");
        // Bad boundary JSON is a validation error and must leave history intact.
        await controller.Submit(s =>
        {
            var before = s.Snapshot().Revision;
            foreach (var operation in new[] { SupportOperation.SetStructureSurface, SupportOperation.GenerateFrame })
            {
                try { s.EditSupport(before, new(operation, structure.Id, Text: "{")); throw new Exception("Malformed JSON accepted"); }
                catch (QuantumException error) { Require(error.Code == 2, "Malformed JSON is translated to invalid argument"); }
                Require(s.Snapshot().Revision == before, "Rejected material/frame edit preserves the revision");
            }
        });
        var temporary = Path.Combine(Path.GetTempPath(), $"quantum-supports-2-{Guid.NewGuid():N}.quantum");
        try
        {
            await controller.Submit(s => { s.Save(temporary); s.New(); s.Open(temporary); });
            vm.Publish(controller.Latest!);
            Require(vm.Snapshot!.Hardware.SelectedInfrastructure == 0,
                "New/Open clears infrastructure selection even when another document reuses its local ID");
            Require(vm.SupportStructures.Single().Appearance!.Family == 3 && vm.InfrastructureItems.Count == 3,
                "Styles, frame provenance, platform/stairs and catwalk survive save/reload");
            Require(vm.Snapshot!.Supports.Members.First(m => m.Id == member.Id).Appearance!.Surface!.RepeatMeters[0] == 2.5f
                && vm.InfrastructureItems.First(i => i.Id == infrastructure).AccessGates!.Single() == gate,
                "UV override and hardware references survive save/reload");
        }
        finally { File.Delete(temporary); }
        Console.WriteLine($"Managed Supports 2: {checks} checks passed.");
    }
}
