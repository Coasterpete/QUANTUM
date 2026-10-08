using System.Diagnostics;
using System.Globalization;
using Quantum.Editor;
using Quantum.Managed;

internal static class SupportChecks
{
    private static int checks;
    private static void Require(bool condition, string message) { ++checks; if (!condition) throw new Exception(message); }
    private static async Task Execute(EditorCommand command, EditorViewModel vm)
    {
        Require(command.CanExecute(null), "Support command enabled");
        await command.ExecuteAsync(); Require(vm.Error.Length == 0, vm.Error);
    }
    private static void Rejected(Action operation, int code)
    {
        try { operation(); } catch (QuantumException error) { Require(error.Code == code, "Native support rejection code"); return; }
        throw new Exception("Invalid support operation was accepted");
    }
    public static async Task Run()
    {
        using var controller = new SessionController();
        await controller.Submit(_ => { });
        var vm = new EditorViewModel(controller); vm.Publish(controller.Latest!);
        vm.WorkspaceMode = 1; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require(vm.IsSupportsWorkspace && controller.Latest!.Supports.Active, "Supports workspace reaches native viewport mode");
        // Model the synchronous selection clearing caused by ItemsSource changes.
        vm.PropertyChanged += (_, e) =>
        {
            if (e.PropertyName == nameof(vm.SupportStructures)) vm.SelectedStructure = null;
            if (e.PropertyName == nameof(vm.SupportNodes)) vm.SelectedSupportNode = null;
            if (e.PropertyName == nameof(vm.SupportMembers)) vm.SelectedSupportMember = null;
        };
        await Execute(vm.CreateSupportStructureCommand, vm);
        var structure = vm.SelectedStructure!;
        Require(structure.Id > 0 && structure.NodeCount == 0, "Create selects native structure despite ItemsSource replacement");
        vm.StructureName = "Frame é"; await Execute(vm.RenameSupportStructureCommand, vm);
        Require(vm.SelectedStructure!.Id == structure.Id && vm.SelectedStructure.Name == "Frame é", "Rename keeps ID and UTF-8 name");
        await Execute(vm.CreateSupportNodeCommand, vm); var a = vm.SelectedSupportNode!;
        vm.NodeZ = "-5"; await Execute(vm.MoveSupportNodeCommand, vm);
        await Execute(vm.SetSupportFoundationCommand, vm);
        Require(vm.IsFoundationSupportNode && !vm.AttachSupportNodeCommand.CanExecute(null), "Foundation is exclusive of track attachment");
        await Execute(vm.CreateSupportNodeCommand, vm); var b = vm.SelectedSupportNode!;
        Require(b.Id != a.Id && b.Anchor == 0 && vm.SelectedStructure!.NodeCount == 2, "New node receives own native ID and no copied anchor");
        vm.AttachmentStation = "10"; vm.AttachmentLateral = "1"; vm.AttachmentVertical = "-0.5";
        await Execute(vm.AttachSupportNodeCommand, vm);
        Require(vm.IsAttachedSupportNode && !vm.MoveSupportNodeCommand.CanExecute(null)
            && vm.SelectedSupportNode!.Station == 10 && vm.SelectedSupportNode.ResolvedX != vm.SelectedSupportNode.X,
            "Attachment edits supply native resolved position with fallback XYZ read-only");
        var rev = vm.Snapshot!.Revision;
        vm.AttachmentLateral = "2"; await Execute(vm.AttachSupportNodeCommand, vm);
        Require(vm.Snapshot!.Supports.PresentationRevision > rev && vm.SelectedSupportNode!.Lateral == 2,
            "Offset edits refresh native support presentation");
        await Execute(vm.DetachSupportNodeCommand, vm);
        Require(vm.IsFreeSupportNode && vm.SelectedSupportNode!.ResolvedX == vm.SelectedSupportNode.X, "Detach restores authored fallback position");
        await Execute(vm.UndoCommand, vm); Require(vm.IsAttachedSupportNode, "Attachment Undo restores node form");
        vm.MemberStartNode = vm.SupportNodes.First(n => n.Id == a.Id); vm.MemberEndNode = vm.SupportNodes.First(n => n.Id == b.Id);
        await Execute(vm.CreateSupportMemberCommand, vm); var member = vm.SelectedSupportMember!;
        Require(member.StartNode == a.Id && member.EndNode == b.Id && !vm.HasSupportNode, "Member selection is exclusive and endpoint-directed");
        vm.MemberShape = 1; vm.MemberWidth = "0.3"; vm.MemberHeight = "0.4"; vm.MemberWall = "0.02";
        await Execute(vm.ApplySupportProfileCommand, vm);
        Require(vm.SelectedSupportMember!.Shape == 1 && vm.SelectedSupportMember.Height == .4, "Native rectangular profile and dimensions");
        vm.ConnectionEnd = 0; vm.ConnectionTreatment = 9; await Execute(vm.ApplySupportTreatmentCommand, vm);
        Require(vm.SelectedSupportMember!.StartConnection.Treatment == 9, "Foundation Footing at start end");
        vm.ConnectionEnd = 1; vm.ConnectionTreatment = 6; await Execute(vm.ApplySupportTreatmentCommand, vm);
        Require(vm.SelectedSupportMember!.EndConnection.Treatment == 6 && vm.SelectedSupportMember.StartConnection.Treatment == 9,
            "Track Saddle at end keeps start treatment independent");
        vm.ConnectorAsset = "assets://support/saddle.glb"; await Execute(vm.ApplySupportAssetCommand, vm);
        Require(vm.SelectedSupportMember!.EndConnection.Asset == "assets://support/saddle.glb", "Connector asset command stays native");
        var before = vm.Snapshot!;
        vm.MemberWall = "1"; await vm.ApplySupportProfileCommand.ExecuteAsync();
        Require(vm.Error.Length > 0 && vm.Snapshot!.Revision == before.Revision && vm.MemberWall == "0.02", "Invalid form edit preserves snapshot and restores authoritative form");
        vm.Error = "";
        await controller.Submit(s =>
        {
            Rejected(() => s.EditSupport(before.Revision - 1, new(SupportOperation.DeleteStructure, structure.Id)), 8);
            Rejected(() => s.EditSupport(before.Revision, new(SupportOperation.SetTreatment, structure.Id, member.Id,
                MemberEnd: 0, Treatment: 5)), 2);
        });
        vm.Publish(controller.Latest!);
        await vm.SelectSupportAsync(new(SupportSelectionKind.Node, structure.Id, b.Id));
        Require(vm.SelectedSupportNode!.Id == b.Id && !vm.HasSupportMember && controller.Latest!.Supports.Selection.Kind == SupportSelectionKind.Node,
            "Browser selection synchronizes native ID, node form and member clearing");
        vm.NodeX = "uncommitted draft"; vm.Publish(vm.Snapshot!);
        Require(vm.NodeX == "uncommitted draft", "Telemetry-only publication preserves numeric drafts");
        await controller.Submit(s => s.SelectSupport(s.Snapshot().Revision, new(SupportSelectionKind.Member, structure.Id, member.Id)));
        vm.Publish(controller.Latest!);
        Require(vm.SelectedSupportMember!.Id == member.Id && !vm.HasSupportNode, "Native viewport-style selection updates browser/Inspector");
        vm.WorkspaceMode = 0; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        Require(!controller.Latest!.Supports.Active && vm.SelectedSupportMember!.Id == member.Id, "Route workspace preserves support identity");
        vm.WorkspaceMode = 1; await controller.Submit(_ => { }); vm.Publish(controller.Latest!);
        var snapshot = vm.Snapshot!;
        var watch = Stopwatch.StartNew();
        await controller.Submit(s =>
        {
            for (int i = 0; i < 1000; ++i)
                Require(ReferenceEquals(snapshot.Supports.Nodes, s.Snapshot().Supports.Nodes), "Managed support arrays stay cached");
        });
        Console.WriteLine($"Cached support snapshots: {watch.Elapsed.TotalMilliseconds / 1000:F4} ms/call");
        var temporary = Path.Combine(Path.GetTempPath(), $"quantum-managed-supports-{Guid.NewGuid():N}.quantum");
        try
        {
            vm.PickFile = _ => Task.FromResult<string?>(temporary); vm.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            await Execute(vm.SaveAsCommand, vm); Require(!vm.Snapshot!.Dirty, "Native save marks support state clean");
            await Execute(vm.NewCommand, vm);
            Require(vm.SupportStructures.Count == 0 && !vm.HasSupportNode && !vm.HasSupportMember, "New clears support browser/selection");
            await Execute(vm.OpenCommand, vm);
            await vm.SelectSupportAsync(new(SupportSelectionKind.Member, structure.Id, member.Id));
            Require(vm.SupportNodes.Count == 2 && vm.SelectedSupportMember!.Shape == 1
                && vm.SelectedSupportMember.EndConnection.Asset == "assets://support/saddle.glb"
                && vm.SupportNodes.First(n => n.Id == b.Id).Anchor == 1, "Coherent Supports save/reload restores authored state");
            await Execute(vm.DeleteSupportMemberCommand, vm); Require(!vm.HasSupportMember, "Delete member clears selected Inspector");
            await Execute(vm.UndoCommand, vm); Require(vm.SupportMembers.Any(m => m.Id == member.Id), "Delete Undo restores stable member ID");
            await vm.SelectSupportAsync(new(SupportSelectionKind.Member, structure.Id, member.Id));
            await Execute(vm.DeleteSupportMemberCommand, vm);
            await vm.SelectSupportAsync(new(SupportSelectionKind.Node, structure.Id, a.Id));
            await Execute(vm.DeleteSupportNodeCommand, vm); Require(!vm.HasSupportNode, "Delete node clears native selection");
            await vm.SelectSupportAsync(new(SupportSelectionKind.Structure, structure.Id));
            await Execute(vm.DeleteSupportStructureCommand, vm); Require(vm.SupportStructures.Count == 0, "Structure deletion removes contents");
            await Execute(vm.UndoCommand, vm); Require(vm.SupportStructures.Count == 1 && vm.SupportStructures[0].Id == structure.Id, "Structure deletion Undo");
        }
        finally { File.Delete(temporary); }
        // Build a representative document through the same real boundary.
        await controller.Submit(s =>
        {
            s.New();
            for (int i = 0; i < 3; ++i)
            {
                var sid = s.EditSupport(s.Snapshot().Revision, new(SupportOperation.CreateStructure)); uint previous = 0;
                for (int n = 0; n < 24; ++n)
                {
                    var id = s.EditSupport(s.Snapshot().Revision, new(SupportOperation.CreateNode, sid, X: n, Y: i, Z: -5));
                    if (previous > 0) s.EditSupport(s.Snapshot().Revision, new(SupportOperation.CreateMember, sid, StartNode: previous, EndNode: id));
                    previous = id;
                }
            }
        });
        vm.Publish(controller.Latest!);
        await vm.SelectSupportAsync(new(SupportSelectionKind.Structure, vm.SupportStructures[1].Id));
        Require(vm.Snapshot!.Supports.Nodes.Count == 72 && vm.Snapshot.Supports.Members.Count == 69
            && vm.SupportNodes.Count == 24 && vm.SupportMembers.Count == 23, "Representative sets populate structure-scoped lists");
        Console.WriteLine($"Managed Supports: {checks} checks passed.");
    }
}
