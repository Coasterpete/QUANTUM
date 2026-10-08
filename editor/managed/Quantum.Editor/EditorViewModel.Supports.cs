using System.Globalization;
using Quantum.Managed;

namespace Quantum.Editor;

public sealed partial class EditorViewModel
{
    private int workspaceMode;
    private bool updatingSupports, supportSelectionPending;
    private long supportSelectionRequest;
    private SupportSelection supportSelection = new(SupportSelectionKind.None, 0);
    private SupportStructure? selectedStructure;
    private SupportNode? selectedNode, memberStartNode, memberEndNode;
    private SupportMember? selectedMember;
    private string structureName = "", nodeX = "", nodeY = "", nodeZ = "";
    private string attachmentStation = "0", attachmentLateral = "0", attachmentVertical = "0";
    private string memberWidth = ".1", memberHeight = ".1", memberWall = "0", connectorAsset = "";
    private int memberShape, connectionEnd, connectionTreatment;
    public int WorkspaceMode
    {
        get => workspaceMode;
        set
        {
            if (workspaceMode == value) return;
            CancelForSelection(); workspaceMode = value;
            Changed(); Changed(nameof(IsSupportsWorkspace)); Changed(nameof(IsRouteWorkspace));
            Changed(nameof(IsTrainWorkspace));
            Changed(nameof(ViewportInstructions)); RefreshCommands();
            SynchronizeSupportWorkspace(); SynchronizeDeviceSelection();
            SynchronizeTrainView();
        }
    }
    public bool IsSupportsWorkspace => WorkspaceMode == 1;
    public bool IsRouteWorkspace => WorkspaceMode == 0;
    public string ViewportInstructions => IsTrainWorkspace
        ? "Left click: physical car envelope · yellow: selected car · magenta: loaded COG · RGB: local frame · native two-bogie model"
        : IsSupportsWorkspace
        ? "Left click: select node / member · Right drag: orbit · Middle drag: pan · Wheel: zoom · Numeric move in Inspector"
        : "Diamond: Track Start · Move / Rotate in Track Layout · Right drag: orbit · Middle drag: pan · Wheel: zoom · F: frame · Esc: cancel";
    public IReadOnlyList<SupportStructure> SupportStructures { get; private set; } = [];
    public IReadOnlyList<SupportNode> SupportNodes { get; private set; } = [];
    public IReadOnlyList<SupportMember> SupportMembers { get; private set; } = [];
    public SupportStructure? SelectedStructure
    {
        get => selectedStructure;
        set { if (!updatingSupports) ChooseSupport(value is null ? new(SupportSelectionKind.None, 0) : new(SupportSelectionKind.Structure, value.Id)); }
    }
    public SupportNode? SelectedSupportNode
    {
        get => selectedNode;
        set { if (!updatingSupports && value is not null) ChooseSupport(new(SupportSelectionKind.Node, value.Structure, value.Id)); }
    }
    public SupportMember? SelectedSupportMember
    {
        get => selectedMember;
        set { if (!updatingSupports && value is not null) ChooseSupport(new(SupportSelectionKind.Member, value.Structure, value.Id)); }
    }
    public SupportNode? MemberStartNode { get => memberStartNode; set { memberStartNode = value; Changed(); RefreshCommands(); } }
    public SupportNode? MemberEndNode { get => memberEndNode; set { memberEndNode = value; Changed(); RefreshCommands(); } }
    public string StructureName { get => structureName; set { structureName = value; Changed(); } }
    public string NodeX { get => nodeX; set { nodeX = value; Changed(); } }
    public string NodeY { get => nodeY; set { nodeY = value; Changed(); } }
    public string NodeZ { get => nodeZ; set { nodeZ = value; Changed(); } }
    public string AttachmentStation { get => attachmentStation; set { attachmentStation = value; Changed(); } }
    public string AttachmentLateral { get => attachmentLateral; set { attachmentLateral = value; Changed(); } }
    public string AttachmentVertical { get => attachmentVertical; set { attachmentVertical = value; Changed(); } }
    public string MemberWidth { get => memberWidth; set { memberWidth = value; Changed(); } }
    public string MemberHeight { get => memberHeight; set { memberHeight = value; Changed(); } }
    public string MemberWall { get => memberWall; set { memberWall = value; Changed(); } }
    public int MemberShape { get => memberShape; set { memberShape = value; Changed(); Changed(nameof(IsRectangularMember)); } }
    public bool IsRectangularMember => MemberShape != 0;
    public int ConnectionEnd { get => connectionEnd; set { connectionEnd = value; Changed(); RefreshConnection(); } }
    public int ConnectionTreatment { get => connectionTreatment; set { connectionTreatment = value; Changed(); } }
    public string ConnectorAsset { get => connectorAsset; set { connectorAsset = value; Changed(); } }
    public bool HasSupportStructure => selectedStructure is not null;
    public bool HasSupportNode => selectedNode is not null;
    public bool HasSupportMember => selectedMember is not null;
    public bool IsFreeSupportNode => selectedNode?.Anchor == 0;
    public bool IsAttachedSupportNode => selectedNode?.Anchor == 1;
    public bool IsFoundationSupportNode => selectedNode?.Anchor == 2;
    public bool CanMoveSupportNode => selectedNode is { Anchor: not 1 };
    private SupportConnection? ActiveConnection => ConnectionEnd == 0 ? selectedMember?.StartConnection : selectedMember?.EndConnection;
    public bool HasConnection => ActiveConnection?.Treatment > 0;
    public bool HasConnectionPlacement => ActiveConnection?.HasPlacement == true;
    public bool CanEditConnectorAsset => HasConnection && ActiveConnection?.AssetTruncated == false;
    public string SupportHeading => selectedNode?.Label ?? selectedMember?.Label ?? selectedStructure?.Label ?? "Support authoring";
    public string SupportContext => selectedStructure?.Summary ?? "Create or select a support structure";
    public string SupportSummary => Snapshot is { } s
        ? $"{s.Supports.Structures.Count} structures · {s.Supports.Nodes.Count} nodes · {s.Supports.Members.Count} members" : "Loading…";
    public string ConnectionContext
    {
        get
        {
            if (selectedMember is not { } member) return "";
            var id = ConnectionEnd == 0 ? member.StartNode : member.EndNode;
            var node = SupportNodes.FirstOrDefault(n => n.Id == id);
            return $"{(ConnectionEnd == 0 ? "Start" : "End")} → Node #{id} · {node?.AnchorLabel}\n"
                + "Saddle / Clamp require Track. Base / Footing require Foundation.";
        }
    }
    public string ConnectionMetadata => ActiveConnection is { } connection
        ? $"{(connection.HasPlacement ? "Authored local placement. " : "")}{(connection.HasMounting ? "Authored mounting retained. " : "")}{(connection.AssetTruncated ? "Long asset shown truncated; edit in fallback. " : "")}" : "";
    public string MemberMetadata => selectedMember is { } m
        ? $"Native role: {RoleName(m.Role)} · orientation: {OrientationName(m.Orientation)}{(m.HasReference ? " · directed reference retained" : "")}" : "";
    private static string RoleName(uint role) => role switch { 1 => "Primary post", 2 => "Ledger / cap", 3 => "Longitudinal tie", 4 => "Brace", 5 => "Track support", _ => "Unspecified" };
    private static string OrientationName(uint orientation) => orientation switch { 1 => "Bent post", 2 => "Bent transverse", 3 => "Run longitudinal", 4 => "Bent diagonal", 5 => "Run diagonal", _ => "Generic" };

    public EditorCommand CreateSupportStructureCommand { get; private set; } = null!;
    public EditorCommand DeleteSupportStructureCommand { get; private set; } = null!;
    public EditorCommand RenameSupportStructureCommand { get; private set; } = null!;
    public EditorCommand CreateSupportNodeCommand { get; private set; } = null!;
    public EditorCommand DeleteSupportNodeCommand { get; private set; } = null!;
    public EditorCommand MoveSupportNodeCommand { get; private set; } = null!;
    public EditorCommand AttachSupportNodeCommand { get; private set; } = null!;
    public EditorCommand DetachSupportNodeCommand { get; private set; } = null!;
    public EditorCommand SetSupportFoundationCommand { get; private set; } = null!;
    public EditorCommand ClearSupportFoundationCommand { get; private set; } = null!;
    public EditorCommand CreateSupportMemberCommand { get; private set; } = null!;
    public EditorCommand DeleteSupportMemberCommand { get; private set; } = null!;
    public EditorCommand ApplySupportProfileCommand { get; private set; } = null!;
    public EditorCommand ApplySupportTreatmentCommand { get; private set; } = null!;
    public EditorCommand ApplySupportAssetCommand { get; private set; } = null!;
    public EditorCommand ClearSupportPlacementCommand { get; private set; } = null!;
    public EditorCommand FrameSupportCommand { get; private set; } = null!;
    private void InitializeSupportCommands()
    {
        InitializeSupportAppearanceCommands();
        CreateSupportStructureCommand = Add(() => SupportMutation(() => new(SupportOperation.CreateStructure)), () => Snapshot is not null);
        DeleteSupportStructureCommand = Add(() => SupportMutation(() => new(SupportOperation.DeleteStructure, selectedStructure!.Id)), () => HasSupportStructure);
        RenameSupportStructureCommand = Add(() => SupportMutation(() => new(SupportOperation.RenameStructure, selectedStructure!.Id, Text: StructureName)), () => HasSupportStructure);
        CreateSupportNodeCommand = Add(() => SupportMutation(() => new(SupportOperation.CreateNode, selectedStructure!.Id,
            X: (selectedNode?.ResolvedX ?? 0) + (selectedNode is null ? 0 : 1), Y: selectedNode?.ResolvedY ?? 0, Z: selectedNode?.ResolvedZ ?? 0)), () => HasSupportStructure);
        DeleteSupportNodeCommand = Add(() => SupportMutation(() => NodeRequest(SupportOperation.DeleteNode)), () => HasSupportNode);
        MoveSupportNodeCommand = Add(() => SupportMutation(() => NodeRequest(SupportOperation.MoveNode) with
            { X = Finite(NodeX, "X"), Y = Finite(NodeY, "Y"), Z = Finite(NodeZ, "Z") }), () => CanMoveSupportNode);
        AttachSupportNodeCommand = Add(() => SupportMutation(() => NodeRequest(SupportOperation.SetAttachment) with
            { X = Finite(AttachmentStation, "Station"), Y = Finite(AttachmentLateral, "Lateral offset"), Z = Finite(AttachmentVertical, "Vertical offset") }), () => HasSupportNode && !IsFoundationSupportNode);
        DetachSupportNodeCommand = Add(() => SupportMutation(() => NodeRequest(SupportOperation.ClearAttachment)), () => IsAttachedSupportNode);
        SetSupportFoundationCommand = Add(() => SupportMutation(() => NodeRequest(SupportOperation.SetFoundation)), () => IsFreeSupportNode);
        ClearSupportFoundationCommand = Add(() => SupportMutation(() => NodeRequest(SupportOperation.ClearFoundation)), () => IsFoundationSupportNode);
        CreateSupportMemberCommand = Add(() => SupportMutation(() => new(SupportOperation.CreateMember, selectedStructure!.Id,
            StartNode: memberStartNode!.Id, EndNode: memberEndNode!.Id, Text: "{\"useStyleProfile\":true}")), () => HasSupportStructure && memberStartNode is not null
                && memberEndNode is not null && memberStartNode.Id != memberEndNode.Id);
        DeleteSupportMemberCommand = Add(() => SupportMutation(() => MemberRequest(SupportOperation.DeleteMember)), () => HasSupportMember);
        ApplySupportProfileCommand = Add(() => SupportMutation(() => MemberRequest(SupportOperation.SetProfile) with
            { Shape = (uint)MemberShape, Width = Finite(MemberWidth, "Width / diameter"),
                Height = Finite(IsRectangularMember ? MemberHeight : MemberWidth, "Height"), Wall = Finite(MemberWall, "Wall thickness") }), () => HasSupportMember);
        ApplySupportTreatmentCommand = Add(() => SupportMutation(() => MemberRequest(ConnectionTreatment == 0 ? SupportOperation.ClearConnection : SupportOperation.SetTreatment) with
            { Treatment = (uint)Math.Max(0, ConnectionTreatment - 1) }), () => HasSupportMember);
        ApplySupportAssetCommand = Add(() => SupportMutation(() => MemberRequest(SupportOperation.SetConnectorAsset) with { Text = ConnectorAsset }), () => CanEditConnectorAsset);
        ClearSupportPlacementCommand = Add(() => SupportMutation(() => MemberRequest(SupportOperation.ClearConnectionPlacement)), () => HasConnectionPlacement);
        FrameSupportCommand = Add(() => Run(() => { Controller.Session.Camera(ViewportAction.FrameSupport); return Task.CompletedTask; }), () => HasSupportStructure);
    }
    private SupportRequest NodeRequest(SupportOperation operation) => new(operation, selectedNode!.Structure, selectedNode.Id);
    private SupportRequest MemberRequest(SupportOperation operation) => new(operation, selectedMember!.Structure, selectedMember.Id, MemberEnd: (uint)ConnectionEnd);
    private static double Finite(string text, string name)
    {
        if (!double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out var value) || !double.IsFinite(value))
            throw new ArgumentException($"{name} must be a finite number.");
        return value;
    }
    private Task SupportMutation(Func<SupportRequest> makeRequest) => Run(async () =>
    {
        if (Snapshot is not { } snapshot) return;
        var request = makeRequest();
        await Controller.Submit(s =>
        {
            var id = s.EditSupport(snapshot.Revision, request);
            if (id == 0) return;
            var kind = request.Operation switch { SupportOperation.CreateStructure or SupportOperation.GenerateFrame => SupportSelectionKind.Structure,
                SupportOperation.CreateNode => SupportSelectionKind.Node, _ => SupportSelectionKind.Member };
            s.SelectSupport(s.Snapshot().Revision, new(kind, kind == SupportSelectionKind.Structure ? id : request.Structure,
                kind == SupportSelectionKind.Structure ? 0 : id));
        });
    });
    private void ChooseSupport(SupportSelection selection)
    {
        _ = SelectSupportAsync(selection);
    }
    public async Task SelectSupportAsync(SupportSelection selection)
    {
        if (Snapshot is not { } snapshot || selection == supportSelection) return;
        CancelForSelection(); ApplySupportSelection(selection, true);
        var request = ++supportSelectionRequest; supportSelectionPending = true;
        try { await Controller.Submit(s => s.SelectSupport(snapshot.Revision, selection)); }
        catch (QuantumException failure) { if (failure.Code != 8) Error = failure.Message; }
        catch (ObjectDisposedException) { }
        catch (TaskCanceledException) { }
        finally
        {
            if (request == supportSelectionRequest)
            {
                supportSelectionPending = false;
                if (Controller.Latest is { } latest) Publish(latest);
            }
        }
    }
    private async void SynchronizeSupportWorkspace()
    {
        var active = IsSupportsWorkspace;
        try { await Controller.Submit(s => s.SetSupportWorkspace(active)); }
        catch (ObjectDisposedException) { }
        catch (TaskCanceledException) { }
        catch (Exception failure) { Error = failure.Message; }
    }
    private void PublishSupports(SupportSnapshot next, bool documentChanged)
    {
        updatingSupports = true;
        try
        {
            if (documentChanged) { SupportStructures = next.Structures; Changed(nameof(SupportStructures)); }
            var choice = supportSelectionPending ? supportSelection : next.Selection;
            ApplySupportSelection(choice, documentChanged || supportSelection != choice);
            Changed(nameof(SupportSummary));
        }
        finally { updatingSupports = false; }
    }
    private void ApplySupportSelection(SupportSelection choice, bool refresh)
    {
        if (!refresh) return;
        var previousUpdating = updatingSupports; updatingSupports = true;
        try
        {
            var previousStructure = selectedStructure?.Id;
            var start = memberStartNode?.Id; var end = memberEndNode?.Id;
            selectedStructure = SupportStructures.FirstOrDefault(s => s.Id == choice.Structure);
            if (selectedStructure is null) choice = new(SupportSelectionKind.None, 0);
            SupportNodes = Snapshot?.Supports.Nodes.Where(n => n.Structure == choice.Structure).ToArray() ?? [];
            SupportMembers = Snapshot?.Supports.Members.Where(m => m.Structure == choice.Structure).ToArray() ?? [];
            selectedNode = choice.Kind == SupportSelectionKind.Node ? SupportNodes.FirstOrDefault(n => n.Id == choice.Element) : null;
            selectedMember = choice.Kind == SupportSelectionKind.Member ? SupportMembers.FirstOrDefault(m => m.Id == choice.Element) : null;
            if ((choice.Kind == SupportSelectionKind.Node && selectedNode is null) || (choice.Kind == SupportSelectionKind.Member && selectedMember is null))
                choice = new(SupportSelectionKind.Structure, choice.Structure);
            supportSelection = choice;
            Changed(nameof(SupportNodes)); Changed(nameof(SupportMembers)); Changed(nameof(SelectedStructure));
            Changed(nameof(SelectedSupportNode)); Changed(nameof(SelectedSupportMember));
            MemberStartNode = SupportNodes.FirstOrDefault(n => previousStructure == choice.Structure && n.Id == start) ?? selectedNode ?? SupportNodes.FirstOrDefault();
            MemberEndNode = SupportNodes.FirstOrDefault(n => previousStructure == choice.Structure && n.Id == end) ?? SupportNodes.FirstOrDefault(n => n.Id != memberStartNode?.Id);
            StructureName = selectedStructure?.Name ?? "";
            NodeX = Format(selectedNode?.X); NodeY = Format(selectedNode?.Y); NodeZ = Format(selectedNode?.Z);
            AttachmentStation = Format(selectedNode?.Station ?? 0); AttachmentLateral = Format(selectedNode?.Lateral ?? 0); AttachmentVertical = Format(selectedNode?.Vertical ?? 0);
            MemberShape = (int)(selectedMember?.Shape ?? 0); MemberWidth = Format(selectedMember?.Width ?? .1);
            MemberHeight = Format(selectedMember?.Height ?? .1); MemberWall = Format(selectedMember?.Wall ?? 0);
            foreach (var name in new[] { nameof(HasSupportStructure), nameof(HasSupportNode), nameof(HasSupportMember), nameof(IsFreeSupportNode),
                nameof(IsAttachedSupportNode), nameof(IsFoundationSupportNode), nameof(CanMoveSupportNode), nameof(SupportHeading), nameof(SupportContext), nameof(MemberMetadata) }) Changed(name);
            SeedSupportAppearance(); RefreshConnection(); RefreshCommands();
        }
        finally { updatingSupports = previousUpdating; }
    }
    private static string Format(double? value) => value?.ToString("G12", CultureInfo.CurrentCulture) ?? "";
    private void RefreshConnection()
    {
        ConnectionTreatment = (int)(ActiveConnection?.Treatment ?? 0); ConnectorAsset = ActiveConnection?.Asset ?? "";
        Changed(nameof(ConnectionContext)); Changed(nameof(ConnectionMetadata)); Changed(nameof(HasConnection));
        Changed(nameof(HasConnectionPlacement)); Changed(nameof(CanEditConnectorAsset)); RefreshCommands();
    }
}
