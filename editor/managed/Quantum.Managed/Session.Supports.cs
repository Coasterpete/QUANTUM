using System.Collections.ObjectModel;

namespace Quantum.Managed;

public enum SupportSelectionKind : uint { None, Structure, Node, Member }
public enum SupportOperation : uint
{
    CreateStructure, DeleteStructure, RenameStructure, CreateNode, DeleteNode,
    MoveNode, SetAttachment, ClearAttachment, SetFoundation, ClearFoundation,
    CreateMember, DeleteMember, SetProfile, SetTreatment, ClearConnection,
    SetConnectorAsset, ClearConnectionPlacement
}
public sealed record SupportSelection(SupportSelectionKind Kind, uint Structure, uint Element = 0);
public sealed record SupportStructure(uint Id, string Name, uint NodeCount, uint MemberCount, bool Generated)
{
    public string Label => $"{Name} · #{Id}";
    public string Summary => $"{NodeCount} nodes · {MemberCount} members{(Generated ? " · Generated run" : "")}";
    public override string ToString() => Label;
}
public sealed record SupportNode(uint Structure, uint Id, uint Anchor, double X, double Y, double Z,
    double ResolvedX, double ResolvedY, double ResolvedZ, double Station, double Lateral, double Vertical)
{
    public string AnchorLabel => Anchor switch { 1 => "Track attached", 2 => "Foundation", _ => "Free" };
    public string Label => $"Node #{Id} · {AnchorLabel}";
    public string ResolvedPosition => $"{ResolvedX:F3}, {ResolvedY:F3}, {ResolvedZ:F3} m";
    public override string ToString() => Label;
}
public sealed record SupportConnection(uint Treatment, bool HasPlacement, bool HasMounting, bool AssetTruncated, string Asset);
public sealed record SupportMember(uint Structure, uint Id, uint StartNode, uint EndNode, uint Shape,
    uint Role, uint Orientation, bool HasReference, double Width, double Height, double Wall,
    SupportConnection StartConnection, SupportConnection EndConnection)
{
    public string Label => $"Member #{Id} · {StartNode} → {EndNode}";
    public string Summary => $"{(Shape == 0 ? "Circular" : "Rectangular")} · {Width:G5} × {Height:G5} m · wall {Wall:G5} m";
    public override string ToString() => Label;
}
// Immutable display copies, never an editable managed graph or serializer.
public sealed record SupportSnapshot(ulong Revision, ulong SelectionRevision, ulong PresentationRevision,
    bool Active, SupportSelection Selection, ReadOnlyCollection<SupportStructure> Structures,
    ReadOnlyCollection<SupportNode> Nodes, ReadOnlyCollection<SupportMember> Members);
public sealed record SupportRequest(SupportOperation Operation, uint Structure = 0, uint Element = 0,
    uint StartNode = 0, uint EndNode = 0, uint Shape = 0, uint MemberEnd = 0, uint Treatment = 0,
    double X = 0, double Y = 0, double Z = 0, double Width = .1, double Height = .1, double Wall = 0, string? Text = null);

public sealed unsafe partial class Session
{
    private ulong supportsRevision;
    private NativeSupportStructure[] structureBuffer = [];
    private NativeSupportNode[] nodeBuffer = [];
    private NativeSupportMember[] memberBuffer = [];
    private ReadOnlyCollection<SupportStructure> supportStructures = Array.AsReadOnly(Array.Empty<SupportStructure>());
    private ReadOnlyCollection<SupportNode> supportNodes = Array.AsReadOnly(Array.Empty<SupportNode>());
    private ReadOnlyCollection<SupportMember> supportMembers = Array.AsReadOnly(Array.Empty<SupportMember>());
    private SupportSnapshot ReadSupports(ulong revision)
    {
        NativeSupportSnapshot summary = new() { Size = (uint)sizeof(NativeSupportSnapshot), Version = 1 };
        for (var attempt = 0; attempt < 3; ++attempt)
        {
            int result; NativeError error;
            fixed (NativeSupportStructure* structures = structureBuffer)
            fixed (NativeSupportNode* nodes = nodeBuffer)
            fixed (NativeSupportMember* members = memberBuffer)
                result = Native.qn_support_snapshot(handle, ref summary, structures, (uint)structureBuffer.Length,
                    nodes, (uint)nodeBuffer.Length, members, (uint)memberBuffer.Length, supportsRevision, out error);
            if (result == 3)
            {
                structureBuffer = new NativeSupportStructure[checked((int)summary.StructureCount)];
                nodeBuffer = new NativeSupportNode[checked((int)summary.NodeCount)];
                memberBuffer = new NativeSupportMember[checked((int)summary.MemberCount)];
                continue;
            }
            Check(result, error);
            if (summary.Revision != revision) throw new InvalidOperationException("Document changed during support snapshot.");
            if (supportsRevision != summary.Revision)
            {
                supportStructures = Array.AsReadOnly(structureBuffer.Take((int)summary.StructureCount)
                    .Select(s => new SupportStructure(s.Id, s.Label(), s.NodeCount, s.MemberCount, s.Generated != 0)).ToArray());
                supportNodes = Array.AsReadOnly(nodeBuffer.Take((int)summary.NodeCount)
                    .Select(n => new SupportNode(n.Structure, n.Id, n.Anchor, n.X, n.Y, n.Z,
                        n.ResolvedX, n.ResolvedY, n.ResolvedZ, n.Station, n.Lateral, n.Vertical)).ToArray());
                static SupportConnection Map(NativeSupportConnection c) => new(c.Treatment,
                    (c.Flags & 1) != 0, (c.Flags & 2) != 0, (c.Flags & 4) != 0, c.AssetLabel());
                supportMembers = Array.AsReadOnly(memberBuffer.Take((int)summary.MemberCount)
                    .Select(m => new SupportMember(m.Structure, m.Id, m.StartNode, m.EndNode, m.Shape, m.Role,
                        m.Orientation, m.HasReference != 0, m.Width, m.Height, m.Wall, Map(m.StartConnection), Map(m.EndConnection))).ToArray());
                supportsRevision = summary.Revision;
            }
            return new(summary.Revision, summary.SelectionRevision, summary.PresentationRevision, summary.Active != 0,
                new((SupportSelectionKind)summary.SelectionKind, summary.SelectionStructure, summary.SelectionElement),
                supportStructures, supportNodes, supportMembers);
        }
        throw new InvalidOperationException("Support snapshot capacity changed repeatedly.");
    }
    public uint EditSupport(ulong revision, SupportRequest request)
    {
        RequireOpen();
        NativeSupportCommand command = new() { ExpectedRevision = revision, Operation = (uint)request.Operation,
            Structure = request.Structure, Element = request.Element, StartNode = request.StartNode, EndNode = request.EndNode,
            Shape = request.Shape, MemberEnd = request.MemberEnd, Treatment = request.Treatment,
            X = request.X, Y = request.Y, Z = request.Z, Width = request.Width, Height = request.Height, Wall = request.Wall };
        Check(Native.qn_support_command(handle, in command, request.Text, out var created, out var error), error);
        return created;
    }
    public void SelectSupport(ulong revision, SupportSelection selection)
    {
        RequireOpen(); Check(Native.qn_select_support(handle, revision, (uint)selection.Kind,
            selection.Structure, selection.Element, out var error), error);
    }
    public void SetSupportWorkspace(bool active)
    {
        RequireOpen(); Check(Native.qn_support_workspace(handle, active ? 1u : 0u, out var error), error);
    }
}
