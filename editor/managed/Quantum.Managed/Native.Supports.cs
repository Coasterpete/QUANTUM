using System.Runtime.InteropServices;

namespace Quantum.Managed;

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeSupportStructure
{
    public uint Id, NodeCount, MemberCount, Generated;
    public fixed byte Name[256];
    public string Label() { fixed (byte* name = Name) return Marshal.PtrToStringUTF8((nint)name) ?? ""; }
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeSupportNode
{
    public uint Structure, Id, Anchor, Reserved;
    public double X, Y, Z, ResolvedX, ResolvedY, ResolvedZ, Station, Lateral, Vertical;
}
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeSupportConnection
{
    public uint Treatment, Flags;
    public fixed byte Asset[512];
    public string AssetLabel() { fixed (byte* asset = Asset) return Marshal.PtrToStringUTF8((nint)asset) ?? ""; }
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeSupportMember
{
    public uint Structure, Id, StartNode, EndNode, Shape, Role, Orientation, HasReference;
    public double Width, Height, Wall;
    public NativeSupportConnection StartConnection, EndConnection;
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeSupportSnapshot
{
    public uint Size, Version;
    public ulong Revision, SelectionRevision, PresentationRevision;
    public uint StructureCount, NodeCount, MemberCount, Active;
    public uint SelectionKind, SelectionStructure, SelectionElement, Reserved;
}
[StructLayout(LayoutKind.Sequential)]
internal struct NativeSupportCommand
{
    public ulong ExpectedRevision;
    public uint Operation, Structure, Element, StartNode, EndNode, Shape, MemberEnd, Treatment;
    public double X, Y, Z, Width, Height, Wall;
}
internal static unsafe partial class Native
{
    [LibraryImport(Library)] internal static partial int qn_support_snapshot(SessionHandle session,
        ref NativeSupportSnapshot summary, NativeSupportStructure* structures, uint structureCapacity,
        NativeSupportNode* nodes, uint nodeCapacity, NativeSupportMember* members, uint memberCapacity,
        ulong knownRevision, out NativeError error);
    [LibraryImport(Library, StringMarshalling = StringMarshalling.Utf8)]
    internal static partial int qn_support_command(SessionHandle session, in NativeSupportCommand command,
        string? text, out uint created, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_select_support(SessionHandle session,
        ulong revision, uint kind, uint structure, uint element, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_support_workspace(SessionHandle session,
        uint active, out NativeError error);
}
