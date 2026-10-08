using System.Runtime.InteropServices;
namespace Quantum.Managed;
internal static unsafe partial class Native
{
    [LibraryImport(Library)] internal static partial int qn_hardware_snapshot(SessionHandle session,
        ulong revision, byte* buffer, uint capacity, out uint required, out NativeError error);
    [LibraryImport(Library, StringMarshalling = StringMarshalling.Utf8)]
    internal static partial int qn_hardware_configuration(SessionHandle session, ulong revision,
        ulong device, string configuration, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_hardware_command(SessionHandle session,
        ulong revision, ulong device, uint command, double value, ulong target, out NativeError error);
    [LibraryImport(Library, StringMarshalling = StringMarshalling.Utf8)]
    internal static partial int qn_infrastructure_configuration(SessionHandle session, ulong revision,
        string infrastructure, ulong nextId, out NativeError error);
}
