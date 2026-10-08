using System.Text.Json;
using System.Runtime.InteropServices;

namespace Quantum.Managed;

public sealed record SupportSurfaceSettings(string Material, float[] Tint, float Roughness,
    float Metallic, float NormalStrength, float[] RepeatMeters);
public sealed record SupportProfileSettings(uint Shape, double Width, double Height, double Wall);
public sealed record SupportFrameSettings(double StationMeters, double WidthMeters,
    double FoundationElevationMeters, double VerticalOffsetMeters, SupportProfileSettings Post,
    SupportProfileSettings Beam, SupportProfileSettings Brace, double[] FoundationSizeMeters);
public sealed record SupportMemberAppearance(uint Id, bool UseStyleProfile,
    SupportSurfaceSettings? Surface, SupportSurfaceSettings Resolved, bool UseFrameProfile = false);
public sealed record SupportFoundationSettings(uint Id, uint Kind, double[] Dimensions);
public sealed record SupportStyleSettings(uint Id, uint Family, SupportSurfaceSettings? Surface,
    SupportSurfaceSettings Resolved, SupportFrameSettings? Frame, SupportMemberAppearance[] Members,
    SupportFoundationSettings[] Foundations);

internal static unsafe partial class Native
{
    [LibraryImport(Library)] internal static partial int qn_select_infrastructure(SessionHandle session,
        ulong revision, ulong id, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_support_style_snapshot(SessionHandle session,
        ulong revision, byte* buffer, uint capacity, out uint required, out NativeError error);
}
public sealed unsafe partial class Session
{
    public void SelectInfrastructure(ulong revision, ulong id)
    {
        RequireOpen(); Check(Native.qn_select_infrastructure(handle,revision,id,out var error),error);
    }
    private byte[] supportStyleBuffer = new byte[4096];
    private SupportStyleSettings[] ReadSupportStyles(ulong revision)
    {
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            int result; uint required; NativeError error;
            fixed (byte* buffer = supportStyleBuffer)
                result = Native.qn_support_style_snapshot(handle, revision, buffer,
                    (uint)supportStyleBuffer.Length, out required, out error);
            if (result == 3) { supportStyleBuffer = new byte[checked((int)required)]; continue; }
            Check(result, error);
            return JsonSerializer.Deserialize<SupportStyleSettings[]>(
                supportStyleBuffer.AsSpan(0, checked((int)required - 1)), HardwareJson) ?? [];
        }
        throw new InvalidOperationException("Support style snapshot changed during sizing.");
    }
}
