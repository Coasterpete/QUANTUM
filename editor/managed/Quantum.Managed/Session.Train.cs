using System.Text.Json;
using System.Runtime.InteropServices;
using System.Text.Json.Serialization;

namespace Quantum.Managed;

public enum TrainEditOperation : uint { Replace, Append, Insert, Duplicate, Remove, Reorder, Prototype, Resize, ApproximateBoxInertia }
public sealed record TrainBogieSettings(double[] PositionMeters, JsonElement[] Contacts);
public sealed record TrainPhysicalSettings(double DryMassKilograms, double[] DryCogMeters,
    double[] BodyDimensionsMeters, double[][] InertiaKgM2, double[] FrontHitchMeters, double[] RearHitchMeters,
    TrainBogieSettings[] Bogies, double LoadMassKilograms, double[] LoadCogMeters,
    double DragAreaSquareMeters, double[] AerodynamicCenterMeters);
public sealed record TrainCarSettings(ulong Id, string Name, string Role, TrainPhysicalSettings Physical,
    double ConnectorLengthMeters, string VisualAsset, double[] VisualTranslationMeters, double[] VisualRotationDegrees)
{
    [JsonIgnore] public string Label => $"{Name} · {Role} · #{Id}";
    [JsonIgnore] public string MassLabel => $"Dry {Physical.DryMassKilograms:F0} kg · load {Physical.LoadMassKilograms:F0} kg";
    public override string ToString() => Label;
}
public sealed record TrainResistanceSettings(double MechanicalForceNewtons, double LinearCoefficient,
    double AirDensity, double DragAreaSquareMeters, double RollingCoefficient);
public sealed record TrainConfigurationSettings(uint SchemaVersion, ulong TrainId, string Name, ulong NextCarId,
    TrainCarSettings[] Cars, TrainPhysicalSettings DefaultPhysical, double DefaultConnectorLengthMeters, TrainResistanceSettings Resistance);
public sealed record TrainDiagnostic(string Severity, ulong CarId, string Message)
{
    [JsonIgnore] public string Label => $"{Severity} · car #{CarId} · {Message}";
}
public sealed record TrainCarSummary(ulong Id, double DryMassKilograms, double LoadedMassKilograms,
    double[] LoadedCogMeters, double ReferenceXMeters, double GapToNextMeters)
{
    [JsonIgnore] public string Label => $"#{Id} · {LoadedMassKilograms:F1} kg · reference X {ReferenceXMeters:F3} m · gap to next {GapToNextMeters:F3} m";
}
public sealed record TrainCalculatedSummary(uint CarCount, uint BogieCount, uint ConnectorCount,
    double DryMassKilograms, double LoadedMassKilograms, bool StraightAssemblyAvailable, double BodyEnvelopeLengthMeters,
    double FrontExtentMeters, double RearExtentMeters, double[] StraightCogMeters,
    TrainCarSummary[] Cars, TrainDiagnostic[] Diagnostics);
public sealed record TrainWorkspaceSnapshot(ulong Revision, TrainConfigurationSettings Configuration,
    TrainCalculatedSummary Summary, ulong SelectedCar, bool Active, uint Overlays, bool PreviewAvailable, string PreviewError);

internal static unsafe partial class Native
{
    [LibraryImport(Library)] internal static partial int qn_train_snapshot(SessionHandle session, ulong revision,
        byte* buffer, uint capacity, out uint required, out NativeError error);
    [LibraryImport(Library, StringMarshalling = StringMarshalling.Utf8)]
    internal static partial int qn_train_edit(SessionHandle session, ulong revision, uint operation,
        ulong car, uint index, string? proposal, out NativeError error);
    [LibraryImport(Library)] internal static partial int qn_train_view(SessionHandle session, ulong revision,
        uint active, ulong car, uint overlays, out NativeError error);
    [LibraryImport(Library, StringMarshalling = StringMarshalling.Utf8)]
    internal static partial int qn_train_validate(SessionHandle session, ulong revision, string proposal,
        byte* buffer, uint capacity, out uint required, out NativeError error);
}

public sealed unsafe partial class Session
{
    private byte[] trainBuffer = new byte[16384];
    private TrainWorkspaceSnapshot? trainSnapshot;
    private TrainWorkspaceSnapshot ReadTrain(ulong revision)
    {
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            int result; uint required; NativeError error;
            fixed (byte* buffer = trainBuffer)
                result = Native.qn_train_snapshot(handle, revision, buffer, (uint)trainBuffer.Length, out required, out error);
            if (result == 3) { trainBuffer = new byte[checked((int)required)]; continue; }
            Check(result, error);
            using var document = JsonDocument.Parse(trainBuffer.AsMemory(0, checked((int)required - 1)));
            var root = document.RootElement;
            var configuration = trainSnapshot?.Revision == revision ? trainSnapshot.Configuration
                : root.GetProperty("configuration").Deserialize<TrainConfigurationSettings>(HardwareJson)!;
            var summary = trainSnapshot?.Revision == revision ? trainSnapshot.Summary
                : root.GetProperty("summary").Deserialize<TrainCalculatedSummary>(HardwareJson)!;
            return trainSnapshot = new(revision, configuration, summary, root.GetProperty("selectedCar").GetUInt64(),
                root.GetProperty("active").GetBoolean(), root.GetProperty("overlays").GetUInt32(),
                root.GetProperty("previewAvailable").GetBoolean(), root.GetProperty("previewError").GetString()!);
        }
        throw new InvalidOperationException("Train snapshot changed during sizing.");
    }
    public void EditTrain(ulong revision, TrainEditOperation operation, ulong car = 0, uint index = 0, string? proposal = null)
    {
        RequireOpen();
        Check(Native.qn_train_edit(handle, revision, (uint)operation, car, index, proposal, out var error), error);
    }
    public void ApplyTrain(ulong revision, TrainConfigurationSettings configuration) =>
        EditTrain(revision, TrainEditOperation.Replace, proposal: JsonSerializer.Serialize(configuration, HardwareJson));
    public void ApplyTrainBoxInertia(ulong revision, TrainConfigurationSettings configuration, ulong car) =>
        EditTrain(revision, TrainEditOperation.ApproximateBoxInertia, car, proposal: JsonSerializer.Serialize(configuration, HardwareJson));
    public void TrainView(ulong revision, bool active, ulong car, uint overlays = 31)
    {
        RequireOpen(); Check(Native.qn_train_view(handle, revision, active ? 1u : 0u, car, overlays, out var error), error);
    }
    public TrainCalculatedSummary ValidateTrain(ulong revision, TrainConfigurationSettings proposal)
    {
        RequireOpen(); var json = JsonSerializer.Serialize(proposal, HardwareJson);
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            int result; uint required; NativeError error;
            fixed (byte* buffer = trainBuffer)
                result = Native.qn_train_validate(handle, revision, json, buffer, (uint)trainBuffer.Length, out required, out error);
            if (result == 3) { trainBuffer = new byte[checked((int)required)]; continue; }
            Check(result, error);
            return JsonSerializer.Deserialize<TrainCalculatedSummary>(trainBuffer.AsSpan(0, checked((int)required - 1)), HardwareJson)!;
        }
        throw new InvalidOperationException("Train proposal changed during sizing.");
    }
}
