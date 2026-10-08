using System.ComponentModel;
using System.Globalization;
using System.Text.Json;
using System.Text.Json.Nodes;
using Quantum.Managed;

namespace Quantum.Editor;

// A form cell holds text for a native proposal. It has no physical behavior.
public sealed class TrainField(string path, string label, double value) : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;
    public string Path { get; } = path;
    public string Label { get; } = label;
    private string text = value.ToString("G", CultureInfo.CurrentCulture);
    public string Value { get => text; set { text = value; PropertyChanged?.Invoke(this, new(nameof(Value))); } }
}

public sealed partial class EditorViewModel
{
    private static readonly JsonSerializerOptions TrainJson = new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase, PropertyNameCaseInsensitive = true };
    private TrainCarSettings? selectedCar;
    private bool publishingTrain;
    private ulong seededTrainRevision;
    private long trainViewRequest;
    public bool IsTrainWorkspace => WorkspaceMode == 2;
    public IReadOnlyList<string> TrainRoles { get; } = ["Lead", "Middle", "Rear", "Zero", "Trailer", "Custom"];
    public IReadOnlyList<string> TrainVisualAssets { get; } = [
        "assets://train/placeholder-lead-car-shell.glb", "assets://train/placeholder-car-shell.glb", "assets://train/placeholder-rear-car-shell.glb"];
    public IReadOnlyList<TrainCarSettings> TrainCars { get; private set; } = [];
    public IReadOnlyList<TrainField> CarFields { get; private set; } = [];
    public IReadOnlyList<TrainField> CarAlignmentFields { get; private set; } = [];
    public IReadOnlyList<TrainField> TrainDefaultFields { get; private set; } = [];
    public TrainCarSettings? SelectedCar
    {
        get => selectedCar;
        set
        {
            if (publishingTrain) return;
            SeedCar(value); RefreshCommands();
            SynchronizeTrainView();
        }
    }
    public bool HasCar => SelectedCar is not null;
    public string TrainCarName { get; set; } = "";
    public string TrainCarRole { get; set; } = "Middle";
    private string trainPrototypeRole = "Middle", trainCarAsset = "";
    public string TrainPrototypeRole { get => trainPrototypeRole; set { trainPrototypeRole = value ?? "Middle"; Changed(); } }
    public string TrainCarAsset { get => trainCarAsset; set { trainCarAsset = value ?? ""; Changed(); Changed(nameof(SelectedTrainVisualAsset)); } }
    public string? SelectedTrainVisualAsset
    {
        get => TrainVisualAssets.Contains(TrainCarAsset) ? TrainCarAsset : null;
        set { if (value is not null) TrainCarAsset = value; }
    }
    public string AuthoredTrainName { get; set; } = "";
    public string TrainCarCount { get; set; } = "4";
    public string DefaultConnectorLength { get; set; } = ".5";
    public IReadOnlyList<TrainDiagnostic> TrainDiagnostics { get; private set; } = [];
    public IReadOnlyList<TrainCarSummary> TrainCarSummaries => Snapshot?.Train.Summary.Cars ?? [];
    public string TrainSummaryDetails => Snapshot?.Train.Summary is { } s
        ? $"{s.CarCount} cars · {s.BogieCount} bogies · {s.ConnectorCount} connectors\n"
          + $"Dry {s.DryMassKilograms:F1} kg · loaded {s.LoadedMassKilograms:F1} kg\n"
          + (s.StraightAssemblyAvailable ? $"Body-envelope length {s.BodyEnvelopeLengthMeters:F3} m\n" : "Straight reference assembly unavailable\n")
          + $"Front X {s.FrontExtentMeters:F3} m · rear X {s.RearExtentMeters:F3} m\n"
          + $"Straight loaded COG {string.Join(" / ", s.StraightCogMeters.Select(v => v.ToString("F3")))} m"
        : "Loading native train summary…";
    public string TrainPreviewStatus => Snapshot?.Train is { } t
        ? t.PreviewAvailable ? "Accepted train · edits stop and reset simulation · one simulated train" : t.PreviewError : "Loading preview…";
    public string CarEngineeringSummary => SelectedCar is { } c
        ? $"Car #{c.Id} · {c.Physical.Bogies.Length} bogies · +X forward / +Y lateral / +Z up\n"
          + "Explicit dry inertia about dry COG. Body dimensions do not recalculate it."
        : "Select a car";
    private bool showTrainBounds = true, showTrainBogies = true, showTrainHitches = true, showTrainCog = true, showTrainFrame = true;
    public bool ShowTrainBounds { get => showTrainBounds; set { showTrainBounds = value; Changed(); SynchronizeTrainView(); } }
    public bool ShowTrainBogies { get => showTrainBogies; set { showTrainBogies = value; Changed(); SynchronizeTrainView(); } }
    public bool ShowTrainHitches { get => showTrainHitches; set { showTrainHitches = value; Changed(); SynchronizeTrainView(); } }
    public bool ShowTrainCog { get => showTrainCog; set { showTrainCog = value; Changed(); SynchronizeTrainView(); } }
    public bool ShowTrainFrame { get => showTrainFrame; set { showTrainFrame = value; Changed(); SynchronizeTrainView(); } }
    private uint TrainOverlayFlags => (ShowTrainBounds ? 1u : 0) | (ShowTrainBogies ? 2u : 0) | (ShowTrainHitches ? 4u : 0)
        | (ShowTrainCog ? 8u : 0) | (ShowTrainFrame ? 16u : 0);
    public EditorCommand AddCarCommand { get; private set; } = null!;
    public EditorCommand InsertCarCommand { get; private set; } = null!;
    public EditorCommand DuplicateCarCommand { get; private set; } = null!;
    public EditorCommand RemoveCarCommand { get; private set; } = null!;
    public EditorCommand MoveCarUpCommand { get; private set; } = null!;
    public EditorCommand MoveCarDownCommand { get; private set; } = null!;
    public EditorCommand ApplyCarCommand { get; private set; } = null!;
    public EditorCommand ValidateCarCommand { get; private set; } = null!;
    public EditorCommand ApplyTrainDefaultsCommand { get; private set; } = null!;
    public EditorCommand ResizeTrainCommand { get; private set; } = null!;
    public EditorCommand ApplyCarPrototypeCommand { get; private set; } = null!;
    public EditorCommand BoxInertiaCommand { get; private set; } = null!;
    public EditorCommand FrameTrainCommand { get; private set; } = null!;
    public EditorCommand FrameCarCommand { get; private set; } = null!;
    public EditorCommand FrameTrainFrontCommand { get; private set; } = null!;
    public EditorCommand FrameTrainRearCommand { get; private set; } = null!;

    private int CarIndex => Array.FindIndex(Snapshot?.Train.Configuration.Cars ?? [], c => c.Id == SelectedCar?.Id);
    private void InitializeTrainCommands()
    {
        AddCarCommand = Add(() => TrainEdit(TrainEditOperation.Append), () => TrainCars.Count < 32);
        InsertCarCommand = Add(() => TrainEdit(TrainEditOperation.Insert, (uint)Math.Max(0, CarIndex)), () => HasCar && TrainCars.Count < 32);
        DuplicateCarCommand = Add(() => TrainEdit(TrainEditOperation.Duplicate), () => HasCar && TrainCars.Count < 32);
        RemoveCarCommand = Add(() => TrainEdit(TrainEditOperation.Remove), () => HasCar && TrainCars.Count > 1);
        MoveCarUpCommand = Add(() => TrainEdit(TrainEditOperation.Reorder, (uint)(CarIndex - 1)), () => CarIndex > 0);
        MoveCarDownCommand = Add(() => TrainEdit(TrainEditOperation.Reorder, (uint)(CarIndex + 1)), () => HasCar && CarIndex + 1 < TrainCars.Count);
        ApplyCarPrototypeCommand = Add(() => TrainEdit(TrainEditOperation.Prototype), () => HasCar);
        ResizeTrainCommand = Add(() => Run(async () => {
            if (Snapshot is not { } s) return;
            if (!uint.TryParse(TrainCarCount, out var count)) throw new ArgumentException("Cars per train requires a whole number.");
            await Controller.Submit(session => session.EditTrain(s.Revision, TrainEditOperation.Resize, index: count));
        }));
        ApplyCarCommand = Add(() => ApplyTrainProposal(false), () => HasCar);
        ValidateCarCommand = Add(() => ApplyTrainProposal(true), () => HasCar);
        ApplyTrainDefaultsCommand = Add(() => Run(async () => {
            if (Snapshot is not { } s) return;
            var node = JsonSerializer.SerializeToNode(s.Train.Configuration, TrainJson)!;
            node["name"] = AuthoredTrainName;
            node["defaultConnectorLengthMeters"] = ParseTrainNumber(DefaultConnectorLength, "Default connector");
            PutFields(node["defaultPhysical"]!, TrainDefaultFields);
            var proposal = node.Deserialize<TrainConfigurationSettings>(TrainJson)!;
            await Controller.Submit(session => session.ApplyTrain(s.Revision, proposal));
        }));
        // Box inertia is a deliberate native authoring operation, not managed math.
        BoxInertiaCommand = Add(() => ApplyTrainProposal(false, boxInertia: true), () => HasCar);
        FrameTrainCommand = Add(() => TrainCamera(ViewportAction.FrameTrain), () => IsTrainWorkspace);
        FrameCarCommand = Add(() => TrainCamera(ViewportAction.FrameCar), () => IsTrainWorkspace && HasCar);
        FrameTrainFrontCommand = Add(() => TrainCamera(ViewportAction.FrameTrainFront), () => IsTrainWorkspace);
        FrameTrainRearCommand = Add(() => TrainCamera(ViewportAction.FrameTrainRear), () => IsTrainWorkspace);
    }
    private Task TrainCamera(ViewportAction action) => Run(() => { Controller.Session.Camera(action); return Task.CompletedTask; });
    private Task TrainEdit(TrainEditOperation operation, uint index = 0) => Run(async () => {
        if (Snapshot is not { } s) return;
        var id = SelectedCar?.Id ?? 0; var role = TrainPrototypeRole;
        await Controller.Submit(session => session.EditTrain(s.Revision, operation, id, index, role));
    });
    private static double ParseTrainNumber(string value, string label) => double.TryParse(value, NumberStyles.Float,
        CultureInfo.CurrentCulture, out var number) && double.IsFinite(number) ? number : throw new ArgumentException($"{label} must be a finite number.");
    private static void PutFields(JsonNode node, IEnumerable<TrainField> fields)
    {
        foreach (var field in fields)
        {
            var parts = field.Path.Split('.'); var current = node;
            for (int i = 0; i + 1 < parts.Length; ++i)
                current = current is JsonArray a ? a[int.Parse(parts[i])]! : current[parts[i]]!;
            var value = ParseTrainNumber(field.Value, field.Label);
            if (current is JsonArray array) array[int.Parse(parts[^1])] = value;
            else current[parts[^1]] = value;
        }
    }
    private TrainConfigurationSettings CarProposal()
    {
        var configuration = Snapshot!.Train.Configuration;
        var node = JsonSerializer.SerializeToNode(configuration, TrainJson)!;
        var car = node["cars"]![CarIndex]!;
        car["name"] = TrainCarName; car["role"] = TrainCarRole; car["visualAsset"] = TrainCarAsset;
        PutFields(car, CarFields); PutFields(car, CarAlignmentFields);
        return node.Deserialize<TrainConfigurationSettings>(TrainJson)!;
    }
    private Task ApplyTrainProposal(bool validateOnly, bool boxInertia = false) => Run(async () => {
        if (Snapshot is not { } s || !HasCar) return;
        var proposal = CarProposal(); var carId = SelectedCar!.Id;
        if (validateOnly)
        {
            TrainCalculatedSummary? summary = null;
            await Controller.Submit(session => summary = session.ValidateTrain(s.Revision, proposal));
            TrainDiagnostics = summary!.Diagnostics; Changed(nameof(TrainDiagnostics));
            return;
        }
        if (boxInertia)
            await Controller.Submit(session => session.ApplyTrainBoxInertia(s.Revision, proposal, carId));
        else await Controller.Submit(session => session.ApplyTrain(s.Revision, proposal));
    });
    private static IReadOnlyList<TrainField> PhysicalFields(TrainPhysicalSettings physical, string prefix = "")
    {
        var result = new List<TrainField>();
        void Scalar(string path, string label, double value) => result.Add(new(prefix + path, label, value));
        void Vector(string path, string label, double[] values) {
            for (int i = 0; i < 3; ++i) Scalar($"{path}.{i}", $"{label} {"XYZ"[i]} · m", values[i]); }
        Scalar("dryMassKilograms", "Dry mass · kg", physical.DryMassKilograms);
        Scalar("loadMassKilograms", "Load/passenger mass · kg", physical.LoadMassKilograms);
        Vector("dryCogMeters", "Dry COG", physical.DryCogMeters);
        Vector("loadCogMeters", "Load COG", physical.LoadCogMeters);
        Vector("bodyDimensionsMeters", "Body dimension", physical.BodyDimensionsMeters);
        Vector("frontHitchMeters", "Front hitch", physical.FrontHitchMeters);
        Vector("rearHitchMeters", "Rear hitch", physical.RearHitchMeters);
        for (int b = 0; b < physical.Bogies.Length; ++b) Vector($"bogies.{b}.positionMeters", $"Bogie {b} local", physical.Bogies[b].PositionMeters);
        for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
            Scalar($"inertiaKgM2.{row}.{column}", $"Dry inertia I{"XYZ"[row]}{"XYZ"[column]} · kg m²", physical.InertiaKgM2[row][column]);
        Scalar("dragAreaSquareMeters", "Car CdA · m²", physical.DragAreaSquareMeters);
        Vector("aerodynamicCenterMeters", "Aerodynamic center", physical.AerodynamicCenterMeters);
        return result;
    }
    private void SeedCar(TrainCarSettings? car)
    {
        selectedCar = car;
        TrainCarName = car?.Name ?? ""; TrainCarRole = car?.Role ?? "Middle"; TrainCarAsset = car?.VisualAsset ?? "";
        CarFields = car is null ? [] : [.. PhysicalFields(car.Physical, "physical."), new("connectorLengthMeters", "Rear connector · m", car.ConnectorLengthMeters)];
        CarAlignmentFields = car is null ? [] : Enumerable.Range(0, 3).Select(i => new TrainField($"visualTranslationMeters.{i}", $"Shell translation {"XYZ"[i]} · m", car.VisualTranslationMeters[i]))
            .Concat(Enumerable.Range(0, 3).Select(i => new TrainField($"visualRotationDegrees.{i}", $"Shell rotation {"XYZ"[i]} · deg", car.VisualRotationDegrees[i]))).ToArray();
        foreach (var property in new[] { nameof(SelectedCar), nameof(HasCar), nameof(TrainCarName), nameof(TrainCarRole), nameof(TrainCarAsset), nameof(CarFields), nameof(CarAlignmentFields), nameof(CarEngineeringSummary) }) Changed(property);
    }
    private void PublishTrain(TrainWorkspaceSnapshot next)
    {
        publishingTrain = true;
        var changed = seededTrainRevision != next.Revision;
        if (changed)
        {
            var selected = next.Configuration.Cars.FirstOrDefault(c => c.Id == next.SelectedCar) ?? next.Configuration.Cars.FirstOrDefault();
            TrainCars = next.Configuration.Cars; Changed(nameof(TrainCars)); SeedCar(selected);
            AuthoredTrainName = next.Configuration.Name; TrainCarCount = next.Configuration.Cars.Length.ToString();
            DefaultConnectorLength = next.Configuration.DefaultConnectorLengthMeters.ToString("G", CultureInfo.CurrentCulture);
            TrainDefaultFields = PhysicalFields(next.Configuration.DefaultPhysical);
            TrainDiagnostics = next.Summary.Diagnostics;
            foreach (var property in new[] { nameof(AuthoredTrainName), nameof(TrainCarCount), nameof(DefaultConnectorLength), nameof(TrainDefaultFields), nameof(TrainDiagnostics), nameof(TrainCarSummaries), nameof(TrainSummaryDetails) }) Changed(property);
            seededTrainRevision = next.Revision;
        }
        else if (next.SelectedCar != selectedCar?.Id) SeedCar(TrainCars.FirstOrDefault(c => c.Id == next.SelectedCar));
        Changed(nameof(TrainPreviewStatus)); publishingTrain = false;
    }
    private async void SynchronizeTrainView()
    {
        if (publishingTrain || Snapshot is not { } s) return;
        var request = ++trainViewRequest; var id = SelectedCar?.Id ?? 0; var active = IsTrainWorkspace; var flags = TrainOverlayFlags;
        try {
            await Controller.Submit(session => { if (request == trainViewRequest) session.TrainView(s.Revision, active, id, flags); });
        }
        catch (Exception error) { if (request == trainViewRequest) Error = error.Message; }
    }
}
