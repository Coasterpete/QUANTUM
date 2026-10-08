using Quantum.Managed;

namespace Quantum.Editor;

public sealed partial class EditorViewModel
{
    private InfrastructureSettings? selectedInfrastructure;
    private bool updatingInfrastructure;
    private int infrastructureKind, infrastructureSide = 1, infrastructureDeck = 1, infrastructureMaterial = 2;
    private string infrastructureName = "Maintenance catwalk", infrastructureStart = "5", infrastructureEnd = "15";
    private string infrastructureLateral = "1.5", infrastructureVertical = "-.25", infrastructureWidth = ".8";
    private string infrastructureRailHeight = "1.1", infrastructurePostSpacing = "1.5", infrastructureGates = "";
    private string infrastructureWorldStart = "0, 0, 0", infrastructureWorldEnd = "3, 0, 1.5", infrastructureRise = ".18";
    private bool infrastructureInner, infrastructureOuter = true, infrastructureWorld;
    public IReadOnlyList<InfrastructureSettings> InfrastructureItems { get; private set; } = [];
    public InfrastructureSettings? SelectedInfrastructure
    {
        get => selectedInfrastructure;
        set { selectedInfrastructure = value; Changed(); Changed(nameof(HasInfrastructure));
            if (!updatingInfrastructure) { SeedInfrastructure(); _ = SynchronizeInfrastructure(value?.Id ?? 0); } RefreshCommands(); }
    }
    public bool HasInfrastructure => selectedInfrastructure is not null;
    public int InfrastructureKind { get => infrastructureKind; set { infrastructureKind = value; Changed(); } }
    public int InfrastructureSide { get => infrastructureSide; set { infrastructureSide = value; Changed(); } }
    public int InfrastructureDeck { get => infrastructureDeck; set { infrastructureDeck = value; Changed(); } }
    public int InfrastructureMaterial { get => infrastructureMaterial; set { infrastructureMaterial = value; Changed(); } }
    public string InfrastructureName { get => infrastructureName; set { infrastructureName = value; Changed(); } }
    public string InfrastructureStart { get => infrastructureStart; set { infrastructureStart = value; Changed(); } }
    public string InfrastructureEnd { get => infrastructureEnd; set { infrastructureEnd = value; Changed(); } }
    public string InfrastructureLateral { get => infrastructureLateral; set { infrastructureLateral = value; Changed(); } }
    public string InfrastructureVertical { get => infrastructureVertical; set { infrastructureVertical = value; Changed(); } }
    public string InfrastructureWidth { get => infrastructureWidth; set { infrastructureWidth = value; Changed(); } }
    public string InfrastructureRailHeight { get => infrastructureRailHeight; set { infrastructureRailHeight = value; Changed(); } }
    public string InfrastructurePostSpacing { get => infrastructurePostSpacing; set { infrastructurePostSpacing = value; Changed(); } }
    public string InfrastructureGates { get => infrastructureGates; set { infrastructureGates = value; Changed(); } }
    public string InfrastructureWorldStart { get => infrastructureWorldStart; set { infrastructureWorldStart = value; Changed(); } }
    public string InfrastructureWorldEnd { get => infrastructureWorldEnd; set { infrastructureWorldEnd = value; Changed(); } }
    public string InfrastructureRise { get => infrastructureRise; set { infrastructureRise = value; Changed(); } }
    public bool InfrastructureInner { get => infrastructureInner; set { infrastructureInner = value; Changed(); } }
    public bool InfrastructureOuter { get => infrastructureOuter; set { infrastructureOuter = value; Changed(); } }
    public bool InfrastructureWorld { get => infrastructureWorld; set { infrastructureWorld = value; Changed(); } }
    public EditorCommand CreateInfrastructureCommand { get; private set; } = null!;
    public EditorCommand ApplyInfrastructureCommand { get; private set; } = null!;
    public EditorCommand DeleteInfrastructureCommand { get; private set; } = null!;
    public EditorCommand SelectInfrastructureGateCommand { get; private set; } = null!;
    private void InitializeInfrastructureCommands()
    {
        CreateInfrastructureCommand = Add(() => Run(() => CommitInfrastructure(true)), () => Snapshot?.TotalLength > 0);
        ApplyInfrastructureCommand = Add(() => Run(() => CommitInfrastructure(false)), () => HasInfrastructure);
        DeleteInfrastructureCommand = Add(() => Run(() =>
        {
            if (Snapshot is not { } snapshot || selectedInfrastructure is not { } item) return Task.CompletedTask;
            return Controller.Submit(s => s.SetInfrastructure(snapshot.Revision,
                snapshot.Hardware.Infrastructure.Where(i => i.Id != item.Id).ToArray(), snapshot.Hardware.NextInfrastructureId));
        }), () => HasInfrastructure);
        SelectInfrastructureGateCommand = Add(() => Run(() =>
        {
            var id = InfrastructureGates.Split([',',' '],StringSplitOptions.RemoveEmptyEntries).FirstOrDefault();
            if (id is not null && Snapshot is { } snapshot)
            {
                var device = snapshot.Devices.FirstOrDefault(d => d.Id == Id(id,"Gate ID"));
                if (device is null || device.Kind != (uint)DeviceKind.Access) throw new ArgumentException("Choose an existing Access gate ID.");
                WorkspaceMode = 0; TimelineSource = 1; SelectedDevice = device;
            }
            return Task.CompletedTask;
        }), () => InfrastructureGates.Trim().Length > 0);
    }
    private static double[] Endpoint(string text)
    {
        var values = text.Split(',',StringSplitOptions.TrimEntries);
        if (values.Length != 3) throw new ArgumentException("World endpoint needs X, Y, Z separated by commas.");
        return values.Select(v => Finite(v,"World endpoint")).ToArray();
    }
    private Task CommitInfrastructure(bool create)
    {
        if (Snapshot is not { } snapshot) return Task.CompletedTask;
        var id = create ? snapshot.Hardware.NextInfrastructureId : selectedInfrastructure!.Id;
        var material = InfrastructureMaterial switch { 0 => "Timber", 1 => "PaintedSteel", 2 => "GalvanizedSteel", _ => "Concrete" };
        var surface = selectedInfrastructure?.Surface?.Material == material ? selectedInfrastructure.Surface :
            new SupportSurfaceSettings(material, InfrastructureMaterial switch { 0 => [.78f,.64f,.47f], 1 => [.23f,.38f,.48f],
                2 => [.68f,.71f,.73f], _ => [.62f,.62f,.6f] }, InfrastructureMaterial == 2 ? .48f : InfrastructureMaterial == 1 ? .42f : .85f,
                InfrastructureMaterial == 2 ? 1 : 0, .2f, [1,1]);
        var item = new InfrastructureSettings(id, InfrastructureName, (uint)InfrastructureKind,
            Finite(InfrastructureStart,"Start station"),Finite(InfrastructureEnd,"End station"),InfrastructureSide == 0 ? -1 : 1,
            Finite(InfrastructureLateral,"Lateral offset"),Finite(InfrastructureVertical,"Vertical offset"),Finite(InfrastructureWidth,"Width"),
            (uint)InfrastructureDeck,new(InfrastructureInner,InfrastructureOuter,Finite(InfrastructureRailHeight,"Rail height"),
                Finite(InfrastructurePostSpacing,"Post spacing")),
            InfrastructureGates.Split([',',' '],StringSplitOptions.RemoveEmptyEntries).Select(v => Id(v,"Gate ID")).ToArray(),
            selectedInfrastructure?.MeshAsset ?? "", surface,InfrastructureWorld,Endpoint(InfrastructureWorldStart),Endpoint(InfrastructureWorldEnd),
            Finite(InfrastructureRise,"Step rise"));
        var items = create ? snapshot.Hardware.Infrastructure.Append(item).ToArray()
            : snapshot.Hardware.Infrastructure.Select(i => i.Id == id ? item : i).ToArray();
        return Controller.Submit(s =>
        {
            s.SetInfrastructure(snapshot.Revision,items,create ? checked(id+1) : snapshot.Hardware.NextInfrastructureId);
            s.SelectInfrastructure(s.Snapshot().Revision,id);
        });
    }
    private ulong lastNativeAccess;
    private async Task SynchronizeInfrastructure(ulong id)
    {
        if (Snapshot is not { } snapshot) return;
        try { await Controller.Submit(s => s.SelectInfrastructure(snapshot.Revision,id)); }
        catch (ObjectDisposedException) { }
        catch (TaskCanceledException) { }
        catch (QuantumException error) { if (error.Code != 8) Error = error.Message; }
    }
    private void PublishInfrastructure(SessionSnapshot next, bool changed)
    {
        var id = next.Hardware.SelectedInfrastructure;
        if (changed || selectedInfrastructure?.Id != (id == 0 ? null : id))
        {
            updatingInfrastructure = true;
            InfrastructureItems = next.Hardware.Infrastructure; Changed(nameof(InfrastructureItems));
            selectedInfrastructure = InfrastructureItems.FirstOrDefault(i => i.Id == id);
            Changed(nameof(SelectedInfrastructure)); Changed(nameof(HasInfrastructure));
            updatingInfrastructure = false;
            if (selectedInfrastructure is not null) SeedInfrastructure();
        }
        if (IsSupportsWorkspace && next.Hardware.SelectedAccess != lastNativeAccess
            && next.Devices.FirstOrDefault(d => d.Id == next.Hardware.SelectedAccess && d.Kind == (uint)DeviceKind.Access) is { } gate)
        {
            WorkspaceMode = 0; TimelineSource = 1; SelectedDevice = gate;
        }
        lastNativeAccess = next.Hardware.SelectedAccess;
    }
    private void SeedInfrastructure()
    {
        if (selectedInfrastructure is not { } i) return;
        InfrastructureKind = (int)i.Kind; InfrastructureName = i.Name;
        InfrastructureStart = Number(i.StartStationMeters); InfrastructureEnd = Number(i.EndStationMeters);
        InfrastructureSide = i.Side == -1 ? 0 : 1; InfrastructureDeck = (int)i.Deck;
        InfrastructureLateral = Number(i.LateralOffsetMeters); InfrastructureVertical = Number(i.VerticalOffsetMeters);
        InfrastructureWidth = Number(i.WidthMeters); InfrastructureGates = string.Join(", ",i.AccessGates ?? []);
        var rails = i.Railings ?? new(); InfrastructureInner = rails.Inner; InfrastructureOuter = rails.Outer;
        InfrastructureRailHeight = Number(rails.HeightMeters); InfrastructurePostSpacing = Number(rails.PostSpacingMeters);
        InfrastructureWorld = i.WorldPlacement; InfrastructureRise = Number(i.StepRiseMeters);
        InfrastructureWorldStart = string.Join(", ",(i.WorldStartMeters ?? [0,0,0]).Select(Number));
        InfrastructureWorldEnd = string.Join(", ",(i.WorldEndMeters ?? [3,0,1.5]).Select(Number));
        InfrastructureMaterial = i.Surface?.Material switch { "Timber" => 0, "PaintedSteel" => 1, "Concrete" => 3, _ => 2 };
    }
}
