using System.ComponentModel;
using System.Text.Json;

namespace Quantum.Editor;

public sealed class WorkspaceLayout
{
    public bool DockOpen { get; set; }
    public string Tab { get; set; } = "Timeline";
    public double DockHeight { get; set; } = 280;
    public double BrowserWidth { get; set; } = 240;
    public double InspectorWidth { get; set; } = 310;
}

// User presentation state only. No document, selection or simulation values.
public sealed class WorkbenchLayout : INotifyPropertyChanged
{
    private sealed class Settings
    {
        public Settings() { }
        public int Version { get; set; } = 1;
        public int Workspace { get; set; }
        public Dictionary<int, WorkspaceLayout> Panes { get; set; } = [];
        public Dictionary<string, bool> Sections { get; set; } = [];
    }
    private Settings settings = new();
    public event PropertyChangedEventHandler? PropertyChanged;
    public event Action? SectionsReset;
    public int Workspace => settings.Workspace;
    private WorkspaceLayout Current => settings.Panes[Workspace];
    public IReadOnlyList<string> Tabs => Workspace switch
    {
        1 => ["Infrastructure", "Diagnostics", "Timeline"],
        2 => ["Summary", "Diagnostics", "Timeline"],
        3 => ["Profile", "Runtime", "Diagnostics", "Timeline"],
        _ => ["Timeline"]
    };
    public bool DockOpen { get => Current.DockOpen; set { Current.DockOpen = value; Notify(); } }
    public string SelectedTab
    {
        get => Current.Tab;
        set { if (value is null || !Tabs.Contains(value) || value == Current.Tab) return; Current.Tab = value; DockOpen = true; Notify(); }
    }
    public double DockHeight { get => Current.DockHeight; set { Current.DockHeight = Clamp(value, 160, 420, 280); } }
    public double BrowserWidth { get => Current.BrowserWidth; set { Current.BrowserWidth = Clamp(value, 190, 420, 240); } }
    public double InspectorWidth { get => Current.InspectorWidth; set { Current.InspectorWidth = Clamp(value, 250, 500, 310); } }
    public string ToggleLabel => DockOpen ? "Collapse ▾" : "Open ▴";
    public bool ShowTimeline => DockOpen && SelectedTab is "Timeline" or "Profile";
    public bool ShowSummary => DockOpen && Workspace == 2 && SelectedTab == "Summary";
    public bool ShowTrainDiagnostics => DockOpen && Workspace == 2 && SelectedTab == "Diagnostics";
    public bool ShowInfrastructure => DockOpen && Workspace == 1 && SelectedTab == "Infrastructure";
    public bool ShowRuntime => DockOpen && Workspace == 3 && SelectedTab == "Runtime";
    public bool ShowDiagnostics => DockOpen && Workspace != 2 && SelectedTab == "Diagnostics";

    public WorkbenchLayout() => SelectWorkspace(0);
    private static WorkspaceLayout Defaults(int workspace) => new()
    {
        DockOpen = workspace is 0 or 3,
        Tab = workspace switch { 1 => "Infrastructure", 2 => "Summary", 3 => "Profile", _ => "Timeline" }
    };
    public void SelectWorkspace(int workspace)
    {
        settings.Workspace = Math.Clamp(workspace, 0, 3);
        if (!settings.Panes.ContainsKey(Workspace)) settings.Panes[Workspace] = Defaults(Workspace);
        if (!Tabs.Contains(Current.Tab)) Current.Tab = Defaults(Workspace).Tab;
        Notify();
    }
    public bool Expanded(string key, bool defaultValue = false) => settings.Sections.GetValueOrDefault(key, defaultValue);
    public void OpenTab(string tab) { SelectedTab = tab; DockOpen = true; }
    public void SetExpanded(string key, bool value) { if (key.Length > 0) settings.Sections[key] = value; }
    public void ResetWorkspace()
    {
        settings.Panes[Workspace] = Defaults(Workspace);
        var prefix = Workspace switch { 1 => "supports.", 2 => "train.", 3 => "operations.", _ => "track." };
        foreach (var key in settings.Sections.Keys.Where(k => k.StartsWith(prefix, StringComparison.Ordinal)).ToArray())
            settings.Sections.Remove(key);
        SectionsReset?.Invoke(); Notify();
    }
    private static double Clamp(double value, double min, double max, double fallback) =>
        double.IsFinite(value) ? Math.Clamp(value, min, max) : fallback;
    private void Notify() => PropertyChanged?.Invoke(this, new(null));
    public static string SettingsPath => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "QUANTUM", "workbench-layout.json");
    public static WorkbenchLayout Load(string path, out string? error)
    {
        var layout = new WorkbenchLayout(); error = null;
        try
        {
            if (!File.Exists(path)) return layout;
            var saved = JsonSerializer.Deserialize<Settings>(File.ReadAllText(path));
            if (saved is null || saved.Version != 1 || saved.Panes is null || saved.Sections is null)
                throw new JsonException("Unsupported workbench layout.");
            saved.Panes = saved.Panes.Where(p => p.Key is >= 0 and <= 3 && p.Value is not null).ToDictionary();
            layout.settings = saved;
            foreach (var (key, pane) in saved.Panes)
            {
                pane.DockHeight = Clamp(pane.DockHeight, 160, 420, 280);
                pane.BrowserWidth = Clamp(pane.BrowserWidth, 190, 420, 240);
                pane.InspectorWidth = Clamp(pane.InspectorWidth, 250, 500, 310);
                pane.Tab ??= Defaults(key).Tab;
            }
            layout.SelectWorkspace(saved.Workspace);
        }
        catch (Exception failure) when (failure is IOException or UnauthorizedAccessException or JsonException)
        { error = $"Layout settings could not be read; using defaults. {failure.Message}"; layout = new(); }
        return layout;
    }
    public void Save(string path)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
        var temporary = path + ".tmp";
        File.WriteAllText(temporary, JsonSerializer.Serialize(settings, new JsonSerializerOptions { WriteIndented = true }));
        File.Move(temporary, path, true);
    }
}
