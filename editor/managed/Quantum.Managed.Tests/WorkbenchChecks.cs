using Quantum.Editor;
using Quantum.Managed;
using System.Xml.Linq;
using Avalonia.Controls;
using Avalonia.Controls.Primitives;
using Avalonia.Data;

internal static class WorkbenchChecks
{
    private static int checks;
    private static void Require(bool value, string name) { ++checks; if (!value) throw new Exception(name); }
    public static async Task Run(string fixture)
    {
        LayoutPersistence();
        using var controller = new SessionController();
        await controller.Submit(s => s.Open(fixture));
        var vm = new EditorViewModel(controller); vm.Publish(controller.Latest!);
        async Task Drain() { await controller.Submit(_ => { }); vm.Publish(controller.Latest!); }
        async Task Execute(EditorCommand command) { await command.ExecuteAsync(); Require(vm.Error.Length == 0, vm.Error); }

        Require(vm.IsRouteWorkspace && vm.Layout.ShowTimeline, "Track starts with Timeline");
        var region = vm.Selected!.Index;
        await Execute(vm.SelectTrackStartCommand);
        Require(vm.IsTrackStartSelected && !vm.ShowRegionInspector, "Track Start owns Inspector context");
        vm.Selected = vm.Snapshot!.Route.First(); await Drain();
        Require(!vm.IsTrackStartSelected && vm.ShowRegionInspector, "Route selection restores Region Inspector");

        vm.WorkspaceMode = 2; await Drain();
        Require(!vm.Layout.DockOpen && vm.Layout.SelectedTab == "Summary", "Train starts with a secondary Summary dock");
        var carId = vm.SelectedCar!.Id;
        var grouped = vm.CarMassFields.Concat(vm.CarBodyFields).Concat(vm.CarBogieFields).Concat(vm.CarHitchFields).Concat(vm.CarAdvancedFields).ToArray();
        Require(grouped.Length == vm.CarFields.Count && grouped.Distinct().Count() == vm.CarFields.Count
            && grouped.All(vm.CarFields.Contains), "Every native physical form cell remains reachable in exactly one group");
        Require(!vm.CarPending && !vm.TrainDefaultsPending, "Accepted Train forms have no pending edits");
        var bogie = vm.CarBogieFields.First(); var acceptedBogie = bogie.Value;
        bogie.Value = "1.23"; Require(vm.CarPending, "Advanced field changes expose pending car state");
        var section = new InspectorSection { SectionKey = "train.bogies", DataContext = vm };
        Require(!section.IsExpanded, "Advanced section starts collapsed");
        section.IsExpanded = true; Require(vm.Layout.Expanded("train.bogies"), "Expanding remembers the user's choice");
        section.IsExpanded = false;
        Require(vm.CarPending && bogie.Value == "1.23", "Collapse preserves staged text");
        section.IsExpanded = true; vm.Layout.ResetWorkspace();
        Require(!section.IsExpanded, "Reset updates an existing section control");
        // Console test continuations can change threads. A live control's
        // settings subscription belongs to its Avalonia UI thread.
        section.DataContext = null;
        vm.Layout.OpenTab("Summary"); vm.Layout.DockOpen = false;
        vm.Layout.SelectedTab = "Summary";
        Require(!vm.Layout.DockOpen, "Binding reselection cannot open a saved collapsed dock");
        vm.WorkspaceMode = 0; await Drain(); vm.WorkspaceMode = 2; await Drain();
        Require(vm.SelectedCar!.Id == carId && vm.CarPending && vm.CarBogieFields.First().Value == "1.23", "Workspace switching preserves car selection and draft");
        await Execute(vm.RevertCarCommand);
        Require(!vm.CarPending && vm.CarBogieFields.First().Value == acceptedBogie, "Revert restores accepted car without history");
        var revision = vm.Snapshot!.Revision;
        vm.TrainCarName = "UX staged car"; Require(vm.CarPending && vm.Snapshot.Revision == revision, "Typing does not mutate document");
        await Execute(vm.ApplyCarCommand);
        Require(!vm.CarPending && vm.SelectedCar!.Name == "UX staged car" && vm.Snapshot!.CanUndo, "Apply commits one native car edit and clears pending");
        await Execute(vm.UndoCommand);
        vm.AuthoredTrainName = "UX defaults"; Require(vm.TrainDefaultsPending && !vm.CarPending, "Defaults use a separate pending boundary");
        await Execute(vm.RevertTrainDefaultsCommand); Require(!vm.TrainDefaultsPending, "Revert defaults restores accepted form");
        vm.TrainCarName = ""; vm.CarFields.First(f => f.Path == "physical.dryMassKilograms").Value = "-1";
        revision = vm.Snapshot!.Revision;
        await vm.ApplyCarCommand.ExecuteAsync();
        Require(vm.HasError && vm.Snapshot!.Revision == revision && vm.CarPending, "Rejected car proposal preserves native revision and visible pending state");
        await vm.RevertCarCommand.ExecuteAsync(); vm.Error = "";

        vm.WorkspaceMode = 3; await Drain();
        Require(vm.IsOperationsWorkspace && vm.IsDeviceTimeline && vm.Layout.ShowTimeline && vm.Layout.SelectedTab == "Profile", "Operations opens native hardware profiles");
        vm.SelectedHardwareChoice = vm.HardwareChoices.First(h => h.Kind == DeviceKind.Launch);
        await Execute(vm.CreateHardwareCommand);
        var deviceId = vm.SelectedDevice!.Id;
        Require(!vm.HardwareConfigurationPending && !vm.HardwarePlacementPending && !vm.HardwareAccelerationPending, "Accepted hardware groups start clean");
        vm.HardwareFilter = "no matching hardware"; vm.SelectedBrowserHardware = null;
        Require(vm.FilteredHardware.Count == 0 && vm.SelectedDevice.Id == deviceId, "Filtering out the selected item retains its identity");
        vm.HardwareFilter = "Launch"; Require(vm.FilteredHardware.Any(d => d.Id == deviceId), "Hardware filter includes family names");
        vm.HardwareFilter = "";
        vm.HardwareTarget = "12"; Require(vm.HardwareConfigurationPending && !vm.HardwarePlacementPending, "Pending target belongs to configuration only");
        vm.HardwareDirection = 1;
        vm.DeviceEnd = "20"; Require(vm.HardwarePlacementPending, "Placement has a separate pending indicator");
        Require(vm.HardwarePendingStatus.Contains("Placement") && vm.HardwarePendingStatus.Contains("Configuration"), "Collapsed groups still expose pending state at Inspector heading");
        await Execute(vm.ApplyDevicePropertiesCommand);
        Require(!vm.HardwarePlacementPending && vm.HardwareConfigurationPending && vm.HardwareTarget == "12" && vm.HardwareDirection == 1, "Placement Apply retains all pending configuration fields");
        vm.DeviceAcceleration = (vm.SelectedDevice!.Acceleration + 1).ToString(System.Globalization.CultureInfo.CurrentCulture);
        Require(vm.HardwareAccelerationPending, "Acceleration has a separate pending indicator");
        await Execute(vm.ApplyDeviceCommand);
        Require(!vm.HardwareAccelerationPending && vm.HardwareConfigurationPending, "Profile Apply retains pending configuration");
        vm.DeviceEnd = "unfinished placement"; vm.DeviceAcceleration = "unfinished acceleration";
        await Execute(vm.ApplyHardwareCommand);
        Require(!vm.HardwareConfigurationPending && vm.SelectedHardwareRuntime!.Settings(new LaunchSettings()).TargetSpeedMetersPerSecond == 12, "Configuration Apply commits to native hardware");
        Require(vm.HardwarePlacementPending && vm.HardwareAccelerationPending && vm.DeviceEnd == "unfinished placement"
            && vm.DeviceAcceleration == "unfinished acceleration", "Configuration Apply preserves unfinished inputs in other groups");
        vm.HardwareTarget = "14"; vm.DeviceEnd = "21";
        revision = vm.Snapshot!.Revision;
        await Execute(vm.RevertHardwareCommand);
        Require(!vm.HardwareConfigurationPending && !vm.HardwarePlacementPending && vm.HardwareTarget == "12" && vm.Snapshot.Revision == revision, "Revert hardware discards drafts without a native transaction");
        vm.Layout.OpenTab("Runtime"); Require(vm.Layout.ShowRuntime && !vm.Layout.ShowTimeline, "Runtime uses a different dock page from authored profiles");
        vm.Layout.DockOpen = false;
        await Execute(vm.OpenTimelineCommand); Require(vm.Layout.ShowTimeline, "Timeline shortcut always reopens the dock");
        vm.WorkspaceMode = 0; await Drain(); vm.WorkspaceMode = 3; await Drain();
        Require(vm.SelectedDevice!.Id == deviceId && vm.Selected!.Index == region, "Workspace changes preserve Hardware and Region identities");

        var supportFixture = Path.Combine(Path.GetDirectoryName(fixture)!, "supports-2-structural-styles.quantum");
        await controller.Submit(s => s.Open(supportFixture)); vm.Publish(controller.Latest!);
        vm.WorkspaceMode = 1; await Drain();
        Require(!vm.Layout.DockOpen, "Supports starts with a collapsed dock");
        var structure = vm.SupportStructures.First();
        await vm.SelectSupportAsync(new(SupportSelectionKind.Structure, structure.Id));
        Require(vm.ShowSupportStructure && !vm.ShowSupportMember && !vm.ShowSupportNode, "Structure shows its own Inspector");
        var member = vm.SupportMembers.First();
        await vm.SelectSupportAsync(new(SupportSelectionKind.Member, structure.Id, member.Id));
        Require(vm.ShowSupportMember && !vm.ShowSupportStructure, "Member suppresses unrelated structure fields");
        vm.SelectedInfrastructure = vm.InfrastructureItems.First(); await Drain();
        Require(vm.InspectInfrastructure && !vm.ShowSupportMember && !vm.ShowSupportStructure && vm.SupportHeading == vm.SelectedInfrastructure!.Name, "Infrastructure changes Inspector context without losing support selection");
        var infrastructureId = vm.SelectedInfrastructure!.Id;
        vm.InfrastructureName = "UX infrastructure"; await Execute(vm.ApplyInfrastructureCommand);
        Require(vm.InspectInfrastructure && vm.SelectedInfrastructure!.Id == infrastructureId, "Applying infrastructure retains its Inspector context");
        await vm.SelectSupportAsync(new(SupportSelectionKind.Member, structure.Id, member.Id));
        Require(vm.ShowSupportMember && !vm.InspectInfrastructure, "Selecting the same member returns from infrastructure");
        var node = vm.SupportNodes.First();
        await vm.SelectSupportAsync(new(SupportSelectionKind.Node, structure.Id, node.Id));
        Require(vm.ShowSupportNode && !vm.ShowSupportMember, "Node shows only its position/attachment Inspector");
        revision = vm.Snapshot!.Revision;
        vm.Layout.SetExpanded("supports.style", true); vm.Layout.OpenTab("Timeline"); vm.Layout.DockOpen = false;
        await Execute(vm.ResetLayoutCommand);
        Require(!vm.Layout.DockOpen && !vm.Layout.Expanded("supports.style") && vm.SelectedSupportNode!.Id == node.Id
            && vm.Snapshot.Revision == revision, "Reset layout restores defaults without changing selection or document");
        VerifyViewContracts(Path.GetFullPath(Path.Combine(Path.GetDirectoryName(fixture)!, "..")));
        Console.WriteLine($"Managed workbench UX: {checks} checks passed.");
    }
    private static void LayoutPersistence()
    {
        var layout = new WorkbenchLayout();
        for (int mode = 0; mode < 4; ++mode)
        {
            layout.SelectWorkspace(mode);
            Require(layout.DockOpen == (mode is 0 or 3) && layout.Tabs.Contains("Timeline"), "Workspace defaults preserve Timeline access");
        }
        layout.SelectWorkspace(2); layout.DockHeight = 333; layout.BrowserWidth = 280; layout.InspectorWidth = 360;
        layout.OpenTab("Diagnostics"); layout.DockOpen = false; layout.SetExpanded("train.bogies", true);
        layout.SelectWorkspace(1); layout.OpenTab("Timeline"); layout.SelectWorkspace(2);
        var path = Path.Combine(Path.GetTempPath(), $"quantum-layout-{Guid.NewGuid():N}.json");
        try
        {
            layout.Save(path); var restored = WorkbenchLayout.Load(path, out var error);
            Require(error is null && restored.Workspace == 2 && !restored.DockOpen && restored.SelectedTab == "Diagnostics"
                && restored.DockHeight == 333 && restored.BrowserWidth == 280 && restored.InspectorWidth == 360 && restored.Expanded("train.bogies"), "Roundtrip restores panes, collapsed state, selected tab, workspace and sections");
            restored.SelectWorkspace(1); Require(restored.SelectedTab == "Timeline" && restored.DockOpen, "Other workspace preferences persist independently");
            restored.SelectWorkspace(2); restored.ResetWorkspace();
            Require(restored.SelectedTab == "Summary" && !restored.DockOpen && restored.DockHeight == 280
                && restored.BrowserWidth == 240 && restored.InspectorWidth == 310 && !restored.Expanded("train.bogies"), "Reset restores pane dimensions and section defaults");
            restored.SelectWorkspace(1); Require(restored.DockOpen, "Reset does not affect another workspace");
            File.WriteAllText(path, "{invalid"); restored = WorkbenchLayout.Load(path, out error);
            Require(error is not null && restored.Workspace == 0 && restored.DockOpen, "Malformed settings recover with visible error/defaults");
            File.WriteAllText(path, "{\"Version\":1,\"Workspace\":99,\"Panes\":{\"0\":{\"DockHeight\":1,\"BrowserWidth\":5000,\"InspectorWidth\":2,\"Tab\":\"Unknown\"}}}");
            restored = WorkbenchLayout.Load(path, out error); restored.SelectWorkspace(0);
            Require(restored.DockHeight == 160 && restored.BrowserWidth == 420 && restored.InspectorWidth == 250 && restored.SelectedTab == "Timeline", "Unusable saved sizes and unknown tabs are bounded");
        }
        finally { File.Delete(path); File.Delete(path + ".tmp"); }
    }
    // Run synchronously in a separate console invocation, before any worker
    // continuations: Avalonia binding teardown requires the creating UI thread.
    public static void VerifyDockBinding()
    {
        var layout = new WorkbenchLayout();
        var tabs = new ListBox { DataContext = layout };
        using var itemsBinding = tabs.Bind(ItemsControl.ItemsSourceProperty, new Binding(nameof(WorkbenchLayout.Tabs)));
        using var selectionBinding = tabs.Bind(SelectingItemsControl.SelectedItemProperty,
            new Binding(nameof(WorkbenchLayout.SelectedTab)) { Mode = BindingMode.OneWay });
        layout.SelectWorkspace(2);
        Require(!layout.DockOpen && layout.SelectedTab == "Summary" && tabs.SelectedItem as string == "Summary",
            "Real tab binding cannot write the previous workspace tab back during ItemsSource replacement");
        layout.ResetWorkspace();
        Require(!layout.DockOpen && tabs.SelectedItem as string == "Summary", "Bound tab reset keeps the Train dock collapsed");
        layout.SelectWorkspace(3); layout.OpenTab("Runtime"); layout.DockOpen = false;
        layout.SelectWorkspace(0); layout.SelectWorkspace(3);
        Require(!layout.DockOpen && tabs.SelectedItem as string == "Runtime", "Bound workspace return preserves collapsed Runtime selection");
        tabs.DataContext = null;
        Console.WriteLine($"Avalonia dock binding: {checks} checks passed (no window or native viewport).");
    }
    private static void VerifyViewContracts(string root)
    {
        var path = Path.Combine(root, "editor", "managed", "Quantum.Editor");
        var authored = XDocument.Load(Path.Combine(path, "HardwareInspector.axaml"));
        var runtime = XDocument.Load(Path.Combine(path, "HardwareRuntimePane.axaml"));
        string[] runtimeCommands = ["DispatchCommand", "TransferMoveCommand", "OpenGateCommand", "CloseGateCommand", "LockGateCommand", "UnlockGateCommand", "HardwareAvailableCommand", "HardwareUnavailableCommand"];
        foreach (var command in runtimeCommands)
        {
            Require(!authored.Descendants().Attributes("Command").Any(a => a.Value.Contains(command))
                && runtime.Descendants().Attributes("Command").Any(a => a.Value.Contains(command)), "Runtime actions are separated from the authored Inspector");
        }
        Require(!authored.Descendants().Attributes("Text").Any(a => a.Value.Contains("HardwareTelemetry"))
            && !runtime.Descendants().Any(e => e.Name.LocalName == "TextBox"), "Runtime telemetry cannot look editable or appear among authored inputs");
        var timeline = XDocument.Load(Path.Combine(path, "TimelinePane.axaml"));
        Require(timeline.Descendants().Any(e => e.Name.LocalName == "ProfilePlot")
            && !timeline.Descendants().Any(e => e.Name.LocalName == "HardwareInspector"), "Timeline retains analytical graph without a duplicate hardware Inspector");
        var window = XDocument.Load(Path.Combine(path, "MainWindow.axaml"));
        Require(window.Descendants().Attributes("SelectedItem").Any(a => a.Value.Contains("Layout.SelectedTab, Mode=OneWay")),
            "Live dock uses the tested one-way selection binding");
    }
}
