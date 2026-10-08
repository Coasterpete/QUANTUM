using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.LogicalTree;
using Avalonia.Input;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Controls.Presenters;
using Avalonia.VisualTree;
using System.Diagnostics;
using System.Text.Json;

namespace Quantum.Editor;

internal static partial class RuntimeChecks
{
    // One bounded live-host pass. Visual acceptance remains the maintainer's.
    public static async Task RunWorkbenchSmoke(MainWindow window, string fixture, string report,
        IClassicDesktopStyleApplicationLifetime desktop)
    {
        var checks = new List<string>(); var watch = Stopwatch.StartNew();
        string? failure = null; var model = window.Model;
        var sizes = new List<object>();
        void Require(bool condition, string name)
        { if (!condition) throw new InvalidOperationException($"{name}: {model.Error}"); checks.Add(name); }
        async Task Settle()
        { await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!); await Task.Delay(100); }
        InspectorSection Section(string key) => window.GetLogicalDescendants().OfType<InspectorSection>().First(s => s.SectionKey == key);
        void CheckSize()
        {
            var viewport = window.HostedViewport.Bounds;
            Require(viewport.Width >= 300 && viewport.Height >= 160, $"Useful viewport at {window.Width} × {window.Height}");
            sizes.Add(new { width = window.Width, height = window.Height, viewport = new { viewport.Width, viewport.Height }, dock = window.TimelineBounds.Height });
        }
        model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
        try
        {
            await Task.Delay(300);
            Require(window.HostedViewport.IsReady && window.HostedViewport.FrameCount > 0, "Embedded Vulkan presents frames");
            model.PickFile = _ => Task.FromResult<string?>(fixture);
            await model.OpenCommand.ExecuteAsync(); Require(!model.HasError, "Open native Supports 2 acceptance scene");
            model.WorkspaceMode = 0; model.Layout.ResetWorkspace(); await Settle();
            Require(model.Layout.ShowTimeline && window.TimelinePlot.Bounds.Width > 100, "Track keeps Timeline arranged and reachable");
            CheckSize();
            Require(((EditorApplication)Application.Current!).FontDiagnostics.Count == 0, "Bundled UI/mono fonts load without fallback");
            Require(window.TimelinePlot.GridPointCount > 0 && window.TimelinePlot.GridPointCount <= TechnicalGrid.MaximumPoints * 4,
                "Timeline renders a bounded coordinate dot grid");
            var length = window.FindControl<TextBox>("LengthField")!;
            Require(length.FontFamily.Name == "Overpass Mono", "Track numerical field uses Overpass Mono");
            length.Focus(NavigationMethod.Tab); await Task.Delay(60);
            var focus = length.GetVisualDescendants().OfType<Border>().First(b => b.Name == "PART_BorderElement");
            Require(length.IsFocused && focus.BorderThickness.Left == 2, "Keyboard input focus has a strong outline");
            var route = window.FindControl<ListBox>("RouteList")!;
            var selectedRow = route.GetVisualDescendants().OfType<ListBoxItem>().First(r => r.IsSelected);
            var selectedSurface = selectedRow.GetVisualDescendants().OfType<ContentPresenter>().First(p => p.Name == "PART_ContentPresenter");
            Require(selectedSurface.BorderThickness.Left == 3, "Live route selection has an accent edge");
            // One Avalonia-side image, no screenshot campaign. The native child
            // viewport is checked through presentation counters and is not in this bitmap.
            using (var bitmap = new RenderTargetBitmap(new PixelSize((int)window.Bounds.Width, (int)window.Bounds.Height)))
            { bitmap.Render(window); bitmap.Save(report + ".png", PngBitmapEncoderOptions.Default); }

            model.WorkspaceMode = 2; model.Layout.ResetWorkspace(); await Settle();
            Require(model.HasCar && !model.Layout.DockOpen && Section("train.basics").IsExpanded
                && !Section("train.bogies").IsExpanded, "Train shows consist and common car settings with secondary dock");
            var car = model.SelectedCar!.Id;
            var bogies = Section("train.bogies"); bogies.IsExpanded = true; await Task.Delay(60);
            Require(bogies.IsExpanded && model.Layout.Expanded("train.bogies"), "Expand real Bogies section");
            bogies.IsExpanded = false; model.Layout.OpenTab("Summary"); await Settle();
            Require(model.Layout.ShowSummary && model.SelectedCar!.Id == car, "Collapse Bogies and open Summary without losing car selection");
            await model.ToggleDockCommand.ExecuteAsync(); await Settle();
            Require(!model.Layout.DockOpen && window.FindControl<Grid>("BottomDock")!.Bounds.Height < 70, "Bottom dock collapses without an empty content row");
            await model.OpenTimelineCommand.ExecuteAsync(); await Settle();
            Require(model.Layout.ShowTimeline && window.TimelinePlot.Bounds.Width > 100, "Timeline reopens from Train");

            model.WorkspaceMode = 1; model.Layout.ResetWorkspace(); await Settle();
            var structure = model.SupportStructures.First();
            await model.SelectSupportAsync(new(Quantum.Managed.SupportSelectionKind.Structure, structure.Id)); await Settle();
            Require(model.ShowSupportStructure && !model.ShowSupportMember && !model.Layout.DockOpen, "Supports uses structure context and secondary dock");
            Section("supports.style").IsExpanded = true; await Task.Delay(60); Section("supports.style").IsExpanded = false;
            var member = model.SupportMembers.First();
            await model.SelectSupportAsync(new(Quantum.Managed.SupportSelectionKind.Member, structure.Id, member.Id)); await Settle();
            Require(model.ShowSupportMember && !model.ShowSupportStructure, "Member Inspector hides structure configuration");
            model.SelectedInfrastructure = model.InfrastructureItems.First(); await Settle();
            Require(model.InspectInfrastructure && !model.ShowSupportMember, "Infrastructure selection changes Inspector context");

            model.WorkspaceMode = 3; model.Layout.ResetWorkspace(); await Settle();
            var launch = model.Snapshot!.Devices.FirstOrDefault(d => d.Kind == (uint)Quantum.Managed.DeviceKind.Launch);
            if (launch is null)
            {
                model.SelectedHardwareChoice = model.HardwareChoices.First(c => c.Kind == Quantum.Managed.DeviceKind.Launch);
                await model.CreateHardwareCommand.ExecuteAsync();
            }
            else model.SelectedDevice = launch;
            await Settle();
            Require(model.IsOperationsWorkspace && model.HasDevice && model.Layout.SelectedTab == "Profile", "Operations opens a hardware browser and authored profile page");
            Section("operations.placement").IsExpanded = true; await Task.Delay(60); Section("operations.placement").IsExpanded = false;
            model.Layout.OpenTab("Runtime"); await Settle();
            Require(model.Layout.ShowRuntime && !model.Layout.ShowTimeline, "Runtime telemetry has a separate read-only dock page");
            model.Layout.OpenTab("Profile"); await model.ToggleDockCommand.ExecuteAsync(); await Settle();
            Require(!model.Layout.DockOpen, "Operations profile collapses");
            await model.ToggleDockCommand.ExecuteAsync(); await Settle(); Require(model.Layout.ShowTimeline, "Operations profile reopens");
            Require(window.HostedViewport.FrameCount > 10 && !model.HasError, "Vulkan remains live across workspace/layout changes");
        }
        catch (Exception error) { failure = error.ToString(); }
        finally
        {
            var closed = new TaskCompletionSource(); window.Closed += (_, _) => closed.TrySetResult();
            window.Close();
            try { await closed.Task.WaitAsync(TimeSpan.FromSeconds(5)); checks.Add("Clean window/controller/viewport shutdown"); }
            catch (Exception error) { failure ??= error.ToString(); }
            await File.WriteAllTextAsync(report, JsonSerializer.Serialize(new
            {
                passed = failure is null, checks, sizes, elapsed_ms = watch.Elapsed.TotalMilliseconds,
                viewport_frames = window.HostedViewport.FrameCount, physical_pointer_testing = false,
                avalonia_surface_image = report + ".png", native_child_in_image = false,
                maintainer_visual_approval = "pending", failure
            }, new JsonSerializerOptions { WriteIndented = true }));
            desktop.Shutdown(failure is null ? 0 : 1);
        }
    }
}
