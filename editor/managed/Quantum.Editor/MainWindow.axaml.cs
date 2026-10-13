using Avalonia;
using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Input;
using Avalonia.Platform.Storage;
using Avalonia.Platform;
using Avalonia.Threading;
using Quantum.Managed;

namespace Quantum.Editor;
public sealed partial class MainWindow : Window
{
    private readonly SessionController controller;
    private readonly EditorViewModel model;
    private ViewportHost viewport;
    internal EditorViewModel Model => model;
    internal ViewportHost HostedViewport => viewport;
    internal Rect TimelineBounds => Timeline.Bounds;
    internal ProfilePlot TimelinePlot => Timeline.AuthoringPlot;
    private bool closeApproved, closePending;
    private readonly string layoutSettingsPath;
    public MainWindow() : this(WorkbenchLayout.SettingsPath) { }
    internal MainWindow(string settingsPath)
    {
        layoutSettingsPath = settingsPath;
        InitializeComponent();
        using (var icon = AssetLoader.Open(new Uri("avares://Quantum.Editor/Assets/quantum.ico")))
            Icon = new WindowIcon(icon);
        controller = new SessionController();
        var layout = WorkbenchLayout.Load(layoutSettingsPath, out var layoutError);
        model = new EditorViewModel(controller, layout);
        DataContext = model;
        InitializeLayout();
        model.WorkspaceMode = layout.Workspace;
        if (layoutError is not null) model.Error = layoutError;
        AddHandler(KeyDownEvent, (_, e) =>
        {
            if (e.Key != Key.Escape || !model.InteractionActive) return;
            _ = model.EndTransformAsync(false); _ = model.EndProfileDragAsync(false); e.Handled = true;
        }, RoutingStrategies.Tunnel);
        model.PickFile = PickFile;
        model.ConfirmUnsaved = ConfirmUnsaved;
        viewport = Viewport;
        ConnectViewport(viewport);
        model.TransformCanceled += CancelViewportTransform;
        Deactivated += (_, _) => { _ = model.EndTransformAsync(false); };
        // Read Latest in a bounded UI timer: worker publication cannot queue
        // unbounded dispatcher callbacks while a file dialog blocks the UI.
        var snapshots = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(100) };
        snapshots.Tick += (_, _) => { if (controller.Latest is { } latest) model.Publish(latest); };
        controller.Failed += error => Dispatcher.UIThread.Post(() => model.Error = error);
        snapshots.Start();
        Closing += async (_, e) =>
        {
            if (closeApproved) return;
            e.Cancel = true;
            if (closePending) return;
            closePending = true;
            try
            {
                if (!await model.ConfirmClose()) return;
                SaveLayout();
                viewport.Shutdown();
                controller.Dispose();
                snapshots.Stop();
                closeApproved = true;
                Close();
            }
            catch (Exception error) { model.Error = $"Could not close QUANTUM: {error.Message}"; }
            finally { closePending = false; }
        };
    }
    private void ConnectViewport(ViewportHost host)
    {
        host.Session = controller.Session;
        host.PresentationMode = (uint)Math.Max(0, PresentationChoice.SelectedIndex);
        host.Failed += error => model.Error = error;
        host.Shortcuts += HandleViewportShortcuts;
        host.TransformInput += intent => { _ = model.ReceiveTransformIntentAsync(intent); };
        host.ClosingViewport += () => { _ = model.EndTransformAsync(false); };
    }
    private void CancelViewportTransform()
    {
        if (!viewport.IsReady) return;
        try { viewport.Session?.CancelViewportTransform(); }
        catch (Exception error) { model.Error = error.Message; }
    }
    internal void RecreateViewport()
    {
        viewport.Shutdown();
        ViewportContainer.Child = null;
        viewport = new ViewportHost { Focusable = true };
        ConnectViewport(viewport);
        ViewportContainer.Child = viewport;
    }
    private async Task<string?> PickFile(bool save)
    {
        var filter = new FilePickerFileType("QUANTUM documents") { Patterns = ["*.quantum"] };
        if (save)
        {
            var file = await StorageProvider.SaveFilePickerAsync(new FilePickerSaveOptions
            {
                Title = "Save QUANTUM document", DefaultExtension = "quantum", SuggestedFileName = model.DocumentName == "Untitled" ? "Untitled.quantum" : model.DocumentName,
                FileTypeChoices = [filter], ShowOverwritePrompt = true
            });
            return file?.TryGetLocalPath();
        }
        var files = await StorageProvider.OpenFilePickerAsync(new FilePickerOpenOptions
        { Title = "Open QUANTUM document", AllowMultiple = false, FileTypeFilter = [filter] });
        return files.Count == 0 ? null : files[0].TryGetLocalPath();
    }
    private async Task<UnsavedChoice> ConfirmUnsaved()
    {
        var dialog = new Window { Title = "Unsaved changes", Width = 460, Height = 210, CanResize = false,
            WindowStartupLocation = WindowStartupLocation.CenterOwner };
        var save = new Button { Content = "Save", Classes = { "primary" } };
        var discard = new Button { Content = "Don't save" };
        var cancel = new Button { Content = "Cancel" };
        save.Click += (_, _) => dialog.Close(UnsavedChoice.Save);
        discard.Click += (_, _) => dialog.Close(UnsavedChoice.Discard);
        cancel.Click += (_, _) => dialog.Close(UnsavedChoice.Cancel);
        dialog.Content = new StackPanel { Margin = new Thickness(24), Spacing = 20, Children =
        {
            new TextBlock { Text = $"Save changes to {model.DocumentName}?", TextWrapping = Avalonia.Media.TextWrapping.Wrap },
            new TextBlock { Text = "Your accepted edits have not been saved.", Classes = { "muted" } },
            new StackPanel { Orientation = Avalonia.Layout.Orientation.Horizontal, Spacing = 8, Children = { save, discard, cancel } }
        } };
        return await dialog.ShowDialog<UnsavedChoice>(this);
    }
    private void HandleViewportShortcuts(uint intents)
    {
        if ((intents & 256) != 0) viewport.Focus();
        if ((intents & 512) != 0) { _ = model.EndTransformAsync(false); _ = model.EndProfileDragAsync(false); }
        foreach (var (bit, command) in new (uint, EditorCommand)[]
        {
            (1, model.UndoCommand), (2, model.RedoCommand), (4, model.SaveCommand),
            (8, model.OpenCommand), (16, model.NewCommand), (32, model.SaveAsCommand), (1024, model.OpenTimelineCommand)
        }) if ((intents & bit) != 0) command.Execute(null);
        if (model.IsSupportsWorkspace && (intents & (64 | 128)) != 0) SupportsTree.FocusStructures();
        else if (model.IsTrainWorkspace && (intents & (64 | 128)) != 0) TrainTree.FindControl<ListBox>("ConsistItems")?.Focus();
        else if (model.IsOperationsWorkspace && (intents & (64 | 128)) != 0) HardwareTree.FindControl<ListBox>("HardwareItems")?.Focus();
        else
        {
            if ((intents & 64) != 0) { if (LengthField.IsEffectivelyVisible) LengthField.Focus(); else RouteList.Focus(); }
            if ((intents & 128) != 0) RouteList.Focus();
        }
    }
    private void CloseClicked(object? sender, RoutedEventArgs e) => Close();
    private void PresentationChanged(object? sender, SelectionChangedEventArgs e)
    {
        if (viewport is null || sender is not ComboBox { SelectedIndex: >= 0 } choice) return;
        viewport.PresentationMode = (uint)choice.SelectedIndex;
        if (!viewport.IsReady) return;
        try { controller.Session.SetViewportPresentation((uint)choice.SelectedIndex); }
        catch (Exception error) { model.Error = error.Message; }
    }
}
