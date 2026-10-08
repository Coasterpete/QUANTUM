using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Primitives;
using Avalonia.Input;

namespace Quantum.Editor;

public sealed partial class MainWindow
{
    private void ActivateDockTab(object? sender)
    {
        if (sender is ListBox { SelectedItem: string tab }) model.Layout.OpenTab(tab);
    }
    private void DockTabTapped(object? sender, TappedEventArgs e) => ActivateDockTab(sender);
    private void DockTabKeyUp(object? sender, KeyEventArgs e)
    {
        if (e.Key is Key.Enter or Key.Space or Key.Left or Key.Right or Key.Home or Key.End) ActivateDockTab(sender);
    }
    private void InitializeLayout()
    {
        model.Layout.PropertyChanged += (_, _) => ApplyLayout();
        foreach (var splitter in new[] { BrowserSplitter, InspectorSplitter, DockSplitter })
        {
            splitter.AddHandler(Thumb.DragCompletedEvent, (_, _) => CaptureLayout());
            splitter.KeyUp += (_, _) => CaptureLayout();
        }
        SizeChanged += (_, _) => ApplyLayout();
        Opened += (_, _) => ApplyLayout();
        ApplyLayout();
    }
    private void CaptureLayout()
    {
        model.Layout.BrowserWidth = Workspace.ColumnDefinitions[0].ActualWidth;
        model.Layout.InspectorWidth = Workspace.ColumnDefinitions[4].ActualWidth;
        if (model.Layout.DockOpen) model.Layout.DockHeight = Math.Max(160, BottomDock.Bounds.Height - DockHeaderHeight);
    }
    private double DockHeaderHeight => Math.Max(38, BottomDock.RowDefinitions[0].ActualHeight);
    private void ApplyLayout()
    {
        var availableWidth = Math.Max(MinWidth, Bounds.Width) - 14;
        var browser = Math.Clamp(model.Layout.BrowserWidth, 190, Math.Max(190, availableWidth - 250 - 300));
        var inspector = Math.Clamp(model.Layout.InspectorWidth, 250, Math.Max(250, availableWidth - browser - 300));
        Workspace.ColumnDefinitions[0].Width = new GridLength(browser);
        Workspace.ColumnDefinitions[2].Width = new GridLength(1, GridUnitType.Star);
        Workspace.ColumnDefinitions[4].Width = new GridLength(inspector);
        // Keep the user's requested size; a smaller window only constrains the
        // arranged size, so maximizing restores the saved workspace dimensions.
        var chrome = new[] { 0, 1, 5, 6 }.Sum(i => Workbench.RowDefinitions[i].ActualHeight);
        var availableHeight = Math.Max(160, Math.Max(MinHeight, Bounds.Height) - chrome - Workspace.MinHeight - DockHeaderHeight - 7);
        Workbench.RowDefinitions[2].Height = new GridLength(1, GridUnitType.Star);
        Workbench.RowDefinitions[3].Height = new GridLength(model.Layout.DockOpen ? 7 : 0);
        Workbench.RowDefinitions[4].Height = model.Layout.DockOpen
            ? new GridLength(Math.Min(model.Layout.DockHeight, availableHeight) + DockHeaderHeight) : GridLength.Auto;
    }
    private void SaveLayout()
    {
        try { model.Layout.Save(layoutSettingsPath); }
        catch (Exception failure) when (failure is IOException or UnauthorizedAccessException)
        { model.Error = $"Could not save workspace layout: {failure.Message}"; }
    }
}
