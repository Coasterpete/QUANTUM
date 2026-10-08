using Avalonia.Controls;
using Avalonia.Interactivity;

namespace Quantum.Editor;
public sealed partial class TimelinePane : UserControl
{
    private EditorViewModel? model;
    internal ProfilePlot AuthoringPlot => Plot;
    public TimelinePane()
    {
        InitializeComponent();
        DataContextChanged += (_, _) =>
        {
            if (model is not null) { Plot.CancelInteraction(); model.ProfileDragCanceled -= Plot.CancelInteraction; }
            model = DataContext as EditorViewModel;
            if (model is not null) model.ProfileDragCanceled += Plot.CancelInteraction;
        };
        Plot.SegmentSelected += segment => { if (model is not null) model.SelectedProfile = model.VisibleProfiles.FirstOrDefault(p =>
            p.OwnerKind == segment.OwnerKind && p.OwnerId == segment.OwnerId && p.Channel == segment.Channel && p.Id == segment.Id); };
        Plot.DragStarted += async (segment, operation) => { if (model is not null) await model.BeginProfileDragAsync(segment, operation); };
        Plot.DragMoved += value => model?.QueueProfileDrag(value);
        Plot.DragFinished += async commit => { if (model is not null) await model.EndProfileDragAsync(commit); };
    }
    private void FrameProfileClicked(object? sender, RoutedEventArgs e) => Plot.FrameProfile();
}
