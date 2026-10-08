using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Media;
using System.Globalization;
using Quantum.Managed;

namespace Quantum.Editor;

// Screen projection and pointer state only. Every preview curve is sampled
// by native code; this control never evaluates a transition or validates it.
public sealed class ProfilePlot : Control
{
    public static readonly StyledProperty<IReadOnlyList<ProfileSegment>> ProfilesProperty =
        AvaloniaProperty.Register<ProfilePlot, IReadOnlyList<ProfileSegment>>(nameof(Profiles), []);
    public static readonly StyledProperty<ProfileSegment?> SelectedProperty =
        AvaloniaProperty.Register<ProfilePlot, ProfileSegment?>(nameof(Selected));
    public static readonly StyledProperty<bool> CanEditProperty =
        AvaloniaProperty.Register<ProfilePlot, bool>(nameof(CanEdit), true);
    public IReadOnlyList<ProfileSegment> Profiles { get => GetValue(ProfilesProperty); set => SetValue(ProfilesProperty, value); }
    public ProfileSegment? Selected { get => GetValue(SelectedProperty); set => SetValue(SelectedProperty, value); }
    public bool CanEdit { get => GetValue(CanEditProperty); set => SetValue(CanEditProperty, value); }
    public event Action<ProfileSegment>? SegmentSelected;
    public event Action<ProfileSegment, ProfileOperation>? DragStarted;
    public event Action<double>? DragMoved;
    public event Action<bool>? DragFinished;

    public sealed record Handle(ProfileSegment Segment, ProfileOperation Operation, Point Position, bool Pinned);
    private sealed record Row(ProfileSegment[] Segments, Rect Plot, double Minimum, double Maximum, IBrush Color);
    private Row[] rows = [];
    private IReadOnlyList<ProfileSegment>? layoutProfiles;
    private Size layoutSize;
    private double viewBegin, viewEnd, profileExtent;
    private (uint Kind, ulong Id)? owner;
    private Handle? hover, selectedHandle, drag;
    private ProfileSegment? hoveredSegment;
    private IPointer? captured;
    private Point origin;
    private double initialValue, unitsPerPixel, panBegin;
    private bool panning, moved;

    static ProfilePlot() => AffectsRender<ProfilePlot>(ProfilesProperty, SelectedProperty, CanEditProperty);
    public ProfilePlot() { Focusable = true; ClipToBounds = true; }
    private static bool Same(ProfileSegment? a, ProfileSegment? b) => a is not null && b is not null
        && a.OwnerKind == b.OwnerKind && a.OwnerId == b.OwnerId && a.Channel == b.Channel && a.Id == b.Id;
    private static bool SameHandle(Handle? a, Handle? b) => a is not null && b is not null
        && Same(a.Segment, b.Segment) && a.Operation == b.Operation;
    private static bool IsBoundary(ProfileOperation operation) => operation is ProfileOperation.MoveBegin or ProfileOperation.MoveEnd;
    private static double Distance(Point delta) => Math.Sqrt(delta.X * delta.X + delta.Y * delta.Y);
    private static FormattedText Label(string text, IBrush color) =>
        new(text, CultureInfo.CurrentCulture, FlowDirection.LeftToRight, Typeface.Default, 11, color);
    private void EnsureLayout()
    {
        if (ReferenceEquals(layoutProfiles, Profiles) && layoutSize == Bounds.Size) return;
        layoutProfiles = Profiles; layoutSize = Bounds.Size;
        if (Profiles.Count == 0) { rows = []; owner = null; return; }
        var identity = (Profiles[0].OwnerKind, Profiles[0].OwnerId);
        var extent = Profiles.Max(p => p.End);
        if (owner != identity || viewEnd <= viewBegin || profileExtent != extent)
        {
            owner = identity; profileExtent = extent; viewBegin = 0; viewEnd = extent;
            selectedHandle = null;
        }
        var channels = Profiles.GroupBy(p => p.Channel).ToArray();
        var rowHeight = Math.Max(1, (Bounds.Height - 20) / channels.Length);
        var previous = rows;
        rows = channels.Select((channel, index) =>
        {
            var segments = channel.OrderBy(p => p.Begin).ToArray();
            var minimum = Math.Min(0, segments.Min(p => p.Samples.Min()));
            var maximum = Math.Max(0, segments.Max(p => p.Samples.Max()));
            var padding = Math.Max((maximum - minimum) * .15, segments[0].Unit == 0 ? .25 : .1);
            minimum -= padding; maximum += padding;
            // Freeze Y while dragging: moving a handle must not move its ruler.
            if (drag is not null && index < previous.Length)
            { minimum = previous[index].Minimum; maximum = previous[index].Maximum; }
            var color = new SolidColorBrush(Color.Parse(channel.Key switch
            { 0 => "#EEAA71", 1 => "#87C5DC", 2 => "#B6A0E3", _ => "#77C5C3" }));
            return new Row(segments, new Rect(54, index * rowHeight + 20,
                Math.Max(1, Bounds.Width - 70), Math.Max(12, rowHeight - 43)), minimum, maximum, color);
        }).ToArray();
    }
    private Point Project(Row row, double distance, double value) => new(
        row.Plot.Left + (distance - viewBegin) / (viewEnd - viewBegin) * row.Plot.Width,
        row.Plot.Top + (row.Maximum - value) / (row.Maximum - row.Minimum) * row.Plot.Height);
    private IEnumerable<Handle> Handles(Row row)
    {
        for (var index = 0; index <= row.Segments.Length; ++index)
        {
            var begin = index == 0;
            var segment = row.Segments[begin ? 0 : index - 1];
            var distance = begin ? segment.Begin : segment.End;
            var point = Project(row, distance, begin ? segment.ValueBegin : segment.ValueEnd);
            yield return new Handle(segment, begin ? ProfileOperation.BeginValue : ProfileOperation.EndValue, point, false);
            yield return new Handle(segment, begin ? ProfileOperation.MoveBegin : ProfileOperation.MoveEnd,
                new Point(point.X, row.Plot.Bottom + 10), begin || index == row.Segments.Length);
        }
    }
    public Handle? HitHandle(Point point)
    {
        EnsureLayout();
        return rows.SelectMany(Handles).Where(h => h.Position.X >= 48 && h.Position.X <= Bounds.Width - 10)
            .Select(h => (Handle: h, Distance: Distance(h.Position - point)))
            .Where(h => h.Distance <= 8).OrderBy(h => h.Distance)
            .ThenBy(h => Same(h.Handle.Segment, Selected) ? 0 : 1).Select(h => h.Handle).FirstOrDefault();
    }
    public ProfileSegment? HitSegment(Point point)
    {
        EnsureLayout();
        var nearest = 8.0; ProfileSegment? hit = null;
        foreach (var row in rows)
        {
            if (!row.Plot.Inflate(8).Contains(point)) continue;
            foreach (var segment in row.Segments)
                for (var i = 1; i < segment.Samples.Count; ++i)
                {
                    Point Sample(int sample) => Project(row, segment.Begin + (segment.End - segment.Begin) * sample / (segment.Samples.Count - 1), segment.Samples[sample]);
                    var a = Sample(i - 1); var b = Sample(i); var ab = b - a;
                    var squared = ab.X * ab.X + ab.Y * ab.Y;
                    var ap = point - a;
                    var t = squared == 0 ? 0 : Math.Clamp((ap.X * ab.X + ap.Y * ab.Y) / squared, 0, 1);
                    var distance = Distance(point - (a + ab * t));
                    if (distance < nearest) { nearest = distance; hit = segment; }
                }
        }
        return hit;
    }
    public ProfileSegment? SelectAt(Point point)
    {
        var handle = HitHandle(point);
        var segment = handle?.Segment ?? HitSegment(point);
        if (segment is null) return null;
        SegmentSelected?.Invoke(segment);
        selectedHandle = handle;
        InvalidateVisual();
        return segment;
    }
    public void FrameProfile()
    {
        if (captured is not null) return;
        viewBegin = 0; viewEnd = Profiles.Count == 0 ? 1 : Profiles.Max(p => p.End);
        layoutProfiles = null; EnsureLayout(); InvalidateVisual();
    }
    protected override void OnPropertyChanged(AvaloniaPropertyChangedEventArgs change)
    {
        base.OnPropertyChanged(change);
        if (change.Property == SelectedProperty && selectedHandle is not null && !Same(selectedHandle.Segment, Selected)) selectedHandle = null;
        if (change.Property == CanEditProperty && !CanEdit) CancelInteraction();
    }
    public override void Render(DrawingContext context)
    {
        base.Render(context); EnsureLayout();
        using var clip = context.PushClip(new Rect(Bounds.Size));
        context.FillRectangle(new SolidColorBrush(Color.Parse("#171D24")), new Rect(Bounds.Size));
        if (rows.Length == 0)
        { context.DrawText(Label("Select a scalar profile, or enable Custom acceleration.", Brushes.LightGray), new Point(12, 16)); return; }
        foreach (var row in rows)
        {
            var first = row.Segments[0];
            context.DrawText(Label($"{first.ChannelName} · {first.Units}", row.Color), new Point(54, row.Plot.Top - 18));
            context.DrawText(Label($"{row.Maximum:G3}", Brushes.Gray), new Point(2, row.Plot.Top));
            context.DrawText(Label($"{row.Minimum:G3}", Brushes.Gray), new Point(2, row.Plot.Bottom - 10));
            context.DrawLine(new Pen(Brushes.DimGray), Project(row, viewBegin, 0), Project(row, viewEnd, 0));
            foreach (var segment in row.Segments)
            {
                var selected = Same(segment, Selected);
                var left = Math.Max(row.Plot.Left, Project(row, segment.Begin, 0).X);
                var right = Math.Min(row.Plot.Right, Project(row, segment.End, 0).X);
                if (selected && right > left) context.FillRectangle(new SolidColorBrush(Color.Parse("#273746")), new Rect(left, row.Plot.Top, right - left, row.Plot.Height));
                using var rowClip = context.PushClip(row.Plot.Inflate(7));
                var pen = new Pen(row.Color, selected ? 3 : Same(segment, hoveredSegment) ? 2.4 : 1.3);
                for (var sample = 1; sample < segment.Samples.Count; ++sample)
                {
                    Point Sample(int i) => Project(row, segment.Begin + (segment.End - segment.Begin) * i / (segment.Samples.Count - 1), segment.Samples[i]);
                    context.DrawLine(pen, Sample(sample - 1), Sample(sample));
                }
            }
            foreach (var handle in Handles(row))
            {
                if (handle.Position.X < row.Plot.Left - 2 || handle.Position.X > row.Plot.Right + 2) continue;
                var active = SameHandle(handle, hover) || SameHandle(handle, selectedHandle);
                var fill = handle.Pinned ? Brushes.DimGray : active ? Brushes.White : row.Color;
                var outline = new Pen(active ? Brushes.White : row.Color, active ? 2 : 1);
                if (IsBoundary(handle.Operation))
                {
                    var rect = new Rect(handle.Position.X - 4, handle.Position.Y - 4, 8, 8);
                    context.DrawRectangle(fill, outline, rect);
                    if (handle.Pinned) context.DrawLine(new Pen(Brushes.LightGray), handle.Position + new Vector(-3, -5), handle.Position + new Vector(3, 5));
                }
                else context.DrawEllipse(fill, outline, handle.Position, active ? 5.5 : 4.5, active ? 5.5 : 4.5);
            }
        }
        context.DrawText(Label($"{viewBegin:F2} → {viewEnd:F2} m · local distance", Brushes.LightGray), new Point(54, Bounds.Height - 17));
    }
    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e); EnsureLayout();
        if (captured is not null || rows.Length == 0) return;
        var point = e.GetPosition(this); var properties = e.GetCurrentPoint(this).Properties;
        Focus();
        if (properties.IsMiddleButtonPressed)
        {
            panning = true; origin = point; panBegin = viewBegin;
            unitsPerPixel = (viewEnd - viewBegin) / rows[0].Plot.Width;
            captured = e.Pointer; captured.Capture(this); Cursor = new Cursor(StandardCursorType.Hand); e.Handled = true; return;
        }
        if (!properties.IsLeftButtonPressed) return;
        var segment = SelectAt(point);
        if (segment is null) return;
        var handle = selectedHandle;
        e.Handled = true;
        if (handle is null || handle.Pinned || !CanEdit) return;
        var row = rows.First(r => r.Segments[0].Channel == segment.Channel);
        origin = point; drag = handle; moved = false;
        initialValue = handle.Operation switch
        { ProfileOperation.BeginValue => segment.ValueBegin, ProfileOperation.EndValue => segment.ValueEnd,
          ProfileOperation.MoveBegin => segment.Begin, _ => segment.End };
        unitsPerPixel = IsBoundary(handle.Operation) ? (viewEnd - viewBegin) / row.Plot.Width : (row.Maximum - row.Minimum) / row.Plot.Height;
        captured = e.Pointer; captured.Capture(this);
        DragStarted?.Invoke(segment, handle.Operation);
    }
    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e); var point = e.GetPosition(this);
        if (panning)
        {
            var width = viewEnd - viewBegin; var extent = Profiles.Max(p => p.End);
            viewBegin = Math.Clamp(panBegin - (point.X - origin.X) * unitsPerPixel, 0, Math.Max(0, extent - width));
            viewEnd = viewBegin + width; InvalidateVisual(); e.Handled = true; return;
        }
        if (drag is not null)
        {
            if (moved || Distance(point - origin) >= 2)
            {
                moved = true;
                DragMoved?.Invoke(initialValue + (IsBoundary(drag.Operation) ? point.X - origin.X : origin.Y - point.Y) * unitsPerPixel);
            }
            e.Handled = true; return;
        }
        hover = HitHandle(point); hoveredSegment = hover?.Segment ?? HitSegment(point);
        Cursor = new Cursor(hover is { Pinned: true } ? StandardCursorType.No
            : hover is not null ? !CanEdit ? StandardCursorType.No : IsBoundary(hover.Operation) ? StandardCursorType.SizeWestEast : StandardCursorType.SizeNorthSouth
            : hoveredSegment is not null ? StandardCursorType.Hand : StandardCursorType.Arrow);
        ToolTip.SetTip(this, hover is { Pinned: true } ? "Distance pinned to the Region/device extent. The circle above can still edit its value."
            : hover is not null ? IsBoundary(hover.Operation) ? "Drag horizontally to move the shared interior boundary."
                : $"Drag vertically to edit {hover.Segment.ChannelName} ({hover.Segment.Units})."
            : hoveredSegment?.Summary);
        InvalidateVisual();
    }
    private void Release(bool commit)
    {
        var editing = drag is not null; var pointer = captured;
        drag = null; captured = null; panning = false; layoutProfiles = null;
        pointer?.Capture(null); Cursor = new Cursor(StandardCursorType.Arrow);
        if (editing) DragFinished?.Invoke(commit);
        InvalidateVisual();
    }
    public void CancelInteraction() => Release(false);
    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        if (captured is null) return;
        if (drag is not null && (moved || Distance(e.GetPosition(this) - origin) >= 2))
        {
            var point = e.GetPosition(this);
            DragMoved?.Invoke(initialValue + (IsBoundary(drag.Operation) ? point.X - origin.X : origin.Y - point.Y) * unitsPerPixel);
        }
        Release(true); e.Handled = true;
    }
    protected override void OnPointerCaptureLost(PointerCaptureLostEventArgs e) { base.OnPointerCaptureLost(e); if (captured is not null) Release(false); }
    protected override void OnPointerExited(PointerEventArgs e) { base.OnPointerExited(e); if (captured is null) { hover = null; hoveredSegment = null; InvalidateVisual(); } }
    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e) { CancelInteraction(); base.OnDetachedFromVisualTree(e); }
    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        if (e.Key == Key.Escape && captured is not null) { Release(false); e.Handled = true; }
        else if (e.Key == Key.F || e.Key == Key.Home) { FrameProfile(); e.Handled = true; }
    }
    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        base.OnPointerWheelChanged(e); EnsureLayout();
        if (captured is not null || rows.Length == 0) return;
        var extent = Profiles.Max(p => p.End);
        var fraction = Math.Clamp((e.GetPosition(this).X - rows[0].Plot.Left) / rows[0].Plot.Width, 0, 1);
        var station = viewBegin + fraction * (viewEnd - viewBegin);
        var width = Math.Clamp((viewEnd - viewBegin) * Math.Pow(1.2, -e.Delta.Y), extent / 1000, extent);
        viewBegin = Math.Clamp(station - width * fraction, 0, extent - width); viewEnd = viewBegin + width;
        InvalidateVisual(); e.Handled = true;
    }
}
