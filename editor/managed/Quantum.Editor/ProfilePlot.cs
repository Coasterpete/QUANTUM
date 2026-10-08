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
    private sealed record Row(ProfileSegment[] Segments, Rect Plot, double Minimum, double Maximum, string ColorKey);
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
    internal int GridPointCount { get; private set; }

    static ProfilePlot() => AffectsRender<ProfilePlot>(ProfilesProperty, SelectedProperty, CanEditProperty);
    public ProfilePlot() { Focusable = true; ClipToBounds = true; }
    private static bool Same(ProfileSegment? a, ProfileSegment? b) => a is not null && b is not null
        && a.OwnerKind == b.OwnerKind && a.OwnerId == b.OwnerId && a.Channel == b.Channel && a.Id == b.Id;
    private static bool SameHandle(Handle? a, Handle? b) => a is not null && b is not null
        && Same(a.Segment, b.Segment) && a.Operation == b.Operation;
    private static bool IsBoundary(ProfileOperation operation) => operation is ProfileOperation.MoveBegin or ProfileOperation.MoveEnd;
    private static double Distance(Point delta) => Math.Sqrt(delta.X * delta.X + delta.Y * delta.Y);
    private IBrush Brush(string key) => (IBrush)this.FindResource(key)!;
    private FormattedText Label(string text, IBrush color, bool technical = true) =>
        new(text, CultureInfo.CurrentCulture, FlowDirection.LeftToRight,
            new Typeface((FontFamily)this.FindResource(technical ? "QuantumMonoFont" : "QuantumUIFont")!),
            (double)this.FindResource("QuantumDiagnosticSize")!, color);
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
            var color = channel.Key switch
            { 0 => "QuantumGraphPrimary", 1 => "QuantumGraphSecondary", 2 => "QuantumGraphTertiary", _ => "QuantumAccent" };
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
        base.Render(context); EnsureLayout(); GridPointCount = 0;
        using var clip = context.PushClip(new Rect(Bounds.Size));
        context.FillRectangle(Brush("QuantumCanvas"), new Rect(Bounds.Size));
        if (rows.Length == 0)
        { context.DrawText(Label("Select a scalar profile, or enable Custom acceleration.", Brush("QuantumTextSecondary"), false), new Point(12, 16)); return; }
        foreach (var row in rows)
        {
            var rowColor = Brush(row.ColorKey);
            var first = row.Segments[0];
            context.DrawText(Label($"{first.ChannelName} · {first.Units}", rowColor, false), new Point(54, row.Plot.Top - 18));
            context.DrawText(Label($"{row.Maximum:G3}", Brush("QuantumTextSecondary")), new Point(2, row.Plot.Top));
            context.DrawText(Label($"{row.Minimum:G3}", Brush("QuantumTextSecondary")), new Point(2, row.Plot.Bottom - 10));
            // Selection stays beneath the coordinate grid, so the ruler remains visible.
            foreach (var segment in row.Segments.Where(s => Same(s, Selected)))
            {
                var left = Math.Max(row.Plot.Left, Project(row, segment.Begin, 0).X);
                var right = Math.Min(row.Plot.Right, Project(row, segment.End, 0).X);
                if (right > left) context.FillRectangle(Brush("QuantumGraphSelection"), new Rect(left, row.Plot.Top, right - left, row.Plot.Height));
            }
            DrawGrid(context, row);
            context.DrawLine(new Pen(Brush("QuantumGridAxis")), Project(row, viewBegin, 0), Project(row, viewEnd, 0));
            foreach (var segment in row.Segments)
            {
                var selected = Same(segment, Selected);
                using var rowClip = context.PushClip(row.Plot.Inflate(7));
                var pen = new Pen(rowColor, selected ? 3.2 : Same(segment, hoveredSegment) ? 2.6 : 2);
                for (var sample = 1; sample < segment.Samples.Count; ++sample)
                {
                    Point Sample(int i) => Project(row, segment.Begin + (segment.End - segment.Begin) * i / (segment.Samples.Count - 1), segment.Samples[i]);
                    context.DrawLine(pen, Sample(sample - 1), Sample(sample));
                }
            }
            foreach (var handle in Handles(row))
            {
                if (handle.Position.X < row.Plot.Left - 2 || handle.Position.X > row.Plot.Right + 2) continue;
                var selected = SameHandle(handle, selectedHandle);
                var active = SameHandle(handle, hover) || selected;
                var fill = handle.Pinned ? Brush("QuantumSurfaceRaised") : active ? Brush("QuantumHandleHover") : rowColor;
                var outline = new Pen(Brush(selected ? "QuantumHandleSelected" : active ? "QuantumHandleHover" : "QuantumHandle"), active ? 2 : 1.5);
                if (active) context.DrawEllipse(null, outline, handle.Position, selected ? 8 : 7, selected ? 8 : 7);
                if (IsBoundary(handle.Operation))
                {
                    var rect = new Rect(handle.Position.X - 5, handle.Position.Y - 5, 10, 10);
                    context.DrawRectangle(fill, outline, rect);
                    if (handle.Pinned)
                    {
                        var cross = new Pen(Brush("QuantumHandle"), 1.5);
                        context.DrawLine(cross, handle.Position + new Vector(-3, -3), handle.Position + new Vector(3, 3));
                        context.DrawLine(cross, handle.Position + new Vector(-3, 3), handle.Position + new Vector(3, -3));
                    }
                }
                else context.DrawEllipse(fill, outline, handle.Position, active ? 5.5 : 5, active ? 5.5 : 5);
            }
        }
        var handleLabel = hover ?? selectedHandle;
        if (handleLabel is not null && Profiles.FirstOrDefault(p => Same(p, handleLabel.Segment)) is { } current)
            handleLabel = handleLabel with { Segment = current };
        var readout = handleLabel is null ? $"{viewBegin:F2} → {viewEnd:F2} m · local distance"
            : IsBoundary(handleLabel.Operation)
                ? $"{(handleLabel.Operation == ProfileOperation.MoveBegin ? handleLabel.Segment.Begin : handleLabel.Segment.End):F2} m{(handleLabel.Pinned ? " · pinned" : " · boundary")}"
                : $"{(handleLabel.Operation == ProfileOperation.BeginValue ? handleLabel.Segment.ValueBegin : handleLabel.Segment.ValueEnd):F3} {handleLabel.Segment.Units} · value";
        context.DrawText(Label(readout, Brush("QuantumTextPrimary")), new Point(54, Bounds.Height - 17));
        if (IsFocused) context.DrawRectangle(null, new Pen(Brush("QuantumInputFocus"), 2), new Rect(Bounds.Size).Deflate(1));
    }
    private void DrawGrid(DrawingContext context, Row row)
    {
        var x = TechnicalGrid.CalculateAxis(viewBegin, viewEnd, row.Plot.Width);
        var y = TechnicalGrid.CalculateAxis(row.Minimum, row.Maximum, row.Plot.Height);
        GridPointCount += x.Count * y.Count;
        var minor = Brush("QuantumGridMinor"); var major = Brush("QuantumGridMajor");
        using (context.PushClip(row.Plot))
            for (var ix = 0; ix < x.Count; ++ix)
            {
                var xt = x.Tick(ix);
                for (var iy = 0; iy < y.Count; ++iy)
                {
                    var yt = y.Tick(iy); var strong = xt.Major && yt.Major;
                    context.DrawEllipse(strong ? major : minor, null,
                        new Point(row.Plot.Left + xt.Pixel, row.Plot.Bottom - yt.Pixel), strong ? 1.4 : 1, strong ? 1.4 : 1);
                }
            }
        // A shared distance ruler avoids repeating labels in every channel.
        if (row != rows[^1]) return;
        for (var i = 0; i < x.Count; ++i)
        {
            var tick = x.Tick(i);
            if (!tick.Major || tick.Pixel < 24 || tick.Pixel > row.Plot.Width - 28) continue;
            context.DrawText(Label($"{tick.Coordinate:G4} m", Brush("QuantumTextSecondary")),
                new Point(row.Plot.Left + tick.Pixel - 20, row.Plot.Bottom + 19));
        }
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
