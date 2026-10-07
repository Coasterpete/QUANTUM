using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using System.Globalization;
using Quantum.Managed;

namespace Quantum.Editor;

// Projects native display samples into screen coordinates. Analytic profile
// evaluation, unit conversion and accepted edits all remain in C++.
public sealed class ProfilePlot : Control
{
    public static readonly StyledProperty<IReadOnlyList<ProfileSegment>> ProfilesProperty =
        AvaloniaProperty.Register<ProfilePlot, IReadOnlyList<ProfileSegment>>(nameof(Profiles), []);
    public static readonly StyledProperty<ProfileSegment?> SelectedProperty =
        AvaloniaProperty.Register<ProfilePlot, ProfileSegment?>(nameof(Selected));
    public IReadOnlyList<ProfileSegment> Profiles { get => GetValue(ProfilesProperty); set => SetValue(ProfilesProperty, value); }
    public ProfileSegment? Selected { get => GetValue(SelectedProperty); set => SetValue(SelectedProperty, value); }
    static ProfilePlot() => AffectsRender<ProfilePlot>(ProfilesProperty, SelectedProperty);
    private static FormattedText Label(string text, IBrush color) =>
        new(text, CultureInfo.CurrentCulture, FlowDirection.LeftToRight, Typeface.Default, 11, color);
    public override void Render(DrawingContext context)
    {
        base.Render(context);
        using var clip = context.PushClip(new Rect(Bounds.Size));
        context.FillRectangle(new SolidColorBrush(Color.Parse("#171D24")), new Rect(Bounds.Size));
        if (Profiles.Count == 0)
        {
            context.DrawText(Label("No scalar profiles for this selection.", Brushes.LightGray), new Point(12, 16));
            return;
        }
        var channels = Profiles.GroupBy(p => p.Channel).ToArray();
        var domainEnd = Profiles.Max(p => p.End);
        var left = 12.0; var width = Math.Max(1, Bounds.Width - 24);
        var rowHeight = Math.Max(1, (Bounds.Height - 20) / channels.Length);
        for (var row = 0; row < channels.Length; ++row)
        {
            var segments = channels[row].ToArray();
            var minimum = segments.Min(p => p.Samples.Min());
            var maximum = segments.Max(p => p.Samples.Max());
            var padding = Math.Max((maximum - minimum) * .15, .05);
            minimum -= padding; maximum += padding;
            var top = row * rowHeight + 22; var height = Math.Max(1, rowHeight - 30);
            var color = new SolidColorBrush(Color.Parse(segments[0].Channel switch
            { 0 => "#EEAA71", 1 => "#87C5DC", 2 => "#B6A0E3", _ => "#77C5C3" }));
            context.DrawText(Label($"{segments[0].ChannelName} · {segments[0].Units}", color), new Point(left, row * rowHeight + 4));
            context.DrawLine(new Pen(Brushes.DimGray), new Point(left, top + height), new Point(left + width, top + height));
            Point Project(double distance, double value) => new(left + distance / domainEnd * width,
                top + (maximum - value) / (maximum - minimum) * height);
            foreach (var segment in segments)
            {
                var pen = new Pen(color, segment == Selected ? 2.5 : 1.2);
                for (var sample = 1; sample < segment.Samples.Count; ++sample)
                {
                    var previousDistance = segment.Begin + (segment.End - segment.Begin) * (sample - 1) / (segment.Samples.Count - 1);
                    var distance = segment.Begin + (segment.End - segment.Begin) * sample / (segment.Samples.Count - 1);
                    context.DrawLine(pen, Project(previousDistance, segment.Samples[sample - 1]), Project(distance, segment.Samples[sample]));
                }
                context.DrawEllipse(color, null, Project(segment.Begin, segment.ValueBegin), 3, 3);
                context.DrawEllipse(color, null, Project(segment.End, segment.ValueEnd), 3, 3);
            }
        }
        context.DrawText(Label($"0 m                                     {domainEnd:F2} m · local distance", Brushes.LightGray),
            new Point(left, Math.Max(0, Bounds.Height - 17)));
    }
}
