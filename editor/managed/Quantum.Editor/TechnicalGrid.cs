namespace Quantum.Editor;

public readonly record struct GridTick(double Coordinate, double Pixel, bool Major);

// One axis, anchored at coordinate zero. Renderers combine X/Y ticks without
// storing a point cloud. There is deliberately no snap increment in this data.
public readonly record struct GridAxis(double Begin, double Span, double Pixels, double Step, long FirstIndex, int Count)
{
    public GridTick Tick(int offset)
    {
        if (offset < 0 || offset >= Count) throw new ArgumentOutOfRangeException(nameof(offset));
        var index = FirstIndex + offset;
        var coordinate = index * Step;
        return new(coordinate, (coordinate - Begin) / Span * Pixels, index % 5 == 0);
    }
}

public static class TechnicalGrid
{
    public const int MaximumTicks = 128;
    public const int MaximumPoints = MaximumTicks * MaximumTicks;
    public static GridAxis CalculateAxis(double begin, double end, double pixels)
    {
        var span = end - begin;
        if (!double.IsFinite(begin) || !double.IsFinite(end) || !double.IsFinite(pixels)
            || !double.IsFinite(span) || span <= 0 || pixels <= 0) return default;
        // At least 24 logical pixels between dots, and a hard upper bound even
        // for oversized/offscreen canvases. Nice steps change only at zoom thresholds.
        var desired = span / Math.Clamp(pixels / 24, 1, MaximumTicks - 1);
        var power = Math.Pow(10, Math.Floor(Math.Log10(desired)));
        if (!double.IsFinite(power) || power <= 0) return default;
        var fraction = desired / power;
        var step = (fraction <= 1 ? 1 : fraction <= 2 ? 2 : fraction <= 5 ? 5 : 10) * power;
        if (!double.IsFinite(step) || step <= 0) return default;
        var first = Math.Ceiling(begin / step);
        var last = Math.Floor(end / step);
        // Beyond this precision a double cannot distinguish successive integer ticks.
        if (!double.IsFinite(first) || !double.IsFinite(last) || Math.Abs(first) > 9e15 || Math.Abs(last) > 9e15) return default;
        var count = (int)Math.Clamp(last - first + 1, 0, MaximumTicks);
        return new(begin, span, pixels, step, (long)first, count);
    }
}
