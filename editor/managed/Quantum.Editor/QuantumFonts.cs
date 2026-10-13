using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Platform;
using System.Diagnostics;

namespace Quantum.Editor;

// Startup validation belongs to the managed font resources, not the native font atlas.
public static class QuantumFonts
{
    public static IReadOnlyList<string> Initialize(Application application)
    {
        var diagnostics = new List<string>();
        Check("QuantumUIFont", "overpass-regular.otf", FontWeight.Normal, "Overpass", false);
        Check("QuantumUIFontMedium", "overpass-semibold.otf", FontWeight.SemiBold, "Overpass", false);
        Check("QuantumUIFontBold", "overpass-semibold.otf", FontWeight.SemiBold, "Overpass", false);
        Check("QuantumMonoFont", "overpass-mono-regular.otf", FontWeight.Normal, "Overpass Mono", true);
        return diagnostics;

        void Check(string key, string asset, FontWeight weight, string expected, bool mono)
        {
            string? failure = null;
            try
            {
                if (!AssetLoader.Exists(new Uri($"avares://Quantum.Editor/Assets/{asset}"))) failure = "bundled asset is missing";
                else if (!application.TryFindResource(key, out var resource) || resource is not FontFamily family)
                    failure = "font token is missing";
                else if (!FontManager.Current.TryGetGlyphTypeface(new Typeface(family, weight: weight), out var glyph) || glyph is null)
                    failure = "bundled family did not resolve";
                else if ((string.IsNullOrEmpty(glyph.TypographicFamilyName) ? glyph.FamilyName : glyph.TypographicFamilyName) != expected || glyph.Weight != weight)
                    failure = $"expected {expected}/{weight}, resolved {glyph.FamilyName}/{glyph.TypographicFamilyName}/{glyph.Weight}";
            }
            catch (Exception error) when (error is IOException or InvalidOperationException or ArgumentException)
            { failure = error.Message; }
            if (failure is null) return;
            var fallback = new FontFamily(mono ? "Cascadia Mono, Consolas, DejaVu Sans Mono, monospace" : "Segoe UI, Noto Sans, sans-serif");
            application.Resources[key] = fallback;
            var message = $"QUANTUM font fallback: {key}: {failure}; using {fallback.Name}.";
            diagnostics.Add(message);
            Trace.TraceWarning(message);
            Console.Error.WriteLine(message);
        }
    }
}
