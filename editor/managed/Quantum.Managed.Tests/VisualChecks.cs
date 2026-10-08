using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Platform;
using Avalonia.Controls.Presenters;
using Avalonia.VisualTree;
using Quantum.Editor;
using System.Xml.Linq;

internal static class VisualChecks
{
    private static int checks;
    private static void Require(bool value, string name) { ++checks; if (!value) throw new Exception(name); }
    public static void Run(string root)
    {
        // Initializes fonts/resources on the creating thread, without a window,
        // desktop lifetime, native Session or Vulkan viewport.
        AppBuilder.Configure<EditorApplication>().UsePlatformDetect().SetupWithoutStarting();
        var app = (EditorApplication)Application.Current!;
        Require(app.FontDiagnostics.Count == 0, "All bundled font faces resolve without fallback");
        object Resource(string key)
        {
            Require(app.TryFindResource(key, out var result) && result is not null, $"Resource {key} resolves");
            return result!;
        }
        foreach (var (key, family, weight) in new[]
        {
            ("QuantumUIFont", "Overpass", FontWeight.Normal),
            ("QuantumUIFontMedium", "Overpass", FontWeight.SemiBold),
            ("QuantumUIFontBold", "Overpass", FontWeight.SemiBold),
            ("QuantumMonoFont", "Overpass Mono", FontWeight.Normal)
        })
        {
            Require(Resource(key) is FontFamily, $"{key} is a font token");
            var resolved = FontManager.Current.TryGetGlyphTypeface(new Typeface((FontFamily)Resource(key), weight: weight), out var glyph);
            Console.WriteLine($"Font {key}: {(string.IsNullOrEmpty(glyph?.TypographicFamilyName) ? glyph?.FamilyName : glyph.TypographicFamilyName)}, {glyph?.Weight}");
            Require(resolved && glyph is not null && (string.IsNullOrEmpty(glyph.TypographicFamilyName) ? glyph.FamilyName : glyph.TypographicFamilyName) == family && glyph.Weight == weight, $"{key} resolves its bundled face");
        }
        foreach (var file in new[] { "overpass-regular.otf", "overpass-semibold.otf", "overpass-mono-regular.otf" })
        {
            using var asset = AssetLoader.Open(new Uri($"avares://Quantum.Editor/Assets/{file}"));
            Require(asset.Length > 1000, $"Packaged font {file}");
        }
        Require(File.Exists(Path.Combine(AppContext.BaseDirectory, "licenses", "Overpass", "LICENSE.md")), "Font license accompanies runtime");
        var mono = Resource("QuantumMonoFont");
        app.Resources["QuantumMonoFont"] = new FontFamily("avares://Quantum.Editor/Missing#Unavailable");
        var warnings = QuantumFonts.Initialize(app);
        Require(warnings.Count == 1 && warnings[0].Contains("QuantumMonoFont"), "Missing family produces a named fallback diagnostic");
        Require(FontManager.Current.TryGetGlyphTypeface(new Typeface((FontFamily)Resource("QuantumMonoFont")), out _), "Fallback remains usable");
        app.Resources["QuantumMonoFont"] = mono;

        foreach (var (foreground, background) in new[]
        {
            ("QuantumTextPrimary", "QuantumBackground"), ("QuantumTextSecondary", "QuantumBackground"),
            ("QuantumTextSecondary", "QuantumSurface1"), ("QuantumTextPrimary", "QuantumInputBackground"),
            ("QuantumTextPrimary", "QuantumSelection"), ("QuantumTextPrimary", "QuantumSelectionHover"),
            ("QuantumTextSecondary", "QuantumSelectionHover"), ("QuantumError", "QuantumErrorSurface"),
            ("QuantumRuntime", "QuantumRuntimeSurface"), ("QuantumOnAccent", "QuantumAccent"),
            ("QuantumOnAccent", "QuantumAccentPressed"), ("QuantumTextDisabled", "QuantumSurface0")
        })
        {
            var ratio = Contrast(((ISolidColorBrush)Resource(foreground)).Color, ((ISolidColorBrush)Resource(background)).Color);
            Console.WriteLine($"Contrast {foreground}/{background}: {ratio:F2}:1");
            Require(ratio >= 4.5, $"Readable {foreground}/{background}");
        }
        VerifyViews(root);
        VerifyControls();
        VerifyGrid();
        Console.WriteLine($"Managed visual resources/contrast/grid: {checks} checks passed (no live window).");
    }
    private static double Contrast(Color a, Color b)
    {
        static double Luminance(Color c)
        {
            static double Linear(byte v) { var s = v / 255.0; return s <= .04045 ? s / 12.92 : Math.Pow((s + .055) / 1.055, 2.4); }
            return .2126 * Linear(c.R) + .7152 * Linear(c.G) + .0722 * Linear(c.B);
        }
        var x = Luminance(a); var y = Luminance(b);
        return (Math.Max(x, y) + .05) / (Math.Min(x, y) + .05);
    }
    private static void VerifyViews(string root)
    {
        var directory = Path.Combine(root, "editor", "managed", "Quantum.Editor");
        foreach (var file in Directory.GetFiles(directory, "*.axaml"))
        {
            var view = XDocument.Load(file);
            Require(!view.Descendants().Attributes().Any(a => a.Name.LocalName is "FontFamily"
                || (a.Name.LocalName is "Foreground" or "Background" or "BorderBrush" && a.Value.StartsWith('#'))), $"Semantic colors/fonts: {Path.GetFileName(file)}");
        }
        foreach (var (file, binding) in new[]
        {
            ("MainWindow.axaml", "Length"), ("TrackLayoutInspector.axaml", "LayoutZ"),
            ("TrainInspector.axaml", "Value"), ("TimelinePane.axaml", "ProfileBegin"),
            ("TimelinePane.axaml", "DistanceSnap"), ("HardwareInspector.axaml", "DeviceAcceleration"),
            ("SupportInspector.axaml", "NodeX"), ("SupportToolsPane.axaml", "FrameStation")
        })
        {
            var boxes = XDocument.Load(Path.Combine(directory, file)).Descendants().Where(e => e.Name.LocalName == "TextBox"
                && (string?)e.Attribute("Text") == $"{{Binding {binding}}}").ToArray();
            Require(boxes.Length > 0 && boxes.All(e => ((string?)e.Attribute("Classes"))?.Split(' ').Contains("numeric") == true), $"Mono numeric control: {file}/{binding}");
        }
        var styles = XDocument.Load(Path.Combine(directory, "Theme", "QuantumControls.axaml"));
        Require(styles.Descendants().Any(e => ((string?)e.Attribute("Selector"))?.Contains("TextBox.numeric") == true
            && e.Elements().Any(s => ((string?)s.Attribute("Value"))?.Contains("QuantumMonoFont") == true)), "Numeric style uses the replaceable mono token");
        var grid = File.ReadAllText(Path.Combine(directory, "TechnicalGrid.cs"));
        Require(!grid.Contains("Quantum.Managed") && !grid.Contains("SnapEnabled"), "Display grid has no native/snap dependency");
    }
    private static void VerifyControls()
    {
        var input = new TextBox { Text = "142.750", Classes = { "numeric" } };
        var primary = new Button { Content = "Apply", Classes = { "primary" } };
        var row = new ListBoxItem { Content = "Selected", IsSelected = true };
        var section = new InspectorSection { Header = "Basics" };
        var host = new Window { Content = new StackPanel { Children = { input, primary, row, section } } };
        host.Measure(new Size(400, 400)); host.Arrange(new Rect(0, 0, 400, 400));
        input.ApplyTemplate(); primary.ApplyTemplate(); row.ApplyTemplate(); section.ApplyTemplate();
        Require(input.FontFamily.Name == "Overpass Mono", "Actual numeric control inherits bundled mono");
        Require(input.Background is ISolidColorBrush, "Actual input has a semantic surface");
        var presenter = row.GetVisualDescendants().OfType<ContentPresenter>().First(c => c.Name == "PART_ContentPresenter");
        Require(((ISolidColorBrush)presenter.Background!).Color == Color.Parse("#285967") && presenter.BorderThickness.Left == 3,
            "Actual selected row has selection surface and a shape cue");
        ((IPseudoClasses)input.Classes).Set(":focus", true);
        var inputBorder = input.GetVisualDescendants().OfType<Border>().First(b => b.Name == "PART_BorderElement");
        Require(((ISolidColorBrush)inputBorder.BorderBrush!).Color == Color.Parse("#A4EEE6") && inputBorder.BorderThickness.Left == 2,
            "Focused input theme has a visible two-pixel outline");
        ((IPseudoClasses)input.Classes).Set(":focus", false);
        ((IPseudoClasses)row.Classes).Set(":pointerover", true);
        Require(((ISolidColorBrush)presenter.Background!).Color == Color.Parse("#2A5C69"), "Selected hover stays distinct from ordinary hover");
        Require(section.MinHeight == 34, "InspectorSection retains Expander theme/style key");
        host.Close();
    }
    private static void VerifyGrid()
    {
        var framed = TechnicalGrid.CalculateAxis(0, 100, 600);
        var zoomed = TechnicalGrid.CalculateAxis(0, 20, 600);
        Require(framed.Step == 5 && zoomed.Step == 1, "Zoom selects useful 1/2/5 coordinate steps");
        var pan = TechnicalGrid.CalculateAxis(1, 101, 600);
        Require(pan.Step == framed.Step && pan.Tick(0).Coordinate == 5 && pan.Tick(0).Pixel == 24, "Pan moves the coordinate origin, preserving interval");
        Require(framed.Tick(0).Major && !framed.Tick(1).Major && framed.Tick(5).Major, "Every fifth tick is major");
        var negative = TechnicalGrid.CalculateAxis(-10, 10, 600);
        Require(Enumerable.Range(0, negative.Count).Select(negative.Tick).Any(t => t.Coordinate == 0 && t.Major), "Signed channels share a zero anchor");
        foreach (var (begin, end, pixels) in new[]
        {
            (0d, 0d, 600d), (0d, 100d, 0d), (100d, 0d, 600d), (double.NaN, 100d, 600d),
            (0d, double.PositiveInfinity, 600d), (0d, 100d, double.NaN),
            (-1e308, 1e308, 600d)
        }) Require(TechnicalGrid.CalculateAxis(begin, end, pixels).Count == 0, "Degenerate/nonfinite ranges skip safely");
        foreach (var span in new[] { 1e-300, 1e-12, .01, 1, 100, 1e12, 1e300 })
            foreach (var pixels in new[] { 1d, 100, 600, 10000, 1e12 })
            {
                var axis = TechnicalGrid.CalculateAxis(-span / 2, span / 2, pixels);
                Require(axis.Count > 0 && axis.Count <= TechnicalGrid.MaximumTicks && axis.Count * axis.Count <= TechnicalGrid.MaximumPoints,
                    "Finite extreme zoom/size has bounded point count");
                foreach (var tick in Enumerable.Range(0, axis.Count).Select(axis.Tick))
                    Require(double.IsFinite(tick.Coordinate) && double.IsFinite(tick.Pixel) && tick.Pixel >= -1e-6 && tick.Pixel <= pixels + .001,
                        "Projected grid tick stays finite and in range");
            }
        var repeat = TechnicalGrid.CalculateAxis(0, 100, 600);
        Require(repeat == framed, "Identical ranges are stable during direct manipulation");
    }
}
