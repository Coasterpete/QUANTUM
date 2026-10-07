using Avalonia;

namespace Quantum.Editor;
internal static class Program
{
    [STAThread]
    public static int Main(string[] args) => BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);
    public static AppBuilder BuildAvaloniaApp() => AppBuilder.Configure<EditorApplication>()
        .UsePlatformDetect()
        .With(new Win32PlatformOptions { OverlayPopups = false })
        .With(new X11PlatformOptions { OverlayPopups = false })
        .LogToTrace();
}
