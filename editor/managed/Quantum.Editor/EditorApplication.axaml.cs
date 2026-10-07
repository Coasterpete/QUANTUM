using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Markup.Xaml;

namespace Quantum.Editor;
public sealed partial class EditorApplication : Application
{
    public override void Initialize() => AvaloniaXamlLoader.Load(this);
    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            try
            {
                var window = new MainWindow();
                desktop.MainWindow = window;
                if (desktop.Args is ["--verify-runtime", var fixture, var report])
                {
                    desktop.ShutdownMode = ShutdownMode.OnExplicitShutdown;
                    window.Opened += async (_, _) => await RuntimeChecks.Run(window, fixture, report, desktop);
                }
            }
            catch (Exception error)
            {
                desktop.MainWindow = new Window
                {
                    Title = "QUANTUM startup failed", Width = 620, Height = 260,
                    Content = new TextBlock { Margin = new Thickness(24), TextWrapping = Avalonia.Media.TextWrapping.Wrap,
                        Text = $"Could not start QUANTUM.\n\n{error.Message}\n\nBuild and stage the native runtime using editor/managed/Build.ps1." }
                };
            }
        }
        base.OnFrameworkInitializationCompleted();
    }
}
