using Avalonia.Controls;
using Avalonia.Platform;
using Avalonia.Input;
using Avalonia.Threading;
using Quantum.Managed;

namespace Quantum.Editor;
public sealed class ViewportHost : NativeControlHost
{
    public Session? Session { get; set; }
    public uint PresentationMode { get; set; }
    public event Action<string>? Failed;
    public event Action<uint>? Shortcuts;
    public bool IsReady { get; private set; }
    public uint FrameCount { get; private set; }
    private readonly DispatcherTimer timer;
    public ViewportHost()
    {
        GotFocus += (_, _) =>
        {
            if (!IsReady || Session is null) return;
            try { Session.Camera(ViewportAction.Focus); }
            catch (Exception error) { Failed?.Invoke(error.Message); }
        };
        // Parent-window activation can restore Avalonia focus while the child
        // receives ordinary SDL keys when focused directly. Keep camera keys
        // useful in both cases; document shortcuts use the window bindings.
        KeyDown += (_, e) =>
        {
            if (!IsReady || Session is null || e.KeyModifiers != KeyModifiers.None) return;
            var forward = e.Key == Key.W ? 1 : e.Key == Key.S ? -1 : 0;
            var lateral = e.Key == Key.A ? -1 : e.Key == Key.D ? 1 : 0;
            if (e.Key != Key.F && forward == 0 && lateral == 0) return;
            try
            {
                if (e.Key == Key.F) Session.Camera(ViewportAction.Frame);
                else Session.Camera(ViewportAction.Move, forward, lateral, seconds: .05);
                e.Handled = true;
            }
            catch (Exception error) { Failed?.Invoke(error.Message); }
        };
        // Vulkan already waits for FIFO presentation. A second 16 ms timer
        // wait adds a full frame of latency; background-priority dispatch
        // lets UI input run before the next native frame.
        timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(1) };
        timer.Tick += (_, _) =>
        {
            if (!IsReady || Session is null || Bounds.Width < 1 || Bounds.Height < 1 || !IsEffectivelyVisible
                || TopLevel.GetTopLevel(this) is Window { WindowState: WindowState.Minimized }) return;
            try
            {
                var scale = TopLevel.GetTopLevel(this)?.RenderScaling ?? 1;
                if (!Session.RenderViewport((uint)Math.Clamp(Math.Round(Bounds.Width * scale), 1, 16384),
                    (uint)Math.Clamp(Math.Round(Bounds.Height * scale), 1, 16384))) return;
                FrameCount++;
                var intents = Session.TakeViewportShortcuts();
                if (intents != 0) Shortcuts?.Invoke(intents);
            }
            catch (Exception error) { timer.Stop(); Failed?.Invoke(error.Message); }
        };
    }
    protected override IPlatformHandle CreateNativeControlCore(IPlatformHandle parent)
    {
        var child = base.CreateNativeControlCore(parent);
        try
        {
            Session?.OpenViewport(child.HandleDescriptor, child.Handle);
            Session?.SetViewportPresentation(PresentationMode);
            IsReady = Session is not null;
            timer.Start();
        }
        catch (Exception error) { Failed?.Invoke(error.Message); }
        return child;
    }
    protected override void DestroyNativeControlCore(IPlatformHandle child)
    {
        Shutdown();
        base.DestroyNativeControlCore(child);
    }
    public void Shutdown()
    {
        timer.Stop();
        if (!IsReady) return;
        Session?.CloseViewport();
        IsReady = false;
    }
}
