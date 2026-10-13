using System.Collections.Concurrent;
using System.Diagnostics;

namespace Quantum.Managed;

// The application layer owns command ordering and UI publication. Native
// document/history/preview state stays in C++. One worker executes physics;
// window/surface/render operations stay on the Avalonia UI thread.
public sealed class SessionController : IDisposable
{
    public Session Session { get; } = new();
    private readonly ConcurrentQueue<(Action<Session> Action, TaskCompletionSource Completion)> commands = new();
    private readonly AutoResetEvent wake = new(false);
    private readonly Thread worker;
    private volatile bool stopping;
    private readonly object admission = new();
    private SessionSnapshot? latest;
    public SessionSnapshot? Latest => Volatile.Read(ref latest);
    public event Action<string>? Failed;
    public SessionController()
    {
        worker = new Thread(Run) { IsBackground = true, Name = "QUANTUM controller" };
        worker.Start();
    }
    public Task Submit(Action<Session> action)
    {
        var completion = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (admission)
        {
            ObjectDisposedException.ThrowIf(stopping, this);
            commands.Enqueue((action, completion)); wake.Set();
        }
        return completion.Task;
    }
    private void Run()
    {
        var clock = Stopwatch.StartNew(); var last = clock.Elapsed.TotalSeconds;
        while (!stopping)
        {
            while (commands.TryDequeue(out var pending))
            {
                try { pending.Action(Session); Volatile.Write(ref latest, Session.Snapshot()); pending.Completion.SetResult(); }
                catch (Exception error) { pending.Completion.SetException(error); }
                // File I/O/edit compilation is a timing discontinuity, not
                // simulation time, matching the native application convention.
                last = clock.Elapsed.TotalSeconds;
            }
            try
            {
                var now = clock.Elapsed.TotalSeconds;
                Session.Update(Math.Min(now - last, .25));
                last = now;
                var next = Session.Snapshot(); Volatile.Write(ref latest, next);
            }
            catch (Exception error) { Failed?.Invoke(error.Message); last = clock.Elapsed.TotalSeconds; }
            wake.WaitOne(16);
        }
        while (commands.TryDequeue(out var pending)) pending.Completion.SetCanceled();
    }
    public void Dispose()
    {
        lock (admission)
        {
            if (stopping) return;
            stopping = true; wake.Set();
        }
        worker.Join(); wake.Dispose(); Session.Dispose();
    }
}
