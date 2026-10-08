using System.Diagnostics;
using Quantum.Managed;

static void Require(bool value, string message) { if (!value) throw new Exception(message); }
static void Expect<T>(Action action) where T : Exception
{
    try { action(); } catch (T) { return; }
    throw new Exception($"Expected {typeof(T).Name}");
}
var fixture = args.Single();
for (var cycle = 0; cycle < 5; ++cycle)
{
    var session = new Session();
    session.Open(fixture);
    var before = session.Snapshot();
    Require(before.Available && before.Route.Count > 0 && !before.Dirty, "Loaded real document");
    Require(before.Profiles.Count > 0 && session.TransitionTypes.Count == 19, "Native authoring snapshot/catalog");
    Require(ReferenceEquals(before.Profiles, session.Snapshot().Profiles), "Authoring arrays are revision cached");
    var profile = before.Profiles[0];
    session.EditProfile(profile, before.Revision, ProfileOperation.EndValue, profile.ValueEnd + .01);
    var profileSnapshot = session.Snapshot();
    Require(profileSnapshot.Dirty && Math.Abs(profileSnapshot.Profiles[0].ValueEnd - profile.ValueEnd - .01) < 1e-12,
        "Native profile edit from wrapper");
    Expect<QuantumException>(() => session.EditProfile(profile, before.Revision, ProfileOperation.BeginValue, 0));
    session.Command(SessionCommand.Undo);
    before = session.Snapshot();
    Require(!before.Dirty && before.Profiles[0].ValueEnd == profile.ValueEnd, "Profile Undo restores baseline");
    var route = before.Route;
    var watch = Stopwatch.StartNew();
    for (var i = 0; i < 1000; ++i) Require(ReferenceEquals(route, session.Snapshot().Route), "Route is revision cached");
    Console.WriteLine($"Cached snapshot + mapping: {watch.Elapsed.TotalMilliseconds / 1000:F4} ms/call");
    var initialSpeed = before.InitialSpeed;
    session.Command(SessionCommand.SetInitialSpeed, before.Revision, initialSpeed + .01);
    var after = session.Snapshot();
    Require(after.Dirty && after.CanUndo && after.InitialSpeed == initialSpeed + .01, "Edit from wrapper");
    try { session.Command(SessionCommand.SetInitialSpeed, before.Revision, initialSpeed); throw new Exception("Stale edit was accepted"); }
    catch (QuantumException error) { Require(error.Code == 8 && error.Message.Contains("changed"), "Conflict translation"); }
    session.Command(SessionCommand.Undo);
    Require(session.Snapshot().InitialSpeed == initialSpeed && !session.Snapshot().Dirty, "Native Undo");
    session.Command(SessionCommand.Redo);
    Require(session.Snapshot().Dirty, "Native Redo");
    Expect<QuantumException>(() => session.Open("missing.quantum"));
    Require(session.Snapshot().Revision > before.Revision && session.Snapshot().Dirty, "Failed Open preserves state");
    session.Command(SessionCommand.Play);
    double updateMilliseconds = 0, physicsMilliseconds = 0;
    for (var frame = 0; frame < 30; ++frame)
    {
        watch.Restart();
        session.Update(1.0 / 60);
        updateMilliseconds += watch.Elapsed.TotalMilliseconds;
        physicsMilliseconds += session.Snapshot().PhysicsMs;
    }
    Console.WriteLine($"Simulation update including interop: {updateMilliseconds / 30:F4} ms/call; native physics: {physicsMilliseconds / 30:F4} ms/update");
    Require(session.Snapshot().Tick > 0, "Real preview update");
    session.Command(SessionCommand.Pause);
    var tick = session.Snapshot().Tick;
    session.Update(1.0 / 60);
    Require(session.Snapshot().Tick == tick, "Pause");
    session.Command(SessionCommand.Reset);
    Require(session.Snapshot().Tick == 0, "Reset");
    session.Dispose(); session.Dispose();
    Expect<ObjectDisposedException>(() => session.Snapshot());
    Expect<ObjectDisposedException>(() => session.New());
}
using (var controller = new SessionController())
{
    await controller.Submit(s => s.Open(fixture));
    Require(controller.Latest?.Available == true, "Controller publication before completion");
    await controller.Submit(s => s.Command(SessionCommand.Play));
    await Task.Delay(120);
    await controller.Submit(s => s.Command(SessionCommand.Pause));
    Require(controller.Latest?.Tick > 0, "Worker simulation");
}
using (var deviceSession = new Session())
{
    deviceSession.Open(Path.Combine(Path.GetDirectoryName(fixture)!, "operations-m1-acceleration-profiles.quantum"));
    var baseline = deviceSession.Snapshot();
    var device = baseline.Devices.First(d => d.Custom);
    deviceSession.SetDeviceAcceleration(device.Id, baseline.Revision, false, device.Acceleration);
    Require(!deviceSession.Snapshot().Devices.First(d => d.Id == device.Id).Custom, "Constant device mode");
    deviceSession.Command(SessionCommand.Undo);
    var custom = deviceSession.Snapshot();
    var profile = custom.Profiles.First(p => p.OwnerKind == 1 && p.OwnerId == device.Id);
    deviceSession.EditProfile(profile, custom.Revision, ProfileOperation.EndValue, profile.ValueEnd + .01);
    var changed = deviceSession.Snapshot();
    Require(changed.Dirty && changed.Devices.First(d => d.Id == device.Id).Custom, "Custom acceleration stays native");
    var temporary = Path.Combine(Path.GetTempPath(), $"quantum-device-wrapper-{Guid.NewGuid():N}.quantum");
    try
    {
        deviceSession.Save(temporary); deviceSession.New(); deviceSession.Open(temporary);
        Require(deviceSession.Snapshot().Profiles.First(p => p.OwnerKind == 1 && p.OwnerId == device.Id && p.Id == profile.Id).ValueEnd
            == profile.ValueEnd + .01, "Device acceleration serialization roundtrip");
        deviceSession.Command(SessionCommand.Play); deviceSession.Update(1.0 / 60);
        Require(deviceSession.Snapshot().Tick > 0, "Native simulation consumes authored devices");
    }
    finally { File.Delete(temporary); }
}
await TimelineChecks.Run();
var disposedController = new SessionController();
disposedController.Dispose(); disposedController.Dispose();
Expect<ObjectDisposedException>(() => disposedController.Submit(_ => { }));
Console.WriteLine("Managed wrapper checks passed.");
