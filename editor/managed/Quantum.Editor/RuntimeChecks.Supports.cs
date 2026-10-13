using Avalonia.Controls.ApplicationLifetimes;
using Quantum.Managed;
using System.Diagnostics;
using System.Text.Json;

namespace Quantum.Editor;

internal static partial class RuntimeChecks
{
    public static async Task RunSupportsSmoke(MainWindow window, string fixture, string report,
        IClassicDesktopStyleApplicationLifetime desktop)
    {
        var checks = new List<string>(); var watch = Stopwatch.StartNew();
        string? failure = null;
        var model = window.Model;
        var temporary = Path.Combine(Path.GetTempPath(), $"quantum-supports-smoke-{Guid.NewGuid():N}.quantum");
        void Require(bool condition, string name)
        { if (!condition) throw new InvalidOperationException($"{name}: {model.Error}"); checks.Add(name); }
        async Task Command(EditorCommand command, string name)
        {
            Require(command.CanExecute(null), $"{name} enabled");
            await command.ExecuteAsync(); Require(model.Error.Length == 0, name);
        }
        async Task Settle()
        {
            await Task.Delay(120);
            await model.Controller.Submit(_ => { }); model.Publish(model.Controller.Latest!);
        }
        try
        {
            await Task.Delay(300);
            Require(window.HostedViewport.IsReady && window.HostedViewport.FrameCount > 0, "Real embedded Vulkan presents");
            Require(window.Icon is not null && window.TimelineBounds.Height >= 200, "Canonical icon and Timeline retained");
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            model.PickFile = _ => Task.FromResult<string?>(fixture);
            await Command(model.OpenCommand, "Open native document");
            model.WorkspaceMode = 1; await Settle();
            await Command(model.CreateSupportStructureCommand, "Create support structure"); var structure = model.SelectedStructure!.Id;
            await Command(model.CreateSupportNodeCommand, "Create foundation node"); var a = model.SelectedSupportNode!.Id;
            model.NodeZ = "-5"; await Command(model.MoveSupportNodeCommand, "Numeric node move");
            await Command(model.SetSupportFoundationCommand, "Mark foundation");
            await Command(model.CreateSupportNodeCommand, "Create attached node"); var b = model.SelectedSupportNode!.Id;
            model.AttachmentStation = "10"; model.AttachmentLateral = "1"; model.AttachmentVertical = "-0.5";
            await Command(model.AttachSupportNodeCommand, "Native track attachment");
            model.MemberStartNode = model.SupportNodes.First(n => n.Id == a); model.MemberEndNode = model.SupportNodes.First(n => n.Id == b);
            await Command(model.CreateSupportMemberCommand, "Connect nodes"); var member = model.SelectedSupportMember!.Id;
            model.MemberShape = 1; model.MemberWidth = "0.3"; model.MemberHeight = "0.4"; model.MemberWall = "0.02";
            await Command(model.ApplySupportProfileCommand, "Rectangular hollow member");
            model.ConnectionEnd = 0; model.ConnectionTreatment = 9;
            await Command(model.ApplySupportTreatmentCommand, "Start Footing treatment");
            model.ConnectionEnd = 1; model.ConnectionTreatment = 6;
            await Command(model.ApplySupportTreatmentCommand, "End Saddle treatment");
            await Command(model.UndoCommand, "Native support Undo"); await Command(model.RedoCommand, "Native support Redo");
            await model.SelectSupportAsync(new(SupportSelectionKind.Node, structure, a)); await Settle();
            await Command(model.FrameSupportCommand, "Frame support node");
            await model.SelectSupportAsync(new(SupportSelectionKind.Member, structure, member)); await Settle();
            model.Controller.Session.Camera(ViewportAction.PickSupport, .5, .5); await Settle();
            Require(model.SelectedSupportNode?.Id == a && !model.HasSupportMember,
                "Synthetic native viewport node pick updates browser and Inspector");
            await model.SelectSupportAsync(new(SupportSelectionKind.Member, structure, member)); await Settle();
            await Command(model.FrameSupportCommand, "Frame support member");
            await model.SelectSupportAsync(new(SupportSelectionKind.Structure, structure)); await Settle();
            model.Controller.Session.Camera(ViewportAction.PickSupport, .5, .5); await Settle();
            Require(model.SelectedSupportMember?.Id == member, "Synthetic native solid-member pick updates Inspector");
            model.PickFile = _ => Task.FromResult<string?>(temporary);
            await Command(model.SaveAsCommand, "Native Supports save");
            await Command(model.NewCommand, "New document"); await Command(model.OpenCommand, "Reload Supports document");
            await model.SelectSupportAsync(new(SupportSelectionKind.Member, structure, member));
            Require(model.SelectedSupportMember is { Shape: 1, Wall: .02, StartConnection.Treatment: 9, EndConnection.Treatment: 6 }
                && model.SupportNodes.First(n => n.Id == b).Anchor == 1, "Authored support survives reload");
            model.WorkspaceMode = 0; await Settle();
            var profile = model.SelectedProfile!;
            await model.BeginProfileDragAsync(profile, ProfileOperation.EndValue);
            model.QueueProfileDrag(profile.ValueEnd + .01); await model.EndProfileDragAsync(true);
            Require(model.Error.Length == 0, "Timeline gesture after Supports editing");
            model.TimelineSource = 1; await Command(model.CreateLaunchCommand, "Track Device creation after Supports");
            model.DeviceMode = 1; model.DeviceAcceleration = "2";
            await Command(model.ApplyDeviceCommand, "Custom acceleration after Supports");
            var acceleration = model.SelectedProfile!;
            await model.BeginProfileDragAsync(acceleration, ProfileOperation.EndValue);
            model.QueueProfileDrag(2.1); await model.EndProfileDragAsync(true);
            Require(model.Error.Length == 0 && model.SelectedProfile!.ValueEnd == 2.1, "Custom acceleration gesture after Supports");
            await Command(model.PlayCommand, "Native simulation Play"); await Settle();
            await Command(model.PauseCommand, "Native simulation Pause");
            Require(model.Snapshot!.Tick > 0 && window.HostedViewport.FrameCount > 2,
                "Fixed-step simulation and hosted Vulkan continue presenting");
        }
        catch (Exception error) { failure = error.ToString(); }
        finally
        {
            File.Delete(temporary);
            await File.WriteAllTextAsync(report, JsonSerializer.Serialize(new
            {
                passed = failure is null, checks, elapsed_ms = watch.Elapsed.TotalMilliseconds,
                viewport_frames = window.HostedViewport.FrameCount, synthetic_native_picking = true,
                physical_pointer_testing = false, maintainer_visual_approval = "pending", failure
            }, new JsonSerializerOptions { WriteIndented = true }));
            model.ConfirmUnsaved = () => Task.FromResult(UnsavedChoice.Discard);
            window.Close(); await Task.Delay(150);
            desktop.Shutdown(failure is null ? 0 : 1);
        }
    }
}
