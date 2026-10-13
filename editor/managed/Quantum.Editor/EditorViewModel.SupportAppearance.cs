using System.Globalization;
using System.Text.Json;
using Quantum.Managed;

namespace Quantum.Editor;

public sealed partial class EditorViewModel
{
    private int supportFamily, structureMaterial, memberMaterial, foundationKind;
    private string structureTint = "#C7A378", memberTint = "#C7A378";
    private string structureRepeat = "1", structureAcross = ".6205357", memberRepeat = "1", memberAcross = ".6205357";
    private string structureRoughness = ".85", memberRoughness = ".85";
    private string frameStation = "10", frameWidth = "4", frameElevation = "-5", frameVertical = "-.5";
    private string framePost = ".3", frameBeam = ".45", frameBrace = ".15", frameFooting = ".9";
    private string foundationWidth = "0", foundationDepth = "0", foundationHeight = "0";
    public int SupportFamily { get => supportFamily; set { supportFamily = value; Changed(); } }
    public int StructureMaterial { get => structureMaterial; set { structureMaterial = value; Changed(); if (!updatingSupports && value > 0) SeedMaterialDraft(value, false); } }
    public int MemberMaterial { get => memberMaterial; set { memberMaterial = value; Changed(); if (!updatingSupports && value > 0) SeedMaterialDraft(value, true); } }
    public string StructureTint { get => structureTint; set { structureTint = value; Changed(); } }
    public string MemberTint { get => memberTint; set { memberTint = value; Changed(); } }
    public string StructureRepeat { get => structureRepeat; set { structureRepeat = value; Changed(); } }
    public string StructureAcross { get => structureAcross; set { structureAcross = value; Changed(); } }
    public string MemberRepeat { get => memberRepeat; set { memberRepeat = value; Changed(); } }
    public string MemberAcross { get => memberAcross; set { memberAcross = value; Changed(); } }
    public string StructureRoughness { get => structureRoughness; set { structureRoughness = value; Changed(); } }
    public string MemberRoughness { get => memberRoughness; set { memberRoughness = value; Changed(); } }
    public string FrameStation { get => frameStation; set { frameStation = value; Changed(); } }
    public string FrameWidth { get => frameWidth; set { frameWidth = value; Changed(); } }
    public string FrameElevation { get => frameElevation; set { frameElevation = value; Changed(); } }
    public string FrameVertical { get => frameVertical; set { frameVertical = value; Changed(); } }
    public string FramePost { get => framePost; set { framePost = value; Changed(); } }
    public string FrameBeam { get => frameBeam; set { frameBeam = value; Changed(); } }
    public string FrameBrace { get => frameBrace; set { frameBrace = value; Changed(); } }
    public string FrameFooting { get => frameFooting; set { frameFooting = value; Changed(); } }
    public int FoundationKind { get => foundationKind; set { foundationKind = value; Changed(); } }
    public string FoundationWidth { get => foundationWidth; set { foundationWidth = value; Changed(); } }
    public string FoundationDepth { get => foundationDepth; set { foundationDepth = value; Changed(); } }
    public string FoundationHeight { get => foundationHeight; set { foundationHeight = value; Changed(); } }
    public bool HasGeneratedFrame => selectedStructure?.Appearance?.Frame is not null;
    public string ProfileInheritance => selectedMember?.Appearance?.UseStyleProfile == true ? "Family profile default"
        : selectedMember?.Appearance?.UseFrameProfile == true ? "Generated frame profile" : "Explicit profile override";
    public EditorCommand ApplySupportStyleCommand { get; private set; } = null!;
    public EditorCommand ApplyStructureMaterialCommand { get; private set; } = null!;
    public EditorCommand ApplyMemberMaterialCommand { get; private set; } = null!;
    public EditorCommand UseStyleProfileCommand { get; private set; } = null!;
    public EditorCommand GenerateFrameCommand { get; private set; } = null!;
    public EditorCommand RegenerateFrameCommand { get; private set; } = null!;
    public EditorCommand ApplyFoundationShapeCommand { get; private set; } = null!;
    private static readonly JsonSerializerOptions SupportAppearanceJson = new(JsonSerializerDefaults.Web);
    private static readonly string[] MaterialNames = ["Timber", "PaintedSteel", "GalvanizedSteel", "Concrete"];
    private static SupportSurfaceSettings SurfaceDraft(int choice, string tint, string repeat, string across, string roughness)
    {
        if (choice < 1 || choice > 4) throw new ArgumentException("Choose a support material.");
        var hex = tint.Trim().TrimStart('#');
        if (hex.Length != 6 || !uint.TryParse(hex, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var color))
            throw new ArgumentException("Material tint must be a six-digit RGB hex color.");
        return new(MaterialNames[choice - 1], [(color >> 16) / 255f, ((color >> 8) & 255) / 255f, (color & 255) / 255f],
            (float)Finite(roughness, "Roughness"), choice == 3 ? 1 : 0, choice == 1 ? 1 : .2f,
            [(float)Finite(repeat, "Longitudinal repeat"), (float)Finite(across, "Across repeat")]);
    }
    private void InitializeSupportAppearanceCommands()
    {
        ApplySupportStyleCommand = Add(() => SupportMutation(() => new(SupportOperation.SetStyle,
            selectedStructure!.Id, Shape: (uint)SupportFamily)), () => HasSupportStructure);
        ApplyStructureMaterialCommand = Add(() => SupportMutation(() => new(SupportOperation.SetStructureSurface,
            selectedStructure!.Id, Text: StructureMaterial == 0 ? "" : JsonSerializer.Serialize(SurfaceDraft(StructureMaterial,
                StructureTint, StructureRepeat, StructureAcross, StructureRoughness), SupportAppearanceJson))), () => HasSupportStructure);
        ApplyMemberMaterialCommand = Add(() => SupportMutation(() => MemberRequest(SupportOperation.SetMemberSurface) with
            { Text = MemberMaterial == 0 ? "" : JsonSerializer.Serialize(SurfaceDraft(MemberMaterial,
                MemberTint, MemberRepeat, MemberAcross, MemberRoughness), SupportAppearanceJson) }), () => HasSupportMember);
        UseStyleProfileCommand = Add(() => SupportMutation(() => MemberRequest(SupportOperation.UseStyleProfile)), () => HasSupportMember);
        GenerateFrameCommand = Add(() => SupportMutation(() => FrameRequest(false)), () => Snapshot?.TotalLength > 0);
        RegenerateFrameCommand = Add(() => SupportMutation(() => FrameRequest(true)), () => HasGeneratedFrame);
        ApplyFoundationShapeCommand = Add(() => SupportMutation(() => NodeRequest(SupportOperation.SetFoundationShape) with
            { Shape = (uint)FoundationKind, Width = Finite(FoundationWidth, "Foundation width"),
                Height = Finite(FoundationDepth, "Foundation depth"), Wall = Finite(FoundationHeight, "Foundation height") }), () => IsFoundationSupportNode);
    }
    private SupportRequest FrameRequest(bool regenerate)
    {
        var post = Finite(FramePost, "Post size"); var beam = Finite(FrameBeam, "Beam depth"); var brace = Finite(FrameBrace, "Brace size");
        uint shape = SupportFamily switch { 1 => 0, 2 => 2, 3 => 3, _ => 1 };
        SupportProfileSettings Profile(double width, double height) => new(shape, width, shape == 0 ? width : height,
            shape == 1 ? 0 : Math.Min(width, height) * .06);
        var footing = Finite(FrameFooting, "Foundation footprint");
        var recipe = new SupportFrameSettings(Finite(FrameStation, "Frame station"), Finite(FrameWidth, "Frame width"),
            Finite(FrameElevation, "Foundation elevation"), Finite(FrameVertical, "Track offset"),
            Profile(post,post), Profile(post,beam), Profile(brace,brace*1.3), [footing,footing,footing*.5]);
        return new(SupportOperation.GenerateFrame, regenerate ? selectedStructure!.Id : 0, Shape: (uint)SupportFamily,
            Text: JsonSerializer.Serialize(recipe, SupportAppearanceJson));
    }
    private static string TintText(SupportSurfaceSettings surface) => $"#{(byte)Math.Clamp(surface.Tint[0]*255,0,255):X2}{(byte)Math.Clamp(surface.Tint[1]*255,0,255):X2}{(byte)Math.Clamp(surface.Tint[2]*255,0,255):X2}";
    private void SeedMaterialDraft(int choice, bool member)
    {
        var tint = choice switch { 2 => "#3B617A", 3 => "#ADB5BA", 4 => "#9E9E99", _ => "#C7A378" };
        var roughness = choice switch { 2 => ".42", 3 => ".48", 4 => ".92", _ => ".85" };
        var repeat = choice == 3 ? ".5" : "1";
        var across = choice == 1 ? ".6205357" : repeat;
        if (member) { MemberTint = tint; MemberRoughness = roughness; MemberRepeat = repeat; MemberAcross = across; }
        else { StructureTint = tint; StructureRoughness = roughness; StructureRepeat = repeat; StructureAcross = across; }
    }
    private void SeedSupportAppearance()
    {
        var style = selectedStructure?.Appearance;
        SupportFamily = (int)(style?.Family ?? 0);
        StructureMaterial = style?.Surface is { } authored ? Array.IndexOf(MaterialNames, authored.Material)+1 : 0;
        if (style?.Resolved is { } s)
        {
            StructureTint = TintText(s); StructureRepeat = Number(s.RepeatMeters[0]);
            StructureAcross = Number(s.RepeatMeters[1]); StructureRoughness = Number(s.Roughness);
        }
        var member = selectedMember?.Appearance;
        MemberMaterial = member?.Surface is { } authoredMember ? Array.IndexOf(MaterialNames, authoredMember.Material)+1 : 0;
        if (member?.Resolved is { } m)
        {
            MemberTint = TintText(m); MemberRepeat = Number(m.RepeatMeters[0]);
            MemberAcross = Number(m.RepeatMeters[1]); MemberRoughness = Number(m.Roughness);
        }
        if (style?.Frame is { } r)
        {
            FrameStation = Number(r.StationMeters); FrameWidth = Number(r.WidthMeters);
            FrameElevation = Number(r.FoundationElevationMeters); FrameVertical = Number(r.VerticalOffsetMeters);
            FramePost = Number(r.Post.Width); FrameBeam = Number(r.Beam.Height); FrameBrace = Number(r.Brace.Width);
            FrameFooting = Number(r.FoundationSizeMeters[0]);
        }
        var foundation = style?.Foundations.FirstOrDefault(f => f.Id == selectedNode?.Id);
        FoundationKind = (int)(foundation?.Kind ?? 0);
        FoundationWidth = Number(foundation?.Dimensions[0] ?? 0);
        FoundationDepth = Number(foundation?.Dimensions[1] ?? 0);
        FoundationHeight = Number(foundation?.Dimensions[2] ?? 0);
        Changed(nameof(HasGeneratedFrame)); Changed(nameof(ProfileInheritance));
    }
}
