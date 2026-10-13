using Avalonia;
using Avalonia.Controls;

namespace Quantum.Editor;

// A normal Expander with one stable user-settings key. It does not own a form.
public sealed class InspectorSection : Expander
{
    protected override Type StyleKeyOverride => typeof(Expander);
    public static readonly StyledProperty<string> SectionKeyProperty = AvaloniaProperty.Register<InspectorSection, string>(nameof(SectionKey), "");
    public static readonly StyledProperty<bool> DefaultExpandedProperty = AvaloniaProperty.Register<InspectorSection, bool>(nameof(DefaultExpanded));
    public string SectionKey { get => GetValue(SectionKeyProperty); set => SetValue(SectionKeyProperty, value); }
    public bool DefaultExpanded { get => GetValue(DefaultExpandedProperty); set => SetValue(DefaultExpandedProperty, value); }
    private WorkbenchLayout? layout;
    private bool restoring;
    public InspectorSection()
    {
        HorizontalAlignment = Avalonia.Layout.HorizontalAlignment.Stretch;
        DataContextChanged += (_, _) =>
        {
            if (layout is not null) layout.SectionsReset -= Restore;
            layout = (DataContext as EditorViewModel)?.Layout;
            if (layout is not null) layout.SectionsReset += Restore;
            Restore();
        };
        PropertyChanged += (_, e) =>
        {
            if (e.Property == SectionKeyProperty || e.Property == DefaultExpandedProperty) Restore();
            if (e.Property == IsExpandedProperty && !restoring) layout?.SetExpanded(SectionKey, IsExpanded);
        };
    }
    private void Restore()
    {
        restoring = true;
        IsExpanded = layout?.Expanded(SectionKey, DefaultExpanded) ?? DefaultExpanded;
        restoring = false;
    }
}
