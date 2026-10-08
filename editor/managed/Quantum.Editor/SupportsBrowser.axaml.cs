using Avalonia.Controls;

namespace Quantum.Editor;
public sealed partial class SupportsBrowser : UserControl
{
    public SupportsBrowser() => InitializeComponent();
    public void FocusStructures() => Structures.Focus();
}
