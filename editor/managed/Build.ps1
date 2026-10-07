param(
    [ValidateSet('Debug', 'Release')][string]$Configuration = 'Debug',
    [switch]$UiOnly,
    [switch]$Run,
    [switch]$Test
)
$ErrorActionPreference = 'Stop'
$repository = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$app = Join-Path $PSScriptRoot 'Quantum.Editor/Quantum.Editor.csproj'
if (-not $UiOnly) {
    if (-not $env:VCPKG_ROOT) { throw 'Set VCPKG_ROOT to your existing vcpkg checkout.' }
    & cmake --preset windows-msvc-debug -S $repository -DQUANTUM_ENABLE_HYBRID_EDITOR=ON
    if ($LASTEXITCODE) { throw 'Native configure failed.' }
    & cmake --build (Join-Path $repository 'build') --config $Configuration --target QuantumNative QUANTUM --parallel
    if ($LASTEXITCODE) { throw 'Native build failed.' }
}
& dotnet build $app -c $Configuration --nologo
if ($LASTEXITCODE) { throw 'Managed build failed.' }
$native = Join-Path $repository "build/editor/native/$Configuration"
$output = Join-Path $PSScriptRoot "Quantum.Editor/bin/$Configuration/net10.0"
Copy-Item -LiteralPath (Join-Path $native 'QuantumNative.dll') -Destination $output
# QUANTUM links the existing static vcpkg Vulkan loader on Windows.
$loader = Join-Path $native 'vulkan-1.dll'
if (Test-Path -LiteralPath $loader) { Copy-Item -LiteralPath $loader -Destination $output }
foreach ($directory in @('shaders', 'assets')) {
    Copy-Item -LiteralPath (Join-Path $native $directory) -Destination $output -Recurse -Force
}
if ($Test) {
    & dotnet build (Join-Path $PSScriptRoot 'Quantum.Managed.Tests/Quantum.Managed.Tests.csproj') -c $Configuration --nologo
    if ($LASTEXITCODE) { throw 'Managed test build failed.' }
    $testOutput = Join-Path $PSScriptRoot "Quantum.Managed.Tests/bin/$Configuration/net10.0"
    Copy-Item -LiteralPath (Join-Path $native 'QuantumNative.dll') -Destination $testOutput
    & (Join-Path $testOutput 'Quantum.Managed.Tests.exe') (Join-Path $repository 'smoke-tests/modern-steel-validation.quantum')
    if ($LASTEXITCODE) { throw 'Managed tests failed.' }
}
if ($Run) { Start-Process -FilePath (Join-Path $output 'Quantum.Editor.exe') -WorkingDirectory $output -WindowStyle Hidden }
