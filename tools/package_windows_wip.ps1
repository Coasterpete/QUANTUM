param(
    [ValidateSet("Release", "RelWithDebInfo")]
    [string]$Configuration = "RelWithDebInfo",

    [string]$BuildDirectory = "build",

    [string]$OutputDirectory = "dist",

    [switch]$RunTests,

    [switch]$KeepStaging
)

$ErrorActionPreference = "Stop"

# ------------------------------------------------------------
# Resolve repository root
# ------------------------------------------------------------

$ScriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDirectory "..")

Set-Location $RepoRoot

$BuildPath = Join-Path $RepoRoot $BuildDirectory
$DistPath = Join-Path $RepoRoot $OutputDirectory

$PackageName = "QUANTUM-WIP-Windows-x64"
$StagePath = Join-Path $DistPath $PackageName
$ZipPath = Join-Path $DistPath "$PackageName.zip"

Write-Host ""
Write-Host "========================================"
Write-Host " QUANTUM Windows WIP Packager"
Write-Host "========================================"
Write-Host ""
Write-Host "Repository:    $RepoRoot"
Write-Host "Configuration: $Configuration"
Write-Host "Build:         $BuildPath"
Write-Host "Stage:         $StagePath"
Write-Host "ZIP:           $ZipPath"
Write-Host ""

# ------------------------------------------------------------
# Verify required tools
# ------------------------------------------------------------

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    throw "CMake was not found in PATH."
}

# ------------------------------------------------------------
# Build QUANTUM
# ------------------------------------------------------------

Write-Host "Building QUANTUM ($Configuration)..."

cmake --build $BuildPath --config $Configuration

if ($LASTEXITCODE -ne 0) {
    throw "QUANTUM build failed."
}

# ------------------------------------------------------------
# Optional tests
# ------------------------------------------------------------

if ($RunTests) {
    Write-Host ""
    Write-Host "Running CTest..."

    ctest `
        --test-dir $BuildPath `
        -C $Configuration `
        --output-on-failure

    if ($LASTEXITCODE -ne 0) {
        throw "CTest failed. Package will not be created."
    }
}

# ------------------------------------------------------------
# Clean staging directory
# ------------------------------------------------------------

Write-Host ""
Write-Host "Preparing staging directory..."

if (Test-Path $StagePath) {
    Remove-Item $StagePath -Recurse -Force
}

New-Item `
    -ItemType Directory `
    -Path $StagePath `
    -Force | Out-Null

# ------------------------------------------------------------
# Install portable runtime files
# ------------------------------------------------------------

Write-Host "Installing portable QUANTUM package..."

cmake `
    --install $BuildPath `
    --config $Configuration `
    --prefix $StagePath

if ($LASTEXITCODE -ne 0) {
    throw "CMake install failed."
}

# ------------------------------------------------------------
# Verify executable exists
# ------------------------------------------------------------

$QuantumExe = Get-ChildItem `
    -Path $StagePath `
    -Filter "QUANTUM.exe" `
    -File `
    -Recurse |
    Select-Object -First 1

if (-not $QuantumExe) {
    throw @"
QUANTUM.exe was not found in the staged package.

The CMake install rules probably do not install the QUANTUM executable yet.
"@
}

Write-Host ""
Write-Host "Found executable:"
Write-Host "  $($QuantumExe.FullName)"

# ------------------------------------------------------------
# Protect against accidentally packaging developer junk
# ------------------------------------------------------------

$ForbiddenExtensions = @(
    ".cpp",
    ".c",
    ".hpp",
    ".h",
    ".obj",
    ".pdb"
)

$ForbiddenFiles = Get-ChildItem `
    -Path $StagePath `
    -File `
    -Recurse |
    Where-Object {
        $ForbiddenExtensions -contains $_.Extension.ToLowerInvariant()
    }

if ($ForbiddenFiles) {
    Write-Warning "Developer files were found in the staged package:"

    $ForbiddenFiles |
        ForEach-Object {
            Write-Warning "  $($_.FullName)"
        }

    throw "Package contains files that should not normally be distributed."
}

# ------------------------------------------------------------
# Search packaged text/config files for developer machine paths
# ------------------------------------------------------------

Write-Host ""
Write-Host "Checking for obvious development-path leakage..."

$TextExtensions = @(
    ".txt",
    ".json",
    ".ini",
    ".cfg",
    ".xml"
)

$PotentialTextFiles = Get-ChildItem `
    -Path $StagePath `
    -File `
    -Recurse |
    Where-Object {
        $TextExtensions -contains $_.Extension.ToLowerInvariant()
    }

$PathLeaks = @()

foreach ($File in $PotentialTextFiles) {
    try {
        $Matches = Select-String `
            -Path $File.FullName `
            -Pattern "C:\\DEV1\\QUANTUM" `
            -SimpleMatch

        if ($Matches) {
            $PathLeaks += $File.FullName
        }
    }
    catch {
        # Ignore unreadable/non-text files.
    }
}

if ($PathLeaks.Count -gt 0) {
    Write-Warning "Possible developer-path references found:"

    $PathLeaks |
        Sort-Object -Unique |
        ForEach-Object {
            Write-Warning "  $_"
        }

    throw "Development-path leakage detected."
}

Write-Host "No obvious C:\DEV1\QUANTUM references found."

# ------------------------------------------------------------
# Remove previous ZIP
# ------------------------------------------------------------

if (Test-Path $ZipPath) {
    Remove-Item $ZipPath -Force
}

# ------------------------------------------------------------
# Create ZIP with one top-level QUANTUM folder
# ------------------------------------------------------------

Write-Host ""
Write-Host "Creating ZIP..."

Compress-Archive `
    -Path $StagePath `
    -DestinationPath $ZipPath `
    -CompressionLevel Optimal

# ------------------------------------------------------------
# Report package size
# ------------------------------------------------------------

$Zip = Get-Item $ZipPath
$SizeMB = [Math]::Round($Zip.Length / 1MB, 2)

Write-Host ""
Write-Host "========================================"
Write-Host " Package complete"
Write-Host "========================================"
Write-Host ""
Write-Host "ZIP:"
Write-Host "  $ZipPath"
Write-Host ""
Write-Host "Size:"
Write-Host "  $SizeMB MB"
Write-Host ""
Write-Host "Portable executable:"
Write-Host "  $($QuantumExe.FullName)"
Write-Host ""
Write-Host "IMPORTANT:"
Write-Host "Test the extracted ZIP from outside the repository"
Write-Host "before sending it to another person."
Write-Host ""

# ------------------------------------------------------------
# Optionally remove staging directory
# ------------------------------------------------------------

if (-not $KeepStaging) {
    Write-Host "Removing temporary staging directory..."
    Remove-Item $StagePath -Recurse -Force
}
else {
    Write-Host "Keeping staging directory:"
    Write-Host "  $StagePath"
}

Write-Host ""
Write-Host "Done."