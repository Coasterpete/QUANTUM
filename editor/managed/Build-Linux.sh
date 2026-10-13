#!/usr/bin/env bash
set -euo pipefail
repository="$(cd "$(dirname "$0")/../.." && pwd)"
native_build="${1:-$repository/build/linux-clang-full-debug}"
configuration="${2:-Debug}"
dotnet_cli="${DOTNET:-dotnet}"
artifacts="$native_build/managed"
# Configure native targets using the existing CMake/vcpkg presets first.
cmake --build "$native_build" --target QuantumNative QuantumNativeTests QuantumApplicationServiceTests -j "${JOBS:-8}"
for project in Quantum.Editor Quantum.Managed.Tests; do
    "$dotnet_cli" build "$repository/editor/managed/$project/$project.csproj" \
        -c "$configuration" --artifacts-path "$artifacts" --nologo
    destination="$artifacts/bin/$project/${configuration,,}"
    cp "$native_build/editor/native/libQuantumNative.so" "$destination/"
    cp -a "$native_build/editor/native/shaders" "$native_build/editor/native/assets" "$destination/"
done
printf 'Launch the apphost: %s\n' "$artifacts/bin/Quantum.Editor/${configuration,,}/Quantum.Editor"
