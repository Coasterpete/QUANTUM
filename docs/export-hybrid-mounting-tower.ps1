# Read-only visual fixture: isolate the first tower of the straight generated
# capture run. Copy its nodes, split post segments, ledgers, braces and mounting
# intent verbatim. Full assembly documents remain in the perspective/bay views.
$mountingDirectory = Join-Path $PSScriptRoot '../build/hybrid-mounting'
$mountingSource = Join-Path $mountingDirectory 'towers.quantum'
$mountingDocument = Get-Content -LiteralPath $mountingSource -Raw | ConvertFrom-Json
$mountingStructure = $mountingDocument.supports.structures[0]
$mountingStructure.nodes = @($mountingStructure.nodes | Where-Object { $_.position.x -eq 0.0 })
$mountingNodeIds = @($mountingStructure.nodes | ForEach-Object { $_.id })
$mountingStructure.members = @($mountingStructure.members | Where-Object {
    $mountingNodeIds -contains $_.startNodeId -and $mountingNodeIds -contains $_.endNodeId
})
$mountingDocument | ConvertTo-Json -Depth 100 | Set-Content -LiteralPath (Join-Path $mountingDirectory 'isolated-tower.quantum')
