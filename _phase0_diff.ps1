$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

# rootA = the zip the user just sent (already extracted)
$rootA = 'c:\SwarmLab\phase0_fault\swarmlab-master'

# rootB = discover the in-workspace 单群避障版 folder (has DroneState.m + max olfati lines),
#         EXCLUDING any phase0_* extraction dirs.
$candidates = Get-ChildItem -Path 'c:\SwarmLab' -Recurse -Directory -Filter 'swarmlab-master' -ErrorAction SilentlyContinue |
    Where-Object { $_.FullName -notmatch 'phase0' }
$best = $null; $bestLines = -1
foreach ($c in $candidates) {
    $olf = Join-Path $c.FullName '@Swarm\compute_vel_olfati_saber.m'
    $ds  = Join-Path $c.FullName 'enumerators\DroneState.m'
    if ((Test-Path -LiteralPath $olf) -and (Test-Path -LiteralPath $ds)) {
        $lc = (Get-Content -LiteralPath $olf | Measure-Object -Line).Lines
        if ($lc -gt $bestLines) { $bestLines = $lc; $best = $c.FullName }
    }
}
$rootB = $best
Write-Output "rootA (zip just sent) = $rootA"
Write-Output "rootB (workspace folder) ="
Write-Output $rootB
Write-Output ""

function Get-Map($root) {
    $map = @{}
    Get-ChildItem -Recurse -Path $root -Filter *.m | ForEach-Object {
        $rel = $_.FullName.Substring($root.Length + 1)
        $map[$rel] = (Get-FileHash -LiteralPath $_.FullName -Algorithm MD5).Hash
    }
    return $map
}

$ma = Get-Map $rootA
$mb = Get-Map $rootB

$diff = @(); $onlyA = @(); $onlyB = @(); $same = 0
foreach ($k in $ma.Keys) {
    if (-not $mb.ContainsKey($k)) { $onlyA += $k }
    elseif ($ma[$k] -ne $mb[$k]) { $diff += $k }
    else { $same++ }
}
foreach ($k in $mb.Keys) { if (-not $ma.ContainsKey($k)) { $onlyB += $k } }

Write-Output ("zip .m = {0}   |   folder .m = {1}" -f $ma.Count, $mb.Count)
Write-Output ("IDENTICAL = {0}   DIFFERENT = {1}   ONLY-IN-ZIP = {2}   ONLY-IN-FOLDER = {3}" -f $same, $diff.Count, $onlyA.Count, $onlyB.Count)
if ($diff.Count -gt 0)  { Write-Output "--- DIFFERENT ---";  $diff  | Sort-Object | ForEach-Object { "  [~] $_" } }
if ($onlyA.Count -gt 0) { Write-Output "--- ONLY IN ZIP ---"; $onlyA | Sort-Object | ForEach-Object { "  [A] $_" } }
if ($onlyB.Count -gt 0) { Write-Output "--- ONLY IN FOLDER ---"; $onlyB | Sort-Object | ForEach-Object { "  [B] $_" } }
