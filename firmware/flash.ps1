<#
  flash.ps1 — flash USB DFU "one-shot" via mcumgr-client.

  Enchaîne automatiquement :
    1. upload de l'image dans le slot secondaire (slot 1)
    2. list  → récupère tout seul le hash de l'image du slot 1
    3. test <hash>  → marque l'image pour le prochain boot
    4. reset → MCUboot swappe et boote la nouvelle image
  (Le firmware s'auto-confirme au boot via boot_write_img_confirmed → pas de revert.)

  Prérequis : avoir buildé avant (ex. `west build -d build_2`).

  Exemples :
    .\flash.ps1
    .\flash.ps1 -Port COM10
    .\flash.ps1 -Port COM7 -File build_2\firmware\zephyr\zephyr.signed.bin
#>
param(
    [string]$Port          = "COM10",
    [string]$File          = "build_2\firmware\zephyr\zephyr.signed.bin",
    [string]$ExpectedBuild = "20260825-directinit-v8",
    [int]   $Timeout       = 2000
)

$ErrorActionPreference = "Stop"
$mcu    = ".\mcumgr-client.exe"
$common = @("-d", $Port, "-u", "$Timeout")

if (-not (Test-Path -LiteralPath $File)) {
    throw "Image introuvable : $File (as-tu buildé ?)"
}

# Barrière anti-régression : l'ancien build pwr-v4 coupe OSC1/P0.08 en fin de
# mesure et fige CPU/USB. Refuser aussi intc-v3, puis exiger explicitement
# l'identifiant attendu avant la moindre écriture DFU sur la carte.
$resolvedFile = (Resolve-Path -LiteralPath $File).Path
$imageText = [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($resolvedFile))
$knownBadBuilds = @("20260824-intc-v3", "20260824-pwr-v4")
$badBuild = $knownBadBuilds | Where-Object { $imageText.Contains($_) } | Select-Object -First 1
if ($badBuild) {
    throw "FLASH REFUSÉ : l'image contient le firmware défectueux $badBuild. Refaire un build pristine directinit-v8."
}
if ([string]::IsNullOrWhiteSpace($ExpectedBuild) -or -not $imageText.Contains($ExpectedBuild)) {
    throw "FLASH REFUSÉ : l'image ne contient pas l'identifiant attendu '$ExpectedBuild'. Refaire le build ou fournir -ExpectedBuild avec l'identifiant vérifié."
}

Write-Host "Image vérifiée : $ExpectedBuild" -ForegroundColor DarkGray

Write-Host "1/4  upload $File" -ForegroundColor Cyan
& $mcu @common upload $File
if ($LASTEXITCODE -ne 0) { throw "upload a echoue (code $LASTEXITCODE)" }

Write-Host "2/4  list (recuperation du hash slot 1)" -ForegroundColor Cyan
$raw   = (& $mcu @common list | Out-String)
$start = $raw.IndexOf('{')
if ($start -lt 0) { throw "Reponse 'list' inattendue :`n$raw" }
$info  = $raw.Substring($start) | ConvertFrom-Json
$img   = $info.images | Where-Object { $_.slot -eq 1 } | Select-Object -First 1
if (-not $img) { throw "Aucune image en slot 1 (upload rate ?)" }
$hash  = $img.hash
$active = $info.images | Where-Object { $_.slot -eq 0 -and $_.active } | Select-Object -First 1
Write-Host "      slot 1 hash = $hash" -ForegroundColor DarkGray

# Une image deja active peut etre uploadee une seconde fois dans le slot 1.
# MCUboot refuse alors `test` (rc=1), car il n'existe aucun changement a
# programmer. C'est un succes idempotent, pas un echec de flash.
if ($active -and ([string]$active.hash -eq [string]$hash)) {
    Write-Host "OK - cette image est deja active (hash identique au slot 0). Aucun swap necessaire." -ForegroundColor Green
    return
}

Write-Host "3/4  test $hash" -ForegroundColor Cyan
& $mcu @common test $hash
if ($LASTEXITCODE -ne 0) { throw "test a echoue (code $LASTEXITCODE)" }

Write-Host "4/4  reset" -ForegroundColor Cyan
& $mcu @common reset

Write-Host "OK - la carte boote la nouvelle image (auto-confirmee, pas de revert)." -ForegroundColor Green
