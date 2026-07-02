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
    [string]$Port    = "COM10",
    [string]$File    = "build_2\firmware\zephyr\zephyr.signed.bin",
    [int]   $Timeout = 2000
)

$ErrorActionPreference = "Stop"
$mcu    = ".\mcumgr-client.exe"
$common = @("-d", $Port, "-u", "$Timeout")

if (-not (Test-Path $File)) { throw "Image introuvable : $File (as-tu buildé ?)" }

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
Write-Host "      slot 1 hash = $hash" -ForegroundColor DarkGray

Write-Host "3/4  test $hash" -ForegroundColor Cyan
& $mcu @common test $hash
if ($LASTEXITCODE -ne 0) { throw "test a echoue (code $LASTEXITCODE)" }

Write-Host "4/4  reset" -ForegroundColor Cyan
& $mcu @common reset

Write-Host "OK - la carte boote la nouvelle image (auto-confirmee, pas de revert)." -ForegroundColor Green
