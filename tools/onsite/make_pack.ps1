# make_pack.ps1 —— 把副本要覆盖到车机的文件打成一个 tar.gz（笔记本上跑）
#
# 用法（Windows PowerShell 5.1 / 7 都能跑）：
#   powershell -ExecutionPolicy Bypass -File tools\onsite\make_pack.ps1
#   powershell -ExecutionPolicy Bypass -File tools\onsite\make_pack.ps1 -OutDir D:\DSH -Proj D:\MTG\Documents\OC\fp_aim_fkey
#
# 它做四件事：
#   1) 读 tools\onsite\pack_manifest.txt（**文件清单的唯一来源**）里的清单；
#   2) 把这些文件按工程根的相对路径暂存，算 sha256；
#   3) 生成 pack/manifest.sha256（给车机校验）+ pack/manifest.txt（人看的清单）
#      + 把 tools\onsite\pack\install.sh 一起打进包；
#   4) 打 tar.gz，并打印**传送命令**和**车机安装命令**。
#
# 好处：车机那边只有"解压 + 跑一个脚本"两步，不再一长串 scp，也不再数文件。

param(
  [string]$Proj    = "D:\MTG\Documents\OC\fp_aim_fkey",
  [string]$OutDir  = "D:\DSH",
  [string]$CarHost = "rm@192.168.1.50"
)
$ErrorActionPreference = "Stop"

$manifest = Join-Path $Proj "tools\onsite\pack_manifest.txt"
if (-not (Test-Path $manifest)) { throw "找不到清单：$manifest" }
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }

$stage = Join-Path $env:TEMP "fp_aim_pack_stage"
if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
New-Item -ItemType Directory -Path $stage | Out-Null

# ---- 1) 读清单并暂存 ----
$items = @()
foreach ($raw in (Get-Content -LiteralPath $manifest -Encoding UTF8)) {
  $line = $raw.Trim()
  if (-not $line) { continue }
  if ($line.StartsWith("#")) { continue }
  $parts = $line -split "\s{2,}", 2
  if ($parts.Count -lt 2) { $parts = $line -split "\s+", 2 }   # 只有一个空格时也认（路径不带空格，安全）
  $rel = $parts[0].Trim()
  $why = if ($parts.Count -gt 1) { $parts[1].Trim() } else { "" }
  $src = Join-Path $Proj ($rel -replace '/', '\')
  if (-not (Test-Path -LiteralPath $src)) { throw "清单里的文件不存在：$rel （在 $Proj 下）" }
  $dst = Join-Path $stage ($rel -replace '/', '\')
  $ddir = Split-Path $dst -Parent
  if (-not (Test-Path $ddir)) { New-Item -ItemType Directory -Path $ddir -Force | Out-Null }
  Copy-Item -LiteralPath $src -Destination $dst -Force
  $items += [pscustomobject]@{
    rel = $rel
    why = $why
    sha = (Get-FileHash -LiteralPath $src -Algorithm SHA256).Hash.ToLower()
  }
}
Write-Host "清单里的文件：$($items.Count) 个"

# ---- 2) 生成包内的清单 ----
$packDir = Join-Path $stage "tools\onsite\pack"
if (-not (Test-Path $packDir)) { New-Item -ItemType Directory -Path $packDir -Force | Out-Null }
$shaLines = $items | ForEach-Object { "$($_.sha)  $($_.rel)" }
[IO.File]::WriteAllText((Join-Path $packDir "manifest.sha256"), (($shaLines -join "`n") + "`n"), (New-Object Text.UTF8Encoding($false)))
$txt = @(
  "# fp_aim 副本补丁包清单",
  "# 生成于 $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')   共 $($items.Count) 个文件",
  "# 格式：<相对工程根的路径>  <为什么需要它>",
  ""
) + ($items | ForEach-Object { "$($_.rel)  $($_.why)" })
[IO.File]::WriteAllText((Join-Path $packDir "manifest.txt"), (($txt -join "`n") + "`n"), (New-Object Text.UTF8Encoding($false)))
Copy-Item -LiteralPath (Join-Path $Proj "tools\onsite\pack\install.sh") -Destination (Join-Path $packDir "install.sh") -Force

# ---- 3) 打 tar.gz ----
$stamp = Get-Date -Format "MMdd_HHmm"
$tarName = "fp_aim_patch_$stamp.tar.gz"
$outTar = Join-Path $OutDir $tarName
if (Test-Path $outTar) { Remove-Item $outTar -Force }
Push-Location $stage
& tar -czf $outTar .
$rc = $LASTEXITCODE
Pop-Location
if ($rc -ne 0) { throw "tar 打包失败（退出码 $rc）" }

$tarSha = (Get-FileHash -LiteralPath $outTar -Algorithm SHA256).Hash.ToLower()
$tarKb  = [math]::Round((Get-Item $outTar).Length / 1KB, 1)

Write-Host ""
Write-Host "=============================================================="
Write-Host " 补丁包已生成"
Write-Host "=============================================================="
Write-Host "  文件 : $outTar  ($tarKb KB)"
Write-Host "  sha256: $tarSha"
Write-Host ""
Write-Host " ① 传到车机（笔记本 PowerShell，笔记本有线 IP 192.168.1.43）"
Write-Host "    scp $outTar ${CarHost}:~/"
Write-Host "    （网线不通就用 U 盘：把这个 tar.gz 拷到 U 盘，车机上再 cp 过去）"
Write-Host ""
Write-Host " ② 车机上安装（一条命令）"
Write-Host "    cd ~/fp_aim && tar -xzf ~/$tarName && bash tools/onsite/pack/install.sh"
Write-Host ""
Write-Host " ③ 之后"
Write-Host "    ./build/keytest            # 按键通道"
Write-Host "    ~/run_measure.sh preflight # 预检"
Write-Host "=============================================================="
