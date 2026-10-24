<#
.SYNOPSIS
    从车上把 collect_run.sh 打好的包拉回笔记本：拷贝 -> 校验 sha256 -> 可选解包。

.DESCRIPTION
    依赖 Windows 自带的 scp.exe / tar.exe（Win10 1803+ 都有）。
    传输完会用包里的 .sha256 校验，校验不过不算成功。

.EXAMPLE
    pwsh -File tools\onsite\pull_run.ps1 -Name low3rad

.EXAMPLE
    pwsh -File tools\onsite\pull_run.ps1 -Name low3rad -Extract

.EXAMPLE
    # 车机地址不是默认值时
    pwsh -File tools\onsite\pull_run.ps1 -Name low3rad -Car rm@192.168.1.51 -Extract
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Name,

    [string]$Car = "rm@192.168.1.50",
    [string]$RemoteDir = "/home/rm/packs",
    [string]$LocalDir = "D:\DSH\onsite_runs",
    [switch]$Extract,
    [switch]$Force
)

$ErrorActionPreference = "Stop"

function Say($msg) { Write-Host "== $msg ==" -ForegroundColor Cyan }
function Ok($msg) { Write-Host "   $msg" -ForegroundColor Green }
function Warn($msg) { Write-Host "   $msg" -ForegroundColor Yellow }
function Die($msg) { Write-Host "!! $msg" -ForegroundColor Red; exit 1 }

foreach ($tool in @("scp", "ssh", "tar")) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) { Die "找不到 $tool，先装 OpenSSH 客户端" }
}

New-Item -ItemType Directory -Force -Path $LocalDir | Out-Null
$SshOpts = @("-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=8")

# ---------------------------------------------------------------------------
Say "1/4 看车上有什么"
$remoteList = & ssh @SshOpts $Car "ls -1 $RemoteDir/$Name* 2>/dev/null" 2>$null
if ($LASTEXITCODE -ne 0 -or -not $remoteList) {
    Die "车上找不到 $RemoteDir/$Name*（先跑 collect_run.sh，或者 -RemoteDir 指错）"
}
$remoteFiles = @($remoteList | Where-Object { $_ -and $_.Trim() })
$remoteFiles | ForEach-Object { Ok (Split-Path $_ -Leaf) }

# ---------------------------------------------------------------------------
Say "2/4 拷回本地 $LocalDir"
foreach ($rf in $remoteFiles) {
    $target = Join-Path $LocalDir (Split-Path $rf -Leaf)
    if ((Test-Path $target) -and -not $Force) {
        $same = (Get-Item $target).Length
        Warn "已存在 $(Split-Path $rf -Leaf)（$([math]::Round($same/1MB,1)) MB），加 -Force 才覆盖"
        continue
    }
    & scp @SshOpts "$Car`:$rf" $target
    if ($LASTEXITCODE -ne 0) { Die "scp 失败：$rf" }
    Ok ("{0}  {1} MB" -f (Split-Path $rf -Leaf), [math]::Round((Get-Item $target).Length / 1MB, 1))
}

# ---------------------------------------------------------------------------
Say "3/4 校验 sha256"
$shaPath = Join-Path $LocalDir "$Name.sha256"
$allGood = $true
if (-not (Test-Path $shaPath)) {
    Warn "没有 $Name.sha256，跳过校验（不推荐：传输损坏很常见）"
} else {
    foreach ($line in Get-Content $shaPath) {
        if ($line -match '^\s*([0-9a-fA-F]{64})\s+\*?(.+?)\s*$') {
            $want = $Matches[1].ToLower()
            $leaf = Split-Path $Matches[2].Trim() -Leaf
            $path = Join-Path $LocalDir $leaf
            if (-not (Test-Path $path)) { Warn "$leaf 不在本地，跳过"; continue }
            $got = (Get-FileHash -Algorithm SHA256 -Path $path).Hash.ToLower()
            if ($got -eq $want) { Ok "$leaf 校验通过" }
            else { $allGood = $false; Write-Host "!! $leaf 校验失败" -ForegroundColor Red }
        }
    }
}
if (-not $allGood) { Die "校验没通过，别拿这份数据下结论。重拷一次。" }

# ---------------------------------------------------------------------------
Say "4/4 拼分卷（如果是分卷）"
$parts = @(Get-ChildItem -Path (Join-Path $LocalDir "$Name.tar.part*") -ErrorAction SilentlyContinue | Sort-Object Name)
$fullTar = Join-Path $LocalDir "$Name.tar"
if ($parts.Count -gt 0) {
    if (Test-Path $fullTar) { Remove-Item $fullTar -Force }
    $out = [System.IO.File]::Create($fullTar)
    try {
        foreach ($p in $parts) {
            $in = [System.IO.File]::OpenRead($p.FullName)
            try { $in.CopyTo($out) } finally { $in.Dispose() }
        }
    } finally { $out.Dispose() }
    Ok ("拼成 {0}  {1} MB" -f (Split-Path $fullTar -Leaf), [math]::Round((Get-Item $fullTar).Length / 1MB, 1))
}

$pack = $null
foreach ($cand in @($fullTar, (Join-Path $LocalDir "$Name.tar.gz"))) {
    if (Test-Path $cand) { $pack = $cand; break }
}
if (-not $pack) { Die "本地没找到 $Name.tar / $Name.tar.gz" }

if ($Extract) {
    $dest = Join-Path $LocalDir $Name
    if (Test-Path $dest) {
        if ($Force) { Remove-Item $dest -Recurse -Force } else { Die "$dest 已存在，加 -Force 覆盖" }
    }
    New-Item -ItemType Directory -Force -Path $dest | Out-Null
    & tar -xf $pack -C $dest
    if ($LASTEXITCODE -ne 0) { Die "解包失败" }
    Ok "解包到 $dest"
    Write-Host ""
    Write-Host "接下来（在这个目录下）：" -ForegroundColor Cyan
    Write-Host "  py tools\analyze_frames.py `"$dest\recordings`""
    Write-Host "  py tools\plot_frames.py    `"$dest\recordings`" --out-dir `"$dest\plots`""
    Write-Host "  py tools\latency_report.py `"$dest`"\logs\*.csv"
} else {
    Write-Host ""
    Write-Host "包在 $pack，加 -Extract 可以直接解包" -ForegroundColor Cyan
}
