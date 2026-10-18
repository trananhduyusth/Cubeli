[CmdletBinding()]
param(
    [switch]$Watch
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$latexOutputDirectory = Join-Path $projectRoot 'build\latex'
$auxiliaryDirectory = Join-Path $latexOutputDirectory 'aux'

$documents = @(
    [pscustomobject]@{
        Source = Join-Path $projectRoot 'README.md'
        PdfDirectory = $projectRoot
    },
    [pscustomobject]@{
        Source = Join-Path $projectRoot 'Ref\STUDY.md'
        PdfDirectory = Join-Path $projectRoot 'Ref'
    }
)

if (-not (Get-Command pandoc -ErrorAction SilentlyContinue)) {
    $pandocDirectory = Join-Path $env:LOCALAPPDATA 'Pandoc'
    if (Test-Path (Join-Path $pandocDirectory 'pandoc.exe')) {
        $env:Path = "$pandocDirectory;$env:Path"
    }
}

if (-not (Get-Command xelatex -ErrorAction SilentlyContinue)) {
    $miktexDirectory = Join-Path $env:LOCALAPPDATA 'Programs\MiKTeX\miktex\bin\x64'
    if (Test-Path (Join-Path $miktexDirectory 'xelatex.exe')) {
        $env:Path = "$miktexDirectory;$env:Path"
    }
}

foreach ($tool in @('pandoc', 'xelatex')) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "$tool was not found. Install Pandoc and MiKTeX, then run this script again."
    }
}

foreach ($document in $documents) {
    if (-not (Test-Path $document.Source)) {
        throw "Markdown source not found: $($document.Source)"
    }
}

function Build-MarkdownPdf {
    param(
        [Parameter(Mandatory)]
        [pscustomobject]$Document
    )

    $name = [IO.Path]::GetFileNameWithoutExtension($Document.Source)
    $texPath = Join-Path $latexOutputDirectory "$name.tex"
    $documentAuxDirectory = Join-Path $auxiliaryDirectory $name
    $generatedPdfPath = Join-Path $documentAuxDirectory "$name.pdf"
    $pdfPath = Join-Path $Document.PdfDirectory "$name.pdf"

    New-Item -ItemType Directory -Force -Path $documentAuxDirectory | Out-Null

    & pandoc --from markdown --to latex --standalone --output $texPath $Document.Source
    if ($LASTEXITCODE -ne 0) {
        throw "Pandoc failed to convert $($Document.Source)."
    }

    Push-Location $Document.PdfDirectory
    try {
        & xelatex --enable-installer --interaction=nonstopmode --halt-on-error --file-line-error "--output-directory=$documentAuxDirectory" $texPath
        if ($LASTEXITCODE -ne 0) {
            throw "XeLaTeX failed to compile $texPath."
        }
    }
    finally {
        Pop-Location
    }

    Copy-Item -Force $generatedPdfPath $pdfPath
    Write-Host "Built $pdfPath"
}

foreach ($document in $documents) {
    Build-MarkdownPdf -Document $document
}

if ($Watch) {
    $lastWriteTimes = @{}
    foreach ($document in $documents) {
        $lastWriteTimes[$document.Source] = (Get-Item $document.Source).LastWriteTimeUtc
    }

    Write-Host 'Watching Markdown sources for changes. Press Ctrl+C to stop.'
    while ($true) {
        Start-Sleep -Milliseconds 400
        foreach ($document in $documents) {
            $lastWriteTime = (Get-Item $document.Source).LastWriteTimeUtc
            if ($lastWriteTime -ne $lastWriteTimes[$document.Source]) {
                Start-Sleep -Milliseconds 250
                $lastWriteTimes[$document.Source] = (Get-Item $document.Source).LastWriteTimeUtc
                try {
                    Build-MarkdownPdf -Document $document
                }
                catch {
                    Write-Host "Build failed: $($_.Exception.Message)" -ForegroundColor Red
                }
            }
        }
    }
}
