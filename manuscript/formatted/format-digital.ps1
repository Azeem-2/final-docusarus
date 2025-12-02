# Digital Formatting Script (EPUB, MOBI, HTML)
# Physical AI, Simulation AI & Humanoid Robotics Book

param(
    [string]$OutputDir = "manuscript/formatted/digital",
    [switch]$EPUB,
    [switch]$MOBI,
    [switch]$HTML
)

# Get script directory and set base path
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent (Split-Path -Parent $scriptDir)
Set-Location $repoRoot

Write-Host "Starting digital formatting..." -ForegroundColor Green
Write-Host "Working directory: $(Get-Location)" -ForegroundColor Gray

# Create output directory
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

# Collect all chapter files in order
$chapters = @(
    "manuscript/front-matter/title-page.md",
    "manuscript/front-matter/copyright.md",
    "manuscript/front-matter/preface.md",
    "manuscript/front-matter/how-to-use.md",
    "manuscript/toc.md",
    "manuscript/part1/introduction.md",
    "manuscript/part1/chapter1-what-is-physical-ai.md",
    "manuscript/part1/chapter2-robotics-vs-ai-vs-embodied-intelligence.md",
    "manuscript/part1/chapter3-evolution-of-humanoid-robotics.md",
    "manuscript/part1/chapter5-introduction-to-digital-twins.md",
    "manuscript/part2/chapter1-mechanical-structures.md",
    "manuscript/part2/chapter2-sensors-and-perception-hardware.md",
    "manuscript/part2/chapter3-actuators-and-motors.md",
    "manuscript/part2/chapter4-power-systems-and-batteries.md",
    "manuscript/part2/chapter5-kinematics.md",
    "manuscript/part2/chapter6-dynamics.md",
    "manuscript/part2/chapter7-control-systems.md",
    "manuscript/part3/chapter1-physics-engines.md",
    "manuscript/part3/chapter2-environment-modeling.md",
    "manuscript/part3/chapter3-reinforcement-learning-basics.md",
    "manuscript/part3/chapter4-imitation-learning.md",
    "manuscript/part3/chapter5-motion-planning-simulation.md",
    "manuscript/part3/chapter6-simulation-toolchains.md",
    "manuscript/part3/chapter7-sim-to-real-transfer.md",
    "manuscript/part4/chapter1-vision-models.md",
    "manuscript/part4/chapter2-multimodal-models.md",
    "manuscript/part4/chapter3-control-policies.md",
    "manuscript/part4/chapter4-reinforcement-learning-advanced.md",
    "manuscript/part4/chapter5-trajectory-optimization.md",
    "manuscript/part4/chapter6-policy-distillation.md",
    "manuscript/part4/chapter7-language-to-action.md",
    "manuscript/part5/chapter2-bipedal-locomotion.md",
    "manuscript/part5/chapter3-balance-stability.md",
    "manuscript/part5/chapter4-manipulation-dexterity.md",
    "manuscript/part5/chapter5-human-robot-interaction.md",
    "manuscript/part5/chapter6-safety-systems.md",
    "manuscript/part5/chapter7-case-studies.md",
    "manuscript/part6/chapter3-build-humanoid-leg.md",
    "manuscript/part6/chapter4-full-humanoid-digital-twin.md",
    "manuscript/part7/chapter1-industry-applications.md",
    "manuscript/appendices/glossary-by-category.md",
    "manuscript/appendices/bibliography.md",
    "manuscript/appendices/index.md",
    "manuscript/back-matter/about-authors.md"
)

# Create combined markdown file
$combinedFile = "$OutputDir/combined.md"
if (Test-Path $combinedFile) { Remove-Item $combinedFile }
$chapterCount = 0
$chapters | ForEach-Object {
    if (Test-Path $_) {
        $chapterCount++
        Write-Host "Adding: $_" -ForegroundColor Gray
        Add-Content -Path $combinedFile -Value "`n`n---`n`n"
        Get-Content $_ -Encoding UTF8 | Add-Content -Path $combinedFile -Encoding UTF8
    } else {
        Write-Host "Warning: File not found: $_" -ForegroundColor Yellow
    }
}
Write-Host "Combined $chapterCount chapters into $combinedFile" -ForegroundColor Green

# Generate EPUB
if ($EPUB) {
    Write-Host "Generating EPUB..." -ForegroundColor Yellow
    $epubFile = "$OutputDir/Physical-AI-Simulation-AI-Humanoid-Robotics.epub"
    
    $pandocArgs = @(
        "$combinedFile",
        "-o", "$epubFile",
        "--toc",
        "--toc-depth=3",
        "--metadata=title:Physical AI, Simulation AI & Humanoid Robotics",
        "--metadata=author:Book Development Team",
        "--metadata=date:2025",
        "--metadata=language:en"
    )
    
    # Remove cover image option if file doesn't exist
    if (-not (Test-Path "manuscript/front-matter/cover.png")) {
        Write-Host "Note: Cover image not found, skipping --epub-cover-image" -ForegroundColor Yellow
    } else {
        $pandocArgs += "--epub-cover-image=manuscript/front-matter/cover.png"
    }
    
    try {
        & pandoc @pandocArgs 2>&1 | Out-String
        if (Test-Path $epubFile) {
            Write-Host "EPUB generated: $epubFile" -ForegroundColor Green
        } else {
            Write-Host "EPUB generation may have failed. Check error messages above." -ForegroundColor Yellow
        }
    } catch {
        Write-Host "Error generating EPUB: $_" -ForegroundColor Red
    }
}

# Generate MOBI (requires KindleGen or Calibre)
if ($MOBI) {
    Write-Host "Generating MOBI..." -ForegroundColor Yellow
    Write-Host "Note: MOBI generation requires KindleGen or Calibre ebook-convert." -ForegroundColor Yellow
    Write-Host "After generating EPUB, convert using: ebook-convert input.epub output.mobi" -ForegroundColor Yellow
}

# Generate HTML
if ($HTML) {
    Write-Host "Generating HTML..." -ForegroundColor Yellow
    $htmlFile = "$OutputDir/index.html"
    
    $pandocArgs = @(
        "$combinedFile",
        "-o", "$htmlFile",
        "--standalone",
        "--toc",
        "--toc-depth=3",
        "--metadata=title:Physical AI, Simulation AI & Humanoid Robotics",
        "--metadata=author:Book Development Team"
    )
    
    # Add CSS if it exists
    if (Test-Path "$OutputDir/styles.css") {
        $pandocArgs += "--css=styles.css"
    }
    
    try {
        & pandoc @pandocArgs 2>&1 | Out-String
        if (Test-Path $htmlFile) {
            Write-Host "HTML generated: $htmlFile" -ForegroundColor Green
        } else {
            Write-Host "HTML generation may have failed. Check error messages above." -ForegroundColor Yellow
        }
    } catch {
        Write-Host "Error generating HTML: $_" -ForegroundColor Red
    }
}

Write-Host "Digital formatting complete!" -ForegroundColor Green
