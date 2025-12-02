# Print Formatting Script (PDF Generation via HTML)
# Alternative method: HTML to PDF conversion
# Physical AI, Simulation AI & Humanoid Robotics Book

param(
    [string]$OutputDir = "manuscript/formatted/print",
    [string]$OutputFile = "Physical-AI-Simulation-AI-Humanoid-Robotics.pdf"
)

Write-Host "Starting PDF generation via HTML conversion..." -ForegroundColor Green

# Create output directory
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

# Use the already-generated HTML file
$htmlFile = "manuscript/formatted/digital/index.html"

if (-not (Test-Path $htmlFile)) {
    Write-Host "Error: HTML file not found. Please run format-digital.ps1 -HTML first." -ForegroundColor Red
    exit 1
}

Write-Host "Using HTML file: $htmlFile" -ForegroundColor Yellow

# Try using wkhtmltopdf if available
$wkhtmltopdf = Get-Command wkhtmltopdf -ErrorAction SilentlyContinue

if ($wkhtmltopdf) {
    Write-Host "Using wkhtmltopdf for PDF generation..." -ForegroundColor Yellow
    $pdfFile = "$OutputDir/$OutputFile"
    
    & wkhtmltopdf --page-size A4 --margin-top 1in --margin-bottom 1in --margin-left 1in --margin-right 1in --encoding UTF-8 --enable-local-file-access "$htmlFile" "$pdfFile" 2>&1
    
    if (Test-Path $pdfFile) {
        Write-Host "PDF generated successfully: $pdfFile" -ForegroundColor Green
    } else {
        Write-Host "PDF generation failed." -ForegroundColor Red
    }
} else {
    Write-Host "wkhtmltopdf not found. Trying alternative methods..." -ForegroundColor Yellow
    
    # Alternative: Use Pandoc with HTML input
    Write-Host "Attempting PDF generation using Pandoc with HTML input..." -ForegroundColor Yellow
    
    $pandocArgs = @(
        "$htmlFile",
        "-o", "$OutputDir/$OutputFile",
        "--from=html",
        "--to=pdf",
        "--pdf-engine=wkhtmltopdf"
    )
    
    try {
        & pandoc @pandocArgs 2>&1 | Out-String
        if (Test-Path "$OutputDir/$OutputFile") {
            Write-Host "PDF generated successfully: $OutputDir/$OutputFile" -ForegroundColor Green
        } else {
            Write-Host "PDF generation failed. Install wkhtmltopdf or use browser print function." -ForegroundColor Yellow
            Write-Host "Alternative: Open $htmlFile in browser and use Print to PDF function." -ForegroundColor Yellow
        }
    } catch {
        Write-Host "Error: $_" -ForegroundColor Red
        Write-Host "`nManual PDF Generation Instructions:" -ForegroundColor Yellow
        Write-Host "1. Open: $htmlFile" -ForegroundColor White
        Write-Host "2. Press Ctrl+P (Print)" -ForegroundColor White
        Write-Host "3. Select 'Save as PDF' as printer" -ForegroundColor White
        Write-Host "4. Save to: $OutputDir/$OutputFile" -ForegroundColor White
    }
}

