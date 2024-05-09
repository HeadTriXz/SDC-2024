function Create-Venv {

    param (
        [System.Management.Automation.CommandInfo]$python,
        [string] $version = "3.12"
    )

    if (-not $python) {
        Write-Host "Python not found. Please install Python $version or later" -BackgroundColor Red
        exit
    }

    Write-Host "No virtual environment found. Creating virtual environment in .venv folder" -BackgroundColor Yellow
    & $python -m venv .venv 2>&1
}


function Get-Venv {
    $venv = $null
    $venv = Get-Item -Path ".venv" -ErrorAction SilentlyContinue
    if (-not $venv) {
        $venv = Get-Item -Path "venv" -ErrorAction SilentlyContinue
    }

    if ($venv) {
        Write-Host "Using virtual environment at $venv" -ForegroundColor Green
    }

    return $venv
}

function Get-Python-Command {

    param (
        [Int16]$major = 3,
        [Int16]$minor = 12
    )

    $python_cmds = @('python', 'python3', 'py', 'py -3')
    $python = $null
    foreach ($cmd in $python_cmds) {
        $python = Get-Command -Name $cmd -ErrorAction SilentlyContinue
        if ($python) {
            $pythonVersion = & $python -V 2>&1

            $pattern = 'Python (\d+)\.(\d+)\.(\d+)'
            if ($pythonVersion -match $pattern) {
                $major = $Matches[1]
                $minor = $Matches[2]

                if ($major -lt 3 -or $minor -lt 12) {
                    $python = $null
                } else {
                    break
                }
            }
        }
    }

    return $python
}

# check if we are in root dir
if (-not (Test-Path -Path ".git")) {
    Write-Host "This script should be run from the root of the project" -BackgroundColor Red
    exit
}

# get the python command. if we aren't in a virtual environment, create one
$python = Get-Python-Command
if ($null -eq $python) {
    Write-Host "Python not found. Please install Python 3.12 or later" -BackgroundColor Red
    exit
}

if (-not (Test-Path env:VIRTUAL_ENV)) {
    Write-Host "Creating virtual environment using " -NoNewline
    $python_version = & $python -V 2>&1
    Write-Host $python_version -ForegroundColor Cyan
    Create-Venv($python)

    # activate the virtual environment
    $activation_script = Get-Venv | Join-Path -ChildPath "Scripts\Activate.ps1"
    & $activation_script

    # get the python command again. now in venv
    $python = Get-Python-Command
}


# install the dependencies
Write-Host "Installing development dependencies into " -NoNewline
Write-Host "$env:VIRTUAL_ENV" -ForegroundColor Cyan
pip install -r requirements/dev.txt | out-null

# install the pre-commit hooks
Write-Host "Installing pre-commit hooks into " -NoNewline
Write-Host ".git\hooks\pre-commit" -ForegroundColor Cyan
pre-commit install | out-null

Write-Host "Setup complete" -BackgroundColor Green
