Param(
  [string]$TaskFile = "codex\tasks\flywheel_stability.md"
)

function Test-Command($cmd) {
  $old = $ErrorActionPreference
  $ErrorActionPreference = 'SilentlyContinue'
  $null = Get-Command $cmd
  $ok = $? 
  $ErrorActionPreference = $old
  return $ok
}

if (-not (Test-Command "codex")) {
  Write-Host "Codex CLI not found. Please install and authenticate it first." 
  Write-Host "Example: npm i -g @openai/codex  (or follow the official docs)"
  exit 1
}

if (-not (Test-Path $TaskFile)) {
  Write-Host "Task file not found: $TaskFile"
  exit 1
}

Write-Host "Running Codex with task: $TaskFile"
& codex run --task-file $TaskFile
