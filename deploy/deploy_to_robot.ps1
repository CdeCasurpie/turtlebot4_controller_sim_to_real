# Copia el controlador autónomo desde esta PC (Windows) al TurtleBot4 vía SSH.
# Uso (desde la raíz del repo o desde cualquier lado):
#   .\deploy\deploy_to_robot.ps1 -RobotIp 192.168.0.102
#   .\deploy\deploy_to_robot.ps1 -RobotIp 192.168.0.102 -User ubuntu -Dest turtlebot4_controller
#
# Pide la contraseña SSH del robot (por defecto: turtlebot4).
# Solo copia lo necesario para correr a bordo (excluye Simulator, tests de pygame
# y la carpeta redundante yolonano/best/).

param(
    [Parameter(Mandatory = $true)][string]$RobotIp,
    [string]$User = "ubuntu",
    [string]$Dest = "turtlebot4_controller"
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot

# 1. Preparar carpeta staging con solo lo necesario
$staging = Join-Path $env:TEMP "tb4_deploy_staging"
if (Test-Path $staging) { Remove-Item -Recurse -Force $staging }
New-Item -ItemType Directory -Path $staging | Out-Null

Copy-Item (Join-Path $repo "run_real_autonomous.py") $staging
Copy-Item (Join-Path $repo "test_controller.py") $staging
Copy-Item (Join-Path $repo "test_vision.py") $staging
Copy-Item -Recurse (Join-Path $repo "TurtleBotController") (Join-Path $staging "TurtleBotController")
Copy-Item -Recurse (Join-Path $repo "deploy") (Join-Path $staging "deploy")

New-Item -ItemType Directory -Path (Join-Path $staging "yolonano") | Out-Null
Copy-Item (Join-Path $repo "yolonano\best.pt") (Join-Path $staging "yolonano\best.pt")
# Si ya exportaste el modelo NCNN en la PC, también se copia
$ncnn = Join-Path $repo "yolonano\best_ncnn_model"
if (Test-Path $ncnn) {
    Copy-Item -Recurse $ncnn (Join-Path $staging "yolonano\best_ncnn_model")
}
# Modelo para la VPU (blob + json de tools.luxonis.com), si existe
$vpu = Join-Path $repo "yolonano\vpu"
if (Test-Path $vpu) {
    Copy-Item -Recurse $vpu (Join-Path $staging "yolonano\vpu")
    Write-Host "Incluyendo modelo VPU (yolonano\vpu)" -ForegroundColor Cyan
}

# Limpiar __pycache__ del staging
Get-ChildItem -Recurse -Directory -Filter "__pycache__" $staging | Remove-Item -Recurse -Force

$files = (Get-ChildItem -Recurse -File $staging | Measure-Object).Count
Write-Host "Staging listo: $files archivos" -ForegroundColor Cyan

# 2. Copiar al robot (un solo scp -> una sola contraseña)
Write-Host "Copiando a ${User}@${RobotIp}:~/$Dest ..." -ForegroundColor Cyan
Write-Host "(contraseña por defecto del TurtleBot4: turtlebot4)"
scp -r "$staging" "${User}@${RobotIp}:~/$Dest.tmp"
if ($LASTEXITCODE -ne 0) { throw "scp falló (¿IP correcta? ¿robot encendido y en la misma red?)" }

# 3. Reemplazo atómico del directorio destino en el robot
ssh "${User}@${RobotIp}" "rm -rf ~/$Dest && mv ~/$Dest.tmp ~/$Dest && echo 'Despliegue OK en ~/$Dest'"
if ($LASTEXITCODE -ne 0) { throw "ssh falló al mover el directorio en el robot" }

Write-Host ""
Write-Host "Listo. Siguientes pasos en el robot:" -ForegroundColor Green
Write-Host "  ssh ${User}@${RobotIp}"
Write-Host "  cd ~/$Dest"
Write-Host "  bash deploy/install_on_robot.sh      # solo la primera vez"
Write-Host "  bash deploy/run_competition.sh       # correr la navegación autónoma"
