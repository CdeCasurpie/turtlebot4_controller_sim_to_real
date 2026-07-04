# Registro: conversión de `yolonanov2` a `.blob` (ya ejecutado)

Este documento es un registro de cómo se generaron los artefactos que ya están en el repo.
Solo hace falta repetir esto si `turtlebot_signals_v2_best.pt` se reentrena. Para desplegar
en el robot con lo que ya existe, ve directo a `DEPLOYMENT_GUIDE.md`.

## Artefactos generados

| Archivo | Contenido |
|---|---|
| `yolonanov2/turtlebot_signals_v2_best.onnx` | ONNX re-exportado con opset 12 |
| `vpu_deployment/models/turtlebot_signals_v2.blob` | Blob FP16, 6 shaves, target Myriad X (OpenVINO 2022.1) |

## Paso 1: `.pt` → ONNX (opset 12)

El `.onnx` original (`yolonanov2/turtlebot_signals_v2.onnx`) traía opset 20, demasiado nuevo
para el compilador clásico de Myriad X. Se re-exportó desde el checkpoint de PyTorch:

```bash
pip install ultralytics
python -c "
from ultralytics import YOLO
m = YOLO('yolonanov2/turtlebot_signals_v2_best.pt')
m.export(format='onnx', opset=12, imgsz=640, simplify=True)
"
```

- `opset=12`: máximo que el toolchain de Myriad X soporta de forma confiable.
- `imgsz=640`: debe coincidir con `INPUT_SIZE` en `test_depthai_yolo.py`.
- `simplify=True`: corre `onnxslim` para reducir el riesgo de operadores no soportados.

Verificación (input `[1,3,640,640]`, output `[1,8,8400]` = 4 clases + 4 coords sin
objectness — confirma que es *anchor-free*, por eso el pipeline usa `anchors=[]`):

```bash
pip install onnx
python -c "
import onnx
m = onnx.load('yolonanov2/turtlebot_signals_v2_best.onnx')
print('input:', [d.dim_value for d in m.graph.input[0].type.tensor_type.shape.dim])
print('output:', [d.dim_value for d in m.graph.output[0].type.tensor_type.shape.dim])
print('opset:', m.opset_import[0].version)
"
```

## Paso 2: ONNX → `.blob` (FP16, 6 shaves)

Intel eliminó el plugin Myriad de OpenVINO local desde la 2023.0 (última versión con
soporte: 2022.3, pesada e inestable de instalar en Windows). Se usó el servicio cloud
oficial de Luxonis (`blobconverter`) en su lugar:

```bash
pip install blobconverter
python -c "
import blobconverter
path = blobconverter.from_onnx(
    model='yolonanov2/turtlebot_signals_v2_best.onnx',
    data_type='FP16',
    shaves=6,
    use_cache=False,
    compile_params=['-ip U8'],
    output_dir='vpu_deployment/models',
)
print('BLOB:', path)
"
```

- `data_type='FP16'`: la VPU Myriad X solo ejecuta en FP16.
- `shaves=6`: núcleos SHAVE asignados (de 16 disponibles).
- `compile_params=['-ip U8']`: la entrada espera `uint8` crudo — la normalización queda on-device, no en la Raspberry Pi.

El archivo se renombró de `..._openvino_2022.1_6shave.blob` a `turtlebot_signals_v2.blob`
para que coincida con el default de `test_depthai_yolo.py`.

> **Nota:** este paso sube el `.onnx` a los servidores de Luxonis. Aceptable para este
> modelo (detector de señales, sin datos sensibles); si se reentrena con datos
> propietarios, evaluar la ruta 100% local (OpenVINO 2022.3 + `compile_tool`).
