import cv2
import time
import sys

try:
    import depthai as dai
except ImportError:
    print("Error: depthai no está instalado. Ejecuta 'pip install depthai'")
    sys.exit(1)

# Inicializar el detector avanzado de WeChat
try:
    detector = cv2.wechat_qrcode_WeChatQRCode()
except Exception as e:
    print(f"Error al inicializar WeChatQRCode: {e}")
    print("Es posible que necesites descargar los modelos .caffemodel si tu OpenCV es antiguo.")
    sys.exit(1)

# Usaremos el OAK-D a través de DepthAI en lugar de VideoCapture(0) para garantizar acceso a la cámara
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam_rgb.setFps(15)

xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

contador_qr = 0
ultimo_tiempo_detectado = 0
qr_en_pantalla_anterior = False
TIEMPO_REINICIO = 5.0

print("==================================================")
print(" BUSCADOR DE CÓDIGOS QR (OAK-D + WeChat Engine)")
print("==================================================")
print("Presiona Ctrl+C para salir.\nBuscando...")

try:
    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        while True:
            in_rgb = q_rgb.get()
            if in_rgb is None:
                continue
                
            frame = in_rgb.getCvFrame()
            
            # El detector de WeChat es sumamente robusto contra brillos y arrugas
            datos, puntos = detector.detectAndDecode(frame)
            tiempo_actual = time.time()

            # WeChat devuelve los puntos si detecta y decodifica con éxito
            if puntos is not None and len(puntos) > 0 and datos and datos[0] != "":
                if not qr_en_pantalla_anterior:
                    contador_qr += 1
                    print(f"\n[¡DETECTADO!] QR detectado {contador_qr} veces.")
                    print(f"Contenido del QR: {datos[0]}")
                    qr_en_pantalla_anterior = True
                
                ultimo_tiempo_detectado = tiempo_actual
            else:
                qr_en_pantalla_anterior = False
                
                if contador_qr > 0 and (tiempo_actual - ultimo_tiempo_detectado) >= TIEMPO_REINICIO:
                    print(f"\n--- Pasaron {TIEMPO_REINICIO} segundos sin detección. Contador de rachas a 0 ---")
                    contador_qr = 0

            # (Removido cv2.imshow para evitar el lag monstruoso del SSH/X11)
            
except KeyboardInterrupt:
    print("\nSaliendo del detector de QR...")
