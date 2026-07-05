import time
from TurtleBotController.turtlebot import TurtleBotReal

def main():
    print("==================================================")
    print(" PRUEBA DE VISIÓN (YOLO NANO) Y ALINEACIÓN")
    print("==================================================")
    
    # Inicializa el robot (esto arrancará el pipeline de DepthAI y ROS)
    robot = TurtleBotReal("TurtleBotController/config.json")
    
    print("\nRobot listo. Buscando señales...")
    print("Presiona Ctrl+C para salir.")
    
    try:
        while True:
            detections = robot.get_vision_detections()
            if detections:
                print("\n[!] Señales detectadas en este frame:")
                for det in detections:
                    print(f" -> Clase: {det['class']}")
                    print(f"    Distancia: {det['distance']:.2f} m")
                    print(f"    Ángulo relativo: {det['relative_angle']:.2f} rad")
            time.sleep(0.1)  # Leer a 10 Hz
            
    except KeyboardInterrupt:
        print("\nPrueba de visión terminada.")
    finally:
        robot.stop()
        # Asegurarse de cerrar todo limpiamente
        if hasattr(robot, 'vpu_detector') and robot.vpu_detector:
            robot.vpu_detector.close()

if __name__ == '__main__':
    main()
