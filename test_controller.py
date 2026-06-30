#!/usr/bin/env python3
import time
import sys
import numpy as np

# Importamos la nueva librería que emula la interfaz del simulador
from TurtleBotController.turtlebot import TurtleBotReal

def main():
    print("Iniciando conexión con el TurtleBot real (DOMAIN_ID 77)...")
    robot = TurtleBotReal("config.json")
    
    print("\nRobot listo. Comenzando exploración con zonas de riesgo...")
    print("Presiona Ctrl+C para detener el robot y salir.\n")
    
    dt = 0.1  # Bucle a 10 Hz
    
    # === ZONAS DEFINIDAS ===
    # Radio físico: 0.17m
    # Evasión de Emergencia (Giro brusco, STOP): < 0.22m
    # Zona de Precaución (Giro sutil, reduce velocidad): < 0.70m
    
    try:
        while True:
            scan = robot.get_lidar_scan()
            
            if len(scan) < 360:
                time.sleep(dt)
                continue
            
            # Puntos cardinales para calibrar el LiDAR
            frente_0   = scan[0]
            frente_180 = scan[180]
            izq_90     = scan[90]
            der_270    = scan[270]
            
            min_dist_absoluta = min(scan)
            min_indice = scan.index(min_dist_absoluta)
            
            # Abanico frontal (-30 a 30 grados) - asumiendo que 0 es frente por ahora
            abanico_frontal = scan[0:30] + scan[330:360]
            dist_frontal = min(abanico_frontal)
            
            v_target = 0.0
            w_target = 0.0
            estado = ""
            
            if dist_frontal <= 0.22:
                # [ZONA ROJA]: Riesgo inminente de choque. Frenar y girar BRUSCAMENTE
                v_target = 0.0
                estado = "🔴 EMERGENCIA"
                
                dist_izq = min(scan[30:90])   
                dist_der = min(scan[270:330]) 
                
                # Giro brusco (1.5 rad/s)
                if dist_izq > dist_der:
                    w_target = 1.5  
                else:
                    w_target = -1.5 
                    
            elif dist_frontal <= 0.70:
                # [ZONA AMARILLA]: Obstáculo detectado. Reducir velocidad y girar SUTILMENTE
                # La velocidad decrece linealmente según se acerca (de 0.2 a 0.05)
                v_target = max(0.05, 0.2 * (dist_frontal - 0.22) / (0.70 - 0.22))
                estado = "🟡 PRECAUCIÓN"
                
                # Desviación sutil (0.5 rad/s)
                if min(scan[30:90]) > min(scan[270:330]):
                    w_target = 0.5  
                else:
                    w_target = -0.5 
            else:
                # [ZONA VERDE]: Camino libre
                v_target = 0.2
                w_target = 0.0
                estado = "🟢 EXPLORANDO"
                
            # Logueo en tiempo real
            sys.stdout.write(f"\r[{estado:<15}] Frente: {dist_frontal:.2f}m | 0°:{frente_0:.2f} 90°:{izq_90:.2f} 180°:{frente_180:.2f} 270°:{der_270:.2f}   ")
            sys.stdout.flush()
                
            robot.move(v_target, w_target, dt)
            
    except KeyboardInterrupt:
        print("\n\nPrograma interrumpido por el usuario (Ctrl+C).")
    except Exception as e:
        print(f"\nOcurrió un error inesperado: {e}")
    finally:
        print("Apagando y frenando el robot de forma segura...")
        try:
            robot.stop()
        except:
            pass

if __name__ == "__main__":
    main()
