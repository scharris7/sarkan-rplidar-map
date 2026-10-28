import serial
import serial.tools.list_ports
import time
import math
import open3d as o3d
import numpy as np
from collections import deque

BAUD = 115200

# Estado inicial: APAGADO por defecto al arrancar el programa
estado_lidar = {"encendido": False}

def buscar_puerto_lidar():
    print("Buscando puertos seriales disponibles...")
    puertos = list(serial.tools.list_ports.comports())
    for puerto in puertos:
        if "CP2102" in puerto.description or "CP210x" in puerto.description or "USB UART" in puerto.device or "ttyUSB" in puerto.device or "ttyACM" in puerto.device:
            print(f"¡LiDAR detectado automáticamente en {puerto.device}!")
            return puerto.device
    for puerto in puertos:
        if "USB" in puerto.device or "ACM" in puerto.device:
            return puerto.device
    return None

def crear_circulo_referencia(radio, num_puntos=100):
    puntos = []
    for i in range(num_puntos):
        angulo = 2 * math.pi * i / num_puntos
        x = radio * math.cos(angulo)
        y = radio * math.sin(angulo)
        puntos.append([x, y, 0.0])
    return puntos

def main():
    PORT = buscar_puerto_lidar()
    if not PORT:
        print("--- Error: No se encontró puerto compatible. ---")
        return

    ser = serial.Serial(PORT, BAUD, timeout=0.002, dsrdtr=False, rtscts=False)
    ser.dtr = False
    time.sleep(1.0)
    ser.reset_input_buffer()
    
    # FORZAR APAGADO INICIAL: Enviar comando STOP y activar RTS/DTR para frenar el motor físico
    print("Inicializando con el LiDAR APAGADO por defecto...")
    try:
        ser.rts = True
        ser.dtr = True
        ser.write(b'\xA5\x25')  # Comando STOP del firmware
    except:
        pass

    # Configuración de Open3D
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="-LiDAR Rover-", width=1280, height=720)
    
    # Callback de teclado: Presionar ESPACIO (código 32) para encender/apagar
    def toggle_motor(vis):
        if estado_lidar["encendido"]:
            print("\nApagando motor y láser...")
            try:
                ser.write(b'\xA5\x25') # Comando STOP
                time.sleep(0.05)
                ser.rts = True
                ser.dtr = True
            except:
                pass
            estado_lidar["encendido"] = False
        else:
            print("\nEncendiendo motor y reanudando escaneo...")
            try:
                ser.rts = False
                ser.dtr = False
                time.sleep(0.1)
                ser.reset_input_buffer()
                ser.write(b'\xA5\x20') # Comando START SCAN
            except:
                pass
            estado_lidar["encendido"] = True
        return False

    vis.register_key_callback(32, toggle_motor) # Tecla Espacio

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)
    
    # Guías de distancia estáticas (0.5m, 1.0m, 2.0m)
    puntos_guias = []
    for r in [0.5, 1.0, 2.0]:
        puntos_guias.extend(crear_circulo_referencia(r))
    
    pcd_guias = o3d.geometry.PointCloud()
    pcd_guias.points = o3d.utility.Vector3dVector(np.array(puntos_guias))
    colores_guias = np.zeros((len(puntos_guias), 3)) + 0.2
    pcd_guias.colors = o3d.utility.Vector3dVector(colores_guias)
    vis.add_geometry(pcd_guias)

    opt = vis.get_render_option()
    opt.background_color = np.array([0.05, 0.05, 0.05])
    opt.point_size = 7.0
    opt.show_coordinate_frame = True
    
    print("\n================ CONTROLES DE TECLADO ================")
    print("  [ESPACIO] : Encender / Apagar el motor y láser")
    print("  [Cerrar]  : Salir de la ventana")
    print("======================================================\n")

    historial_puntos = deque(maxlen=2500)
    primera_vez = True
    contador_frames = 0

    try:
        while vis.poll_events():
            puntos_vuelta_actual = []
            
            # Solo procesar datos si el usuario encendió el LiDAR con la barra espaciadora
            if estado_lidar["encendido"] and ser.in_waiting >= 5:
                data_lote = ser.read(ser.in_waiting)
                i = 0
                while i < len(data_lote) - 4:
                    b0 = data_lote[i]
                    s = b0 & 0x01
                    not_s = (b0 >> 1) & 0x01
                    
                    if s != not_s:
                        packet = data_lote[i:i+5]
                        if len(packet) == 5:
                            angle_deg = ((packet[1] >> 1) | (packet[2] << 7)) / 64.0
                            distance_mm = (packet[3] | (packet[4] << 8)) / 4.0
                            
                            if distance_mm > 0:
                                dist_m = distance_mm / 1000.0
                                if 0.1 < dist_m < 3.0:
                                    angle_rad = math.radians(angle_deg)
                                    x = dist_m * math.cos(angle_rad)
                                    y = dist_m * math.sin(angle_rad)
                                    puntos_vuelta_actual.append((angle_deg, dist_m, x, y))
                                
                                i += 5
                            else:
                                i += 1
                        else:
                            i += 1
                    else:
                        i += 1

            # Filtrado espacial anti-ruido (solo si está encendido)
            if estado_lidar["encendido"] and puntos_vuelta_actual:
                puntos_filtrados_frame = []
                coords_actuales = np.array([[p[2], p[3]] for p in puntos_vuelta_actual])
                
                for idx, (ang, dist, x, y) in enumerate(puntos_vuelta_actual):
                    if len(coords_actuales) > 1:
                        distancias_vecinos = np.linalg.norm(coords_actuales - np.array([x, y]), axis=1)
                        vecinos_cercanos = np.sum((distancias_vecinos > 0) & (distancias_vecinos < 0.07))
                        
                        if vecinos_cercanos >= 2:
                            historial_puntos.append([x, y, 0.0])
                            puntos_filtrados_frame.append((ang, dist, x, y))

                # Análisis de objetivos sólidos
                contador_frames += 1
                if contador_frames % 15 == 0 and len(puntos_filtrados_frame) > 5:
                    sectores = {}
                    for ang, d, x, y in puntos_filtrados_frame:
                        idx_sector = int(ang // 15) * 15
                        if idx_sector not in sectores:
                            sectores[idx_sector] = []
                        sectores[idx_sector].append((ang, d))
                    
                    print("\n--- OBJETIVOS SÓLIDOS ---")
                    encontrado = False
                    for sector, pts in sectores.items():
                        if len(pts) >= 4:
                            dist_prom = sum(p[1] for p in pts) / len(pts)
                            ang_prom = sum(p[0] for p in pts) / len(pts)
                            print(f"  [Objeto] Ángulo: {ang_prom:5.1f}° | Distancia: {dist_prom:4.2f} m")
                            encontrado = True
                    if not encontrado:
                        print("  (Zona despejada)")

            # Actualización visual
            if len(historial_puntos) > 20:
                puntos_np = np.array(historial_puntos)
                pcd.points = o3d.utility.Vector3dVector(puntos_np)
                
                distancias = np.linalg.norm(puntos_np[:, :2], axis=1)
                t = np.clip((distancias - 0.1) / 2.8, 0.0, 1.0)
                
                colores = np.zeros((len(puntos_np), 3))
                colores[:, 0] = 1.0 - t
                colores[:, 1] = 0.3 * (1.0 - t)
                colores[:, 2] = t
                
                pcd.colors = o3d.utility.Vector3dVector(colores)
                vis.update_geometry(pcd)
                
                if primera_vez:
                    ctr = vis.get_view_control()
                    ctr.set_front([0.0, 0.0, -1.0])
                    ctr.set_up([0.0, 1.0, 0.0])
                    ctr.set_lookat([0.0, 0.0, 0.0])
                    ctr.set_zoom(0.4)
                    print("¡Visualizador configurado correctamente!")
                    primera_vez = False

            vis.update_renderer()
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nDeteniendo el LiDAR...")
        try:
            ser.rts = True
            ser.dtr = True
            ser.write(b'\xA5\x25')
            ser.close()
        except:
            pass
        vis.destroy_window()
        print("Visualizador cerrado.")

if __name__ == '__main__':
    main()
