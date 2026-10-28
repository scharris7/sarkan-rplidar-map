# SARKAN — RPLiDAR A1 3D Visualizer

Herramienta desarrollada en Python para el proyecto **SARKAN**, orientada a la adquisición de datos, el filtrado espacial de ruido y la visualización interactiva en 3D del sensor **RPLiDAR A1M1-R3**. Incluye una vista cenital estabilizada y colorimetría basada en la proximidad.

El script utiliza captura serial mediante `pyserial`, control del motor mediante comandos binarios y líneas de estado (`RTS`/`DTR`), y renderizado en tiempo real con `Open3D`.

## Tabla de contenido

- [Descripción](#descripción)
- [Hardware y requisitos](#hardware-y-requisitos)
- [Instalación de dependencias](#instalación-de-dependencias)
- [Validación inicial del hardware](#validación-inicial-del-hardware)
- [Instalación del proyecto](#instalación-del-proyecto)
- [Ejecución](#ejecución)
- [Controles interactivos](#controles-interactivos)

## Descripción

Antes de integrar el sensor en los nodos del rover o en sistemas de navegación más avanzados, se recomienda validar de forma independiente que el RPLiDAR responde correctamente y entrega un escaneo utilizable.

El repositorio incluye el siguiente script principal:

- **`ver_lidar_3D.py`**: adquiere datos por puerto serial, aplica un filtro de densidad para reducir ruido, muestra en la terminal un análisis de los cúmulos detectados y abre un visor gráfico 3D interactivo.

## Hardware y requisitos

- **Sensor LiDAR:** RPLiDAR A1M1-R3.
- **Adaptador USB-UART:** CP2102.
- **Equipo anfitrión:** PC o NVIDIA Jetson con Linux.
- **Software:** Python 3.
- **Librerías de Python:**
  - `pyserial`
  - `open3d`
  - `numpy`

### Instalar dependencias

Instala las librerías necesarias con `pip`:

```bash
pip install pyserial open3d numpy
```

> **Nota:** si tu sistema utiliza un entorno virtual de Python, actívalo antes de instalar las dependencias y ejecutar el script.

## Validación inicial del hardware

### 1. Conectar y detectar el puerto serial

Conecta el adaptador USB del LiDAR al equipo anfitrión y comprueba que Linux detecte el dispositivo:

```bash
ls /dev/ttyUSB*
```

Normalmente, el puerto puede aparecer como `/dev/ttyUSB0`. El nombre exacto puede variar según los dispositivos conectados.

### 2. Verificar los permisos del puerto

Si no tienes permisos para abrir el puerto serial, puedes otorgar permisos temporales de lectura y escritura:

```bash
sudo chmod 666 /dev/ttyUSB0
```

> **Importante:** reemplaza `/dev/ttyUSB0` por el puerto que haya detectado tu sistema. Este cambio de permisos es temporal y permite el acceso a cualquier usuario; úsalo solo si es necesario.

## Instalación del proyecto

Clona el repositorio en el directorio donde quieras trabajar. Por ejemplo, para utilizar `~/lidar_test`:

```bash
mkdir -p ~/lidar_test
cd ~/lidar_test
git clone https://github.com/scharris7/sarkan-rplidar-map.git .
```

## Ejecución

1. Abre una terminal y entra en la carpeta del proyecto:

   ```bash
   cd ~/lidar_test
   ```

2. Ejecuta el script principal:

   ```bash
   python3 ver_lidar_3D.py
   ```

Según el comportamiento previsto del script, se buscará automáticamente un puerto serial compatible y se abrirá el visor de Open3D. El motor permanecerá detenido inicialmente hasta que lo actives mediante el control interactivo.

## Controles interactivos

Con la ventana de visualización 3D abierta, puedes utilizar los siguientes controles:

| Tecla o acción | Descripción |
|---|---|
| **`ESPACIO`** | Enciende o apaga el motor y el láser del LiDAR. El sistema inicia con el motor apagado. |
| **Ratón: clic izquierdo + arrastrar** | Permite ajustar la perspectiva de la vista 3D. |
| **Terminal** | Muestra el análisis de los cúmulos detectados mediante filtros de densidad, incluyendo su ángulo y distancia promedio en metros. |

---

## Información del proyecto

- **Proyecto:** SARKAN
- **Sensor:** RPLiDAR A1M1-R3
- **Plataforma:** Linux
- **Tecnologías:** Python 3, `pyserial`, NumPy y Open3D
