# Robot_coche/src — Descripción de la función de cada archivo

Este documento explica para qué sirve cada archivo/carpeta dentro de `Robot_coche/src`. Este directorio es el área de código fuente del workspace de ROS2 e incluye dos paquetes ROS2 en Python:

- `freenove_4wd_hw`: módulos de hardware/algoritmos (copiados desde `Code/Server`), usados como librería por los nodos
- `freenove_4wd_nodes`: paquete de nodos ROS2 (rclpy); publica/suscribe topics y, al ejecutarse, controla el coche y la cámara

---

## 1) freenove_4wd_hw (paquete de módulos de hardware)

Ruta: `Robot_coche/src/freenove_4wd_hw/`

### 1.1 Archivos de build y metadatos del paquete

- `package.xml`
  - Manifiesto del paquete ROS2: declara nombre, dependencias, licencia, etc. colcon/ament lo usa para resolver dependencias.
- `setup.py`
  - Entrada de instalación del paquete Python (setuptools). Se usa durante `colcon build` para instalar los módulos en `install/`.
- `setup.cfg`
  - Configuración típica de ament_python (por ejemplo, directorio de instalación de scripts).
- `resource/freenove_4wd_hw`
  - Archivo de recurso del índice de ament (normalmente contiene el nombre del paquete). Permite que ROS2 descubra el paquete en el entorno.

### 1.2 Directorio de módulos Python: freenove_4wd_hw/

Ruta: `Robot_coche/src/freenove_4wd_hw/freenove_4wd_hw/`

- `__init__.py`
  - Inicialización del paquete. Aquí se añade el directorio del paquete a `sys.path` para mantener compatibilidad con importaciones “en el mismo directorio” del código original (por ejemplo `from pca9685 import PCA9685`), evitando reescribir todos los imports.
- `adc.py`
  - Lectura ADC por I2C (ADS7830): fototransistores/fotorresistencias, voltaje de batería y otras entradas analógicas.
- `buzzer.py`
  - Control del zumbador por GPIO (encendido/apagado).
- `car.py`
  - Abstracción del coche: combina motor, servo, ultrasonidos, infrarrojos y ADC; contiene lógicas de modos (evitación, seguimiento de línea, seguimiento de luz, rotación).
- `command.py`
  - Conjunto de constantes de comandos de la etapa TCP (normalmente ya no se usa en ROS2, pero se conserva para reutilizar/extender).
- `infrared.py`
  - Lectura de 3 sensores infrarrojos de línea (gpiozero LineSensor), lectura individual y valor combinado.
- `led.py`
  - Capa de efectos LED (seguimiento, respiración, arcoíris, etc.). Selecciona driver WS281x o SPI según la versión de hardware.
- `message.py`
  - Parser de mensajes del protocolo TCP (`CMD_xxx#...`). El ROS2 actual no depende de él, pero se conserva para facilitar un futuro bridge TCP→ROS2.
- `motor.py`
  - Control de motores de 4 ruedas: mapea 4 “duties” a canales PWM de PCA9685; soporta avance/retroceso/freno.
- `parameter.py`
  - Gestión de parámetros de versión de hardware (`params.json`): versión de placa/conexión/Raspberry Pi; decide driver LED, coeficientes ADC, etc.
- `pca9685.py`
  - Driver PCA9685 (16 canales PWM por I2C): frecuencia PWM, duty por canal, pulso de servo, etc.
- `rpi_ledpixel.py`
  - Driver de tira LED basado en `rpi_ws281x` (tipo WS281x).
- `servo.py`
  - Control de servos: convierte ángulo a pulso PWM y lo envía al canal correspondiente del PCA9685.
- `spi_ledpixel.py`
  - Driver WS2812 por SPI: genera la forma de onda por SPI para simular el timing WS2812.
- `ultrasonic.py`
  - Medición de distancia por ultrasonidos (gpiozero DistanceSensor), devuelve distancia en cm/m.

---

## 2) freenove_4wd_nodes (paquete de nodos ROS2)

Ruta: `Robot_coche/src/freenove_4wd_nodes/`

### 2.1 Archivos de build y metadatos del paquete

- `package.xml`
  - Manifiesto del paquete ROS2: declara dependencias de `rclpy`, tipos de mensajes (std_msgs/geometry_msgs/sensor_msgs) y la dependencia de ejecución hacia `freenove_4wd_hw`.
- `setup.py`
  - Entrada de instalación del paquete Python y registro de ejecutables mediante `entry_points.console_scripts` (para `ros2 run ...`).
- `setup.cfg`
  - Configuración típica de ament_python.
- `resource/freenove_4wd_nodes`
  - Archivo de recurso del índice de ament (descubrimiento del paquete).
- `launch/bringup.launch.py`
  - Archivo launch: inicia el nodo de base y el nodo de cámara con un solo comando.

### 2.2 Directorio de nodos Python: freenove_4wd_nodes/

Ruta: `Robot_coche/src/freenove_4wd_nodes/freenove_4wd_nodes/`

- `__init__.py`
  - Inicialización del paquete (archivo vacío, indica que es un package Python).
- `car_base_node.py`
  - Nodo ROS2 (rclpy) para base/hardware:
    - Suscribe `/cmd_vel` (Twist) para controlar motores
    - Publica ultrasonidos, infrarrojos de línea, luz y voltaje de batería
    - Suscribe topics de control para zumbador/servo/modo LED
    - Llama internamente a los módulos de `freenove_4wd_hw` (Car/Led/Buzzer, etc.)
- `camera_driver.py`
  - Driver de cámara (copiado desde `Code/Server/camera.py`) basado en Picamera2 que produce frames JPEG.
  - Se usa como “capa driver” desde `camera_node.py` para reducir acoplamiento entre ROS2 y la implementación de cámara.
- `camera_node.py`
  - Nodo ROS2 (rclpy) de cámara:
    - Usa un hilo en segundo plano para obtener frames (porque `get_frame()` bloquea esperando un frame nuevo)
    - Publica `/image_raw/compressed` (sensor_msgs/CompressedImage, JPEG)
    - Soporta parámetros: resolución, flip, intervalo mínimo de publicación, etc.

