# MrRootBot Ultra Final - PID Control & Odometry

Este proyecto implementa un sistema de control PID para un robot de tracción diferencial utilizando un ESP32. Incluye control de velocidad de bucle cerrado, odometría en tiempo real y comunicación mediante Bluetooth Serial.

## Características

- **Control PID:** Control de velocidad independiente para cada rueda (RPM) con filtrado de derivados y rampas de aceleración.
- **Odometría:** Seguimiento de la posición (X, Y) y orientación (Theta) del robot basándose en los encoders.
- **Comunicación Bluetooth:** Control remoto y telemetría a través de Bluetooth Serial.
- **Seguridad:** Watchdog integrado que detiene los motores si se pierde la conexión o no se reciben comandos durante 2 segundos.
- **Telemetría:** Envío periódico de estado (Posición y RPMs filtradas).

## Hardware y Conexiones (ESP32)

### Motores (Puente H)
| Pin ESP32 | Función |
|-----------|---------|
| 22        | IN1     |
| 19        | IN2     |
| 18        | IN3     |
| 21        | IN4     |
| 25        | ENA (PWM) |
| 26        | ENB (PWM) |

### Encoders (Quadrature)
| Pin ESP32 | Función |
|-----------|---------|
| 35        | Canal A Izquierdo (C1L) |
| 34        | Canal B Izquierdo (C2L) |
| 32        | Canal A Derecho (C1R) |
| 33        | Canal B Derecho (C2R) |

## Protocolo de Comunicación (Bluetooth)

Nombre del dispositivo: `MrRootBot_Ultra_Final`

### Comandos de Entrada
Los comandos deben terminar con un carácter de nueva línea (`\n` o `\r`).

- `V:<RPM_L>,<RPM_R>`: Establece la velocidad objetivo en RPM para cada rueda.
- `K:<Linear>,<Angular>`: Establece la velocidad lineal (m/s) y angular (rad/s) del robot.
- `S`: Parada de emergencia (Stop). Resetea integrales y rampas.
- `R`: Resetea la odometría (X=0, Y=0, Theta=0) y los contadores de los encoders.

### Telemetría de Salida
Se envía cada 150ms con el siguiente formato:
`X:<pos_x> Y:<pos_y> T:<theta> L:<rpm_izq> R:<rpm_der>`

## Parámetros de Control y Configuración

El código utiliza los siguientes valores por defecto:
- **Kp:** 3.8
- **Ki:** 2.5
- **Kd:** 0.15
- **Pulsos por Revolución:** 5600
- **Diámetro de Rueda:** 0.065 m
- **Distancia entre Ruedas (Wheel Base):** 0.15 m
- **RPM Máximas:** 180.0

## Dependencias
- Librería `BluetoothSerial` (incluida en el core de ESP32 para Arduino).
- Requiere soporte de Bluetooth Clásico habilitado en la configuración de la placa.
