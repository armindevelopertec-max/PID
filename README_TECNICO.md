# Documentación Técnica Detallada: MrRootBot Ultra Final

Este documento proporciona una explicación técnica exhaustiva del firmware `PID.ino`, diseñado para un robot diferencial basado en ESP32 con control de bucle cerrado.

---

## 1. Arquitectura del Sistema
El firmware opera bajo un modelo de **ejecución periódica** mediante un bucle de control de tiempo real. Utiliza interrupciones de hardware para la captura de señales de alta frecuencia (encoders) y un lazo principal que sincroniza la sensórica, el control y la telemetría.

## 2. Definiciones de Hardware y Constantes Físicas

### Asignación de Pines (I/O)
*   **Puente H (Motores):** Pines 22, 19 (Motor L) y 18, 21 (Motor R).
*   **PWM (Enable):** Pines 25 y 26, utilizando el periférico `LEDC` del ESP32.
*   **Encoders:** Pines 35, 34 (Izquierdo) y 32, 33 (Derecho). Se configuran como `INPUT_PULLUP`.

### Especificaciones del Robot
*   `PULSES_PER_REV = 5600`: Resolución extremadamente alta para precisión milimétrica.
*   `WHEEL_DIAMETER = 0.065m`: Diámetro de las ruedas para conversión de pulsos a metros.
*   `WHEEL_BASE = 0.15m`: Distancia entre ejes para cálculos cinemáticos y odometría.

## 3. Subsistema de Encoders (ISRs)
El código utiliza interrupciones en los pines del canal A de cada encoder (`C1L` y `C1R`) configuradas en modo `CHANGE`.

*   **Lógica de Cuadratura:** En cada interrupción, se lee el estado actual de ambos canales (A y B) para determinar la dirección del giro mediante una operación de bits (`(newL - actL) & 3`).
*   **Debounce por Software:** Se implementa un bloqueo de 100 microsegundos (`now - lastInt < 100`) para filtrar ruido eléctrico en las señales de los encoders.
*   **Atributo `IRAM_ATTR`:** Las funciones `encoderL()` y `encoderR()` se ejecutan desde la memoria RAM interna para minimizar la latencia de respuesta.

## 4. Procesamiento de Sensores y Odometría (`updateSensors`)

### Estimación de Velocidad
La velocidad se calcula como la derivada temporal de la posición:
$Vel_{RPM} = \frac{\Delta Pulsos \cdot 60}{\Delta t \cdot Pulsos_{Rev}}$

*   **Filtro Paso Bajo (LPF):** Se aplica un filtro de media exponencial: `filteredVel = ALPHA_RPM * vel + (1 - ALPHA_RPM) * filteredVel`. Esto suaviza el ruido de cuantización de los encoders.

### Odometría (Dead Reckoning)
Calcula la pose $(X, Y, \theta)$ integrando el desplazamiento:
1.  **Distancia por Rueda:** $ds = \frac{RPM}{60} \cdot \pi \cdot D \cdot dt$
2.  **Desplazamiento Central ($dc$):** Promedio de $ds_L$ y $ds_R$.
3.  **Variación Angular ($d\theta$):** $\frac{ds_R - ds_L}{WheelBase}$.
4.  **Integración:** Actualiza coordenadas cartesianas usando senos y cosenos del ángulo actual.

## 5. Algoritmo de Control (`updateControl`)

### Rampa de Aceleración
Para evitar picos de corriente y proteger la mecánica, las velocidades objetivo (`targetRPM`) no se aplican instantáneamente. Se utiliza `MAX_ACCEL` para aproximar la velocidad real a la deseada de forma lineal.

### El Lazo PID
El controlador implementa la ecuación estándar: $Salida = Kp \cdot e + Ki \cdot \int e + Kd \cdot \frac{de}{dt}$.

*   **Término Proporcional ($Kp$):** Respuesta inmediata al error actual.
*   **Término Integral ($Ki$):** Elimina el error de estado estacionario.
    *   **Anti-Windup:** La integración solo ocurre si la salida no está saturada (`abs(out) < 255`) o si el error está cambiando de signo, evitando que el error acumulado "explote".
*   **Término Derivativo ($Kd$):** Anticipa el error futuro y amortigua oscilaciones.
    *   **Suavizado de Derivada:** Utiliza un historial de 3 muestras y un filtro paso bajo adicional (`ALPHA_DERIV`) para evitar que el ruido del encoder se traduzca en vibraciones en el motor.

### Gestión de Deadband
Si la salida calculada es menor que `PWM_DEADBAND` (45) pero mayor que 0, se fuerza a 45 para vencer la inercia mínima del motor y evitar el "zumbido" sin movimiento.

## 6. Cinemática Inversa (`setVelocity`)
Traduce comandos de velocidad lineal ($v$) y angular ($\omega$) a velocidades de rueda individuales:
*   $v_L = v - \frac{\omega \cdot Base}{2}$
*   $v_R = v + \frac{\omega \cdot Base}{2}$

## 7. Bucle de Ejecución y Comunicación

### Tiempo Real
El `loop()` garantiza que `updateSensors` y `updateControl` se ejecuten estrictamente cada 20ms (`LOOP_PERIOD_US = 20000`). Esto es crítico para la estabilidad de las integrales y derivadas del PID.

### Protocolo de Comandos
El parser de Bluetooth identifica cabeceras:
*   `V`: Control directo de RPM (Modo bajo nivel).
*   `K`: Control cinemático (Modo navegación).
*   `S`: Parada de seguridad inmediata.
*   `R`: Reseteo de estado global.

### Watchdog
Si no se recibe un comando de movimiento en 2000ms, el robot detiene los motores automáticamente poniendo los targets de RPM en 0.

---

Este firmware representa un sistema de control robusto diseñado para aplicaciones que requieren precisión en el posicionamiento y suavidad en el movimiento.
