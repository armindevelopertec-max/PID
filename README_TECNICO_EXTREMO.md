# Manual Técnico Maestro: Firmware MrRootBot Ultra Final

Este documento es una guía exhaustiva y profunda sobre cada aspecto del código fuente `PID.ino`. Está diseñado para ingenieros y desarrolladores que necesiten entender la implementación a nivel de registro, algoritmo y física.

---

## 1. Fundamentos del Sistema y Configuración de Hardware

El firmware está diseñado para el microcontrolador **ESP32**, aprovechando su arquitectura de doble núcleo (aunque aquí se usa un modelo de super-loop) y sus periféricos de hardware especializados.

### 1.1. Periférico PWM (LEDC)
A diferencia de los Arduino tradicionales, el ESP32 no usa `analogWrite`. Se utiliza el periférico **LEDC**:
- **Frecuencia (`PWM_FREQ = 5000`)**: 5 kHz. Elegida para estar fuera del rango auditivo molesto y proporcionar una respuesta rápida en los motores DC.
- **Resolución (`PWM_RESOLUTION = 8`)**: 8 bits, lo que permite un rango de 0 a 255.
- **Configuración**: Se utilizan dos canales LEDC (0 y 1) vinculados a los pines de habilitación `ENA` (25) y `ENB` (26).

### 1.2. Interfaz de Potencia (Puente H L298N o similar)
El control de dirección se gestiona mediante lógica binaria en los pines `IN1, IN2, IN3, IN4`. 
- **Estado de Frenado**: El código no implementa frenado activo por cortocircuito, sino que pone los targets en 0 y permite que el PID maneje la deceleración o que la fricción actúe.

---

## 2. Captura de Datos de Alta Frecuencia: Encoders

El robot utiliza encoders de cuadratura con una resolución de **5600 pulsos por revolución**. Esto significa que cada pulso representa un desplazamiento angular de ~0.064 grados.

### 2.1. Interrupciones de Hardware (ISRs)
Las funciones `encoderL()` y `encoderR()` están marcadas con `IRAM_ATTR`, lo que las coloca en la memoria RAM estática. Esto evita latencias causadas por la lectura de la memoria Flash, permitiendo manejar miles de interrupciones por segundo sin colapsar el CPU.

### 2.2. Lógica de Decodificación de Cuadratura
El código implementa un decodificador de estado:
```cpp
int newL = (digitalRead(C1L) << 1) | digitalRead(C2L);
int diff = (newL - actL) & 3;
```
- Se crea un número de 2 bits con el estado de ambos canales.
- La diferencia `diff` determina la dirección:
    - `1`: Giro en un sentido.
    - `3`: Giro en sentido opuesto.
    - Cualquier otro valor se ignora (ruido o salto de estado).

### 2.3. Filtrado de Ruido (Software Debounce)
`if (now - lastIntL < 100) return;`
Este umbral de 100 microsegundos limita la frecuencia máxima detectable a 10 kHz, lo cual es suficiente para las RPM máximas del robot pero filtra picos de ruido electromagnético generados por las escobillas de los motores.

---

## 3. Procesamiento de Señal y Odometría

### 3.1. Cálculo de Velocidad con Diferenciación Numérica
La velocidad "raw" se obtiene como:
$v_{raw} = \frac{\Delta n}{\Delta t} \cdot \frac{60}{PPR}$
Donde $\Delta n$ es el cambio en los contadores y $\Delta t$ es el tiempo transcurrido (aprox. 20ms).

### 3.2. Filtro de Media Exponencial (LPF)
Para evitar que el ruido del encoder afecte al control derivativo, se usa un filtro paso bajo:
$y[n] = \alpha \cdot x[n] + (1-\alpha) \cdot y[n-1]$
Con `ALPHA_RPM = 0.5`, se logra un equilibrio entre suavidad y latencia en la lectura de velocidad.

### 3.3. Integración de la Pose (Odometría)
El robot estima su posición relativa mediante el modelo de **Cinemática Diferencial**:
1. **Distancia lineal de cada rueda**: $d_L, d_R$.
2. **Distancia central**: $d_c = \frac{d_R + d_L}{2}$.
3. **Cambio de orientación**: $\Delta \theta = \frac{d_R - d_L}{WheelBase}$.
4. **Actualización de Coordenadas**:
   - $X_{nuevo} = X_{anterior} + d_c \cdot \cos(\theta)$
   - $Y_{nuevo} = Y_{anterior} + d_c \cdot \sin(\theta)$
   - $\theta_{nuevo} = \theta_{anterior} + \Delta \theta$

El ángulo se normaliza usando `atan2(sin(T), cos(T))` para mantenerlo siempre en el rango $[-\pi, \pi]$.

---

## 4. El Algoritmo de Control PID

Es el núcleo del firmware. No es un PID estándar; es una implementación robusta con características avanzadas.

### 4.1. Rampas de Aceleración
`rampL += constrain(targetRPML - rampL, -step, step);`
Esto limita la derivada de la velocidad (aceleración). Evita que el robot "patine" al arrancar o que los engranajes sufran impactos mecánicos bruscos.

### 4.2. Término Proporcional ($P$)
Multiplica el error actual por `Kp = 3.8`. Proporciona la fuerza principal para alcanzar el objetivo.

### 4.3. Término Integral ($I$) y Anti-Windup
Multiplica la acumulación del error por `Ki = 2.5`.
**Lógica de Saturación (Anti-Windup):**
```cpp
if (abs(outL) < 255 || (outL * errorL < 0)) {
    integralL += errorL * dt;
}
```
Si el motor ya está al 100% de potencia (`outL >= 255`), la integral deja de acumular. Esto evita que, si el robot se bloquea físicamente, la integral crezca infinitamente, lo que causaría un comportamiento violento al desbloquearse.

### 4.4. Término Derivativo ($D$) con Filtrado Dual
El término $D$ es extremadamente sensible al ruido. El código aplica dos capas de protección:
1. **Media Móvil**: Promedia las últimas 3 lecturas de la derivada.
2. **Filtro Paso Bajo**: Aplica `ALPHA_DERIV = 0.8` para suavizar la señal resultante.
Esto permite usar un `Kd = 0.15` que realmente ayuda a frenar oscilaciones sin introducir vibración.

---

## 5. Comunicación y Seguridad

### 5.1. Protocolo Serial sobre Bluetooth
El sistema escucha continuamente en `SerialBT`. Utiliza un buffer de 64 caracteres para reconstruir líneas completas.
- El comando `K` (Cinemática) es el más sofisticado, ya que calcula internamente las RPM necesarias para realizar un giro con radio de curvatura específico.

### 5.2. El Sistema de Perro Guardián (Watchdog)
`if (nowMillis - lastCmdTime > WATCHDOG_MS)`
Si el software de control (app o PC) deja de enviar datos por más de 2 segundos, el robot asume pérdida de conexión y pone las velocidades objetivo en 0. Es una medida de seguridad crítica para evitar que el robot "escape" fuera de control.

---

## 6. Sincronización Temporal (Real-Time Scheduling)

El código no usa `delay()`. Utiliza `micros()` para calcular el `dt` real de cada iteración.
- El `LOOP_PERIOD_US = 20000` garantiza una frecuencia de actualización de **50 Hz**.
- Toda la física (integrales de odometría y PID) depende de este `dt` preciso para ser matemáticamente correcta.

---

## Resumen de Parámetros de Ajuste
- **Kp (3.8)**: Agresividad.
- **Ki (2.5)**: Precisión en estado estable.
- **Kd (0.15)**: Estabilidad y amortiguación.
- **MAX_ACCEL (200)**: Suavidad de movimiento.
- **DEADBAND (45)**: Torque inicial para vencer fricción estática.
