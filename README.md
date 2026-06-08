# Robot PID Control - Guía de Usuario (Wi-Fi & WebSockets)

Este proyecto implementa un control de velocidad PID para un robot diferencial basado en ESP32, permitiendo navegación por odometría y control remoto vía Wi-Fi.

---

## 🚀 Guía de Instalación Rápida

### 1. Configuración de Credenciales (Wi-Fi)
Antes de cargar el código, debes configurar tu red:
1. Abre `PID.ino` en el Arduino IDE.
2. Busca las siguientes líneas al principio del archivo:
   ```cpp
   const char* ssid = "TU_SSID";
   const char* password = "TU_PASSWORD";
   ```
3. Reemplaza `TU_SSID` y `TU_PASSWORD` con los datos de tu red Wi-Fi (2.4GHz recomendada).

### 2. Carga al ESP32 (Arduino IDE)
1. Conecta tu ESP32 a la computadora mediante USB.
2. En el Arduino IDE, selecciona tu placa .
3. Selecciona el puerto COM correcto.
4. Haz clic en **Subir (Upload)**.

### 3. Visualización de la IP
1. Una vez terminada la carga, abre el **Monitor Serie** (Herramientas -> Monitor Serie).
2. Configura la velocidad a **115200 baudios**.
3. Presiona el botón **EN/RST** del ESP32.
4. Verás puntos suspensivos mientras se conecta y finalmente aparecerá:
   `WiFi connected`
   `IP address: 192.168.X.X` <-- **Anota esta IP**.

### 4. Prueba de Conexión (Ping)
Para asegurarte de que tu PC puede "ver" al robot:
1. Asegúrate de que tu PC esté conectada a la **misma red Wi-Fi** que el robot.
2. Abre una terminal (CMD en Windows o Terminal en Linux).
3. Escribe el comando:
   ```bash
   ping [IP_DEL_ROBOT]
   ```
   *Ejemplo:* `ping 192.168.1.50`
4. Si recibes respuestas (`Tiempo de respuesta...`), la red está configurada correctamente.

---

## 🐍 Configuración del Entorno de Control (Python)
Recomendado usar **Python 3.11.15**.

1. **Abrir una Terminal:** Navega hasta la carpeta del proyecto.
2. **Entrar a la carpeta de control:**
   ```bash
   cd control-robot
   ```
3. **Crear Entorno Virtual (VENV):**
   ```bash
   python -m venv venv
   ```
4. **Activar Entorno:**
   - **Windows:** `.\venv\Scripts\activate`
   - **Linux/Mac:** `source venv/bin/activate`
5. **Instalar Dependencias:**
   ```bash
   pip install -r requirements.txt
   ```

---

## 🎮 Comandos de Control

| Comando | Acción | Ejemplo | Descripción |
| :--- | :--- | :--- | :--- |
| **D** | Avanzar Recto | `D:100` | Avanza 100 cm (1 metro) |
| **A** | Girar Grados | `A:90` | Gira 90° a la derecha |
| **S** | Parada Total | `S` | Detiene motores y limpia PID |
| **V** | Velocidad RPM | `V:100,100` | Establece RPM objetivo directas |
| **R** | Reset | `R` | Reinicia odometría y posición (0,0,0) |

---

## 🛠️ Ajustes de Hardware y PID

- **PWM Mínimo:** Ajustar `PWM_DEADBAND = 235` en `PID.ino` para vencer la fricción.
- **Física:** `WHEEL_DIAMETER` (4.5 cm) y `WHEEL_BASE` (13.5 cm) configurados.

---

## 📊 Herramienta de Interfaz Todo-en-Uno

Para controlar el robot y ver las RPM en tiempo real:
```bash
python interfaz_completa.py [IP_DEL_ROBOT]
```
*(Asegúrate de tener el entorno virtual activo)*.
