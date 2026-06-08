import asyncio
import websockets
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections
import re
import threading

# --- CONFIGURACIÓN DE DATOS ---
max_points = 100
rpm_izq_data = collections.deque([0.0] * max_points, maxlen=max_points)
rpm_der_data = collections.deque([0.0] * max_points, maxlen=max_points)
rpm_pattern = re.compile(r"RPM_Izq:([-+]?\d*\.\d+|\d+) RPM_Der:([-+]?\d*\.\d+|\d+)")

# --- LÓGICA DE COMUNICACIÓN (ASYNCIO) ---
async def robot_manager(uri):
    global rpm_izq_data, rpm_der_data
    
    try:
        async with websockets.connect(uri) as websocket:
            print(f"\n[OK] Conectado a {uri}")
            print("Instrucciones: D:dist, A:grados, S (Stop), salir")
            print("-" * 40)

            # Tarea para recibir datos
            async def receiver():
                async for message in websocket:
                    # Actualizar datos para el gráfico
                    match = rpm_pattern.search(message)
                    if match:
                        rpm_izq_data.append(float(match.group(1)))
                        rpm_der_data.append(float(match.group(2)))
                    
                    # Mostrar telemetría en una sola línea (opcional)
                    # print(f"\r{message[:80]}...", end="", flush=True)

            # Tarea para enviar comandos
            async def sender():
                loop = asyncio.get_running_loop()
                while True:
                    cmd = await loop.run_in_executor(None, input, "Enviar Comando > ")
                    if cmd.lower() == 'salir':
                        plt.close('all')
                        sys.exit(0)
                    if cmd:
                        await websocket.send(cmd)

            # Ejecutar ambas tareas en paralelo
            await asyncio.gather(receiver(), sender())

    except Exception as e:
        print(f"\n[ERROR] {e}")
        plt.close('all')

def start_async_loop(uri):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(robot_manager(uri))

# --- LÓGICA DE INTERFAZ (MATPLOTLIB) ---
def update_plot(frame, line_izq, line_der):
    line_izq.set_ydata(rpm_izq_data)
    line_der.set_ydata(rpm_der_data)
    return line_izq, line_der

def main():
    if len(sys.argv) < 2:
        print("Uso: python interfaz_completa.py [IP_DEL_ROBOT]")
        return

    ip = sys.argv[1]
    uri = f"ws://{ip}:81"

    # Configurar Gráfico
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(10, 5))
    fig.canvas.manager.set_window_title(f"Control Robot - {ip}")
    
    ax.set_title("RPM Motores en Tiempo Real")
    ax.set_ylim(-200, 200)
    ax.set_ylabel("RPM")
    ax.grid(True, alpha=0.2)

    line_izq, = ax.plot(range(max_points), [0]*max_points, label="Izq", color="cyan")
    line_der, = ax.plot(range(max_points), [0]*max_points, label="Der", color="magenta")
    ax.legend(loc="upper right")

    # Iniciar comunicación en hilo separado
    comm_thread = threading.Thread(target=start_async_loop, args=(uri,), daemon=True)
    comm_thread.start()

    # Animación en hilo principal
    ani = FuncAnimation(fig, update_plot, fargs=(line_izq, line_der), interval=50, blit=True)
    
    print("Iniciando Interfaz Gráfica...")
    plt.show()

if __name__ == "__main__":
    main()
