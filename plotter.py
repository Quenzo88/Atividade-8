import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections

# Configurações - AJUSTE A PORTA SERIAL DE ACORDO COM O SEU SISTEMA
COM_PORT = 'COM6'  # Windows (ex: COM3, COM4) ou '/dev/ttyACM0' no Linux
BAUD_RATE = 115200

# Buffers de exibição (Últimas 100 amostras)
SAMPLES_DISPLAY = 100
raw_x_buffer = collections.deque(maxlen=SAMPLES_DISPLAY)
filt_x_buffer = collections.deque(maxlen=SAMPLES_DISPLAY)

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    print(f"Conectado com sucesso na porta {COM_PORT}!")
except Exception as e:
    print(f"Erro ao abrir a porta serial {COM_PORT}: {e}")
    exit()

# Configuração da janela do Matplotlib
fig, ax = plt.subplots()
line_raw, = ax.plot([], [], label='Eixo X Original (Raw)', color='blue', alpha=0.4)
line_filt, = ax.plot([], [], label='Eixo X Filtrado (FIR)', color='red', linewidth=2)
ax.set_title("Acelerômetro MMA8451Q em Tempo Real - PSI3441")
ax.set_xlabel("Amostras")
ax.set_ylabel("Aceleração (mili-m/s²)")
ax.legend(loc='upper right')
ax.grid(True)

def update(frame):
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            # Identifica o padrão enviado pelo firmware Zephyr
            if "ACEL," in line:
                payload = line.split("ACEL,")[1]
                parts = payload.split(",")
                if len(parts) >= 5:
                    # Estrutura: ts, raw_x, filt_x, raw_y, raw_z
                    ts = int(parts[0])
                    raw_x = int(parts[1])
                    filt_x = int(parts[2])
                    
                    raw_x_buffer.append(raw_x)
                    filt_x_buffer.append(filt_x)
        except Exception:
            pass

    # Atualiza as linhas do gráfico
    line_raw.set_data(range(len(raw_x_buffer)), list(raw_x_buffer))
    line_filt.set_data(range(len(filt_x_buffer)), list(filt_x_buffer))
    
    # Ajusta os limites dos eixos dinamicamente
    if len(raw_x_buffer) > 0:
        ax.set_xlim(0, SAMPLES_DISPLAY)
        ax.set_ylim(min(raw_x_buffer) - 500, max(raw_x_buffer) + 500)
        
    return line_raw, line_filt

# Inicia a animação (atualiza o gráfico a cada 30ms)
ani = animation.FuncAnimation(fig, update, interval=30, blit=False)
plt.show()

ser.close()
print("Porta serial fechada.")