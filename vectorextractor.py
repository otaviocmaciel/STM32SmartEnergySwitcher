import serial
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import find_peaks
from scipy.fftpack import fft
import numpy as np

# Configurações da UART
serial_port = 'COM6'  # Ajuste conforme necessário
baud_rate = 230400
timeout = 1

# Inicializa a comunicação serial
ser = serial.Serial(serial_port, baud_rate, timeout=timeout)

# Buffer para armazenar os dados do ADC
adc_values = []

def send_command():
    while True:
        ser.write(b'#B.')
        time.sleep(10)

def read_data():
    global adc_values
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    value = int(line)
                    adc_values.append(value)
                    if len(adc_values) >= 1000:
                        adc_values = adc_values[-1000:]  # Mantém apenas os últimos 2000 valores
                except ValueError:
                    pass

def update_plot(frame):
    plt.clf()
    if len(adc_values) == 1000:
        # Subtrai a média para remover o offset DC
        centered_values = adc_values - np.mean(adc_values)

        # Plotagem dos valores do ADC
        plt.subplot(2, 1, 1)
        plt.plot(adc_values, label='ADC Values')
        plt.xlabel('Amostras')
        plt.ylabel('Valor ADC')
        plt.title('Sinal Capturado do ADC')
        plt.legend()

        # Calcular a FFT
        N = 2000
        T = 1.0 / 120000.0  # 120 kHz de taxa de amostragem
        yf = fft(centered_values)
        xf = np.fft.fftfreq(N, T)[:N//2]

        # Limitar o eixo X a 60 Hz - 500 Hz
        min_limit = 60
        max_limit = 1000
        indices = np.where((xf >= min_limit) & (xf <= max_limit))
        xf = xf[indices]
        yf = yf[indices]

        # Identificar picos
        peaks, _ = find_peaks(2.0/N * np.abs(yf), height=0)

        # Plotagem da FFT
        plt.subplot(2, 1, 2)
        plt.plot(xf, 2.0/N * np.abs(yf), label='FFT')
        plt.plot(xf[peaks], 2.0/N * np.abs(yf)[peaks], "x")  # Marca os picos com 'x'
        plt.xlabel('Frequência (Hz)')
        plt.ylabel('Amplitude')
        plt.title('Espectro de Frequência')
        plt.legend()
        plt.ylim(0, np.max(2.0/N * np.abs(yf)) * 1.1)  # Ajusta o eixo Y para melhor visualização

# Thread para enviar comandos
command_thread = threading.Thread(target=send_command)
command_thread.daemon = True
command_thread.start()

# Thread para ler dados
read_thread = threading.Thread(target=read_data)
read_thread.daemon = True
read_thread.start()

# Configuração da animação do matplotlib
fig = plt.figure()
ani = FuncAnimation(fig, update_plot, interval=5000)

plt.show()
