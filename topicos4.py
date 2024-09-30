import math
import tkinter as tk
from tkinter import ttk
import serial

# Configuração da porta serial
serial_port = 'COM6'  # Altere para a porta serial correta
baud_rate = 115200

# Função para formatar os valores para o protocolo
def enviar_serial():
    # Obtém a opção selecionada e os valores digitados
    opcao = opcao_var.get()
    valor1 = valor1_entry.get()
    valor2 = valor2_entry.get()
    valor3 = valor3_entry.get()

    try:
        # Converte os valores para float e depois para inteiro (valores entre 0000 e 9999)
        valor1 = float(valor1)  # Conversão para float
        valor2 = float(valor2)  # Conversão para float
        valor3 = float(valor3)  # Conversão para float

        int_opcao = format(opcoes.index(opcao), '04d')
        int_valor1 = format(int(((valor1 * (12 / 220) * math.sqrt(2)) * (10 / (47 + 10))) / (3.3 / (2 ** 12))),'04d')  # Multiplica para simular precisão decimal
        int_valor2 = format(int(((valor2 * (12 / 220) * math.sqrt(2)) * (10 / (47 + 10))) / (3.3 / (2 ** 12))), '04d')
        print(f"Val Min: {int_valor1},    Val Max: {int_valor2}")
        int_valor3 = format(int(valor3), '04d')

        # Formata a string conforme o protocolo
        protocolo = f"{int_opcao}-{int_valor1}-{int_valor2}-{int_valor3}"
        print(f"Enviando: {protocolo}")

        # Envia pela porta serial
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            ser.write(protocolo.encode())

    except ValueError:
        print("Erro: Digite valores válidos.")

# Interface gráfica usando tkinter
root = tk.Tk()
root.title("Envio Serial de Protocolo")

# Label e Lista de opções
opcao_var = tk.StringVar(value="AC > BAT > PANEL")  # Valor inicial
opcoes = ["AC > BAT > PANEL", "AC > PANEL > BAT", "BAT > AC > PANEL", "BAT > PANEL > AC", "PANEL > AC > BAT", "PANEL > BAT > AC"]

ttk.Label(root, text="Selecione uma Opção:").grid(column=0, row=0, padx=10, pady=10)
opcao_menu = ttk.OptionMenu(root, opcao_var, opcoes[0], *opcoes)
opcao_menu.grid(column=1, row=0, padx=10, pady=10)

# Campo de valor 1
ttk.Label(root, text="Valor Minimo AC (V):").grid(column=0, row=1, padx=10, pady=10)
valor1_entry = ttk.Entry(root)
valor1_entry.grid(column=1, row=1, padx=10, pady=10)

# Campo de valor 2
ttk.Label(root, text="Valor Maximo AC (V):").grid(column=0, row=2, padx=10, pady=10)
valor2_entry = ttk.Entry(root)
valor2_entry.grid(column=1, row=2, padx=10, pady=10)

# Campo de valor 3
ttk.Label(root, text="Valor Minimo Bateria (%):").grid(column=0, row=3, padx=10, pady=10)
valor3_entry = ttk.Entry(root)
valor3_entry.grid(column=1, row=3, padx=10, pady=10)

# Botão de envio
enviar_btn = ttk.Button(root, text="Enviar", command=enviar_serial)
enviar_btn.grid(column=1, row=4, padx=10, pady=20)

root.mainloop()
