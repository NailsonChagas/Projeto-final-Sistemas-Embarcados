import numpy as np
import matplotlib.pyplot as plt

def salvar_array_c(nome_arquivo, array, nome_variavel="data"):
    """
    Salva um array numpy no formato C (array de floats)
    """
    with open(nome_arquivo, 'w') as f:
        f.write(f"// Array: {nome_variavel}\n")
        f.write(f"// Tamanho: {len(array)}\n")
        f.write(f"const float {nome_variavel}[{len(array)}] = {{\n")
        
        for i in range(0, len(array), 5):
            linha = array[i:i+5]
            valores_formatados = [f"{val:.6f}f" for val in linha]
            f.write("    " + ", ".join(valores_formatados))
            if i + 5 < len(array):
                f.write(",\n")
            else:
                f.write("\n")
        
        f.write("};\n")

freq = 100
amplitude = 0.5
offset = amplitude    

fs = 20 * (10**3)
duracao = 1
t = np.linspace(0, duracao / freq, int(fs / freq), endpoint=False)

print(f"Gerando sinais com frequência: {freq} Hz")
print(f"Amplitude: {amplitude}")
print(f"Número de amostras por período: {len(t)}")

# Sinais antes do offset
sinal_seno = amplitude * np.sin(2 * np.pi * freq * t)
sinal_quadrado = amplitude * np.sign(np.sin(2 * np.pi * freq * t))
sinal_triangular = (2 * amplitude / np.pi) * np.arcsin(np.sin(2 * np.pi * freq * t))
sinal_seno_retificado = amplitude * np.abs(np.sin(2 * np.pi * freq * t))

# ===== APLICANDO OFFSET =====
sinal_seno += offset
sinal_quadrado += offset
sinal_triangular += offset
sinal_seno_retificado += offset

# Salvando
salvar_array_c('sinal_seno_c.txt', sinal_seno, "sinal_seno")
salvar_array_c('sinal_quadrado_c.txt', sinal_quadrado, "sinal_quadrado")
salvar_array_c('sinal_triangular_c.txt', sinal_triangular, "sinal_triangular")
salvar_array_c('sinal_seno_retificado_c.txt', sinal_seno_retificado, "sinal_seno_retificado")

# Plotagem
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle(f'Sinais com Frequência = {freq} Hz, Amplitude = {amplitude}, Offset = {offset}', fontsize=16)

axes[0,0].plot(t, sinal_seno, linewidth=2)
axes[0,0].set_title('Sinal Senoidal')
axes[0,0].grid(True, alpha=0.3)

axes[0,1].plot(t, sinal_quadrado, linewidth=2)
axes[0,1].set_title('Sinal Quadrado')
axes[0,1].grid(True, alpha=0.3)

axes[1,0].plot(t, sinal_triangular, linewidth=2)
axes[1,0].set_title('Sinal Triangular')
axes[1,0].grid(True, alpha=0.3)

axes[1,1].plot(t, sinal_seno_retificado, linewidth=2)
axes[1,1].set_title('Seno Retificado')
axes[1,1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('verificacao_sinais.png', dpi=150, bbox_inches='tight')
plt.show()

print("\nInformações dos vetores gerados:")
print("=" * 60)
print(f"{'Sinal':15} | {'Tamanho':8} | {'Min':10} | {'Max':10}")
print("-" * 60)
print(f"{'Seno':15} | {len(sinal_seno):8} | {sinal_seno.min():10.6f} | {sinal_seno.max():10.6f}")
print(f"{'Quadrado':15} | {len(sinal_quadrado):8} | {sinal_quadrado.min():10.6f} | {sinal_quadrado.max():10.6f}")
print(f"{'Triangular':15} | {len(sinal_triangular):8} | {sinal_triangular.min():10.6f} | {sinal_triangular.max():10.6f}")
print(f"{'Seno Retif.':15} | {len(sinal_seno_retificado):8} | {sinal_seno_retificado.min():10.6f} | {sinal_seno_retificado.max():10.6f}")
