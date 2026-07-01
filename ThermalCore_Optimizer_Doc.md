# ThermalCore Optimizer: Documentação Técnica e Produção

O **ThermalCore Optimizer** é um assistente físico-digital de design térmico para componentes eletrônicos. O equipamento une uma Interface Homem-Máquina (HMI) baseada em microcontrolador a um motor de otimização termodinâmica rodando em Python[cite: 5].

---

## 1. Arquitetura do Sistema (MVP)

*   **HMI (Hardware):** Arduino Uno + LCD Keypad Shield (D1 Robot).
    *   *Display:* LCD 16x2.
    *   *Inputs:* Botões em divisor de tensão no pino A0 (UP, DOWN, LEFT, RIGHT, SELECT).
    *   *I/O Disponível:* Pinos A1-A5 e portas digitais livres para futura expansão (ex: termopares MAX6675 para validação física).
*   **Motor de Otimização (Software):** Python 3 via `pyserial`.
    *   *Processamento:* NumPy (varredura de malhas paramétricas) e SciPy (resolução de equações não lineares)[cite: 5].
*   **Protocolo de Comunicação:** JSON via interface Serial (UART) a 9600 ou 115200 baud rate.

---

## 2. Modelagem Termodinâmica (Módulo 1: Otimizador de Microeletrônica)

O Módulo 1 dimensiona um dissipador de calor aletado para manter um chip eletrônico abaixo de sua temperatura crítica. A modelagem considera condução unidimensional e convecção em superfícies estendidas[cite: 8].

### 2.1. Circuito Térmico Global
A transferência de calor $q_c$ do chip para o fluido (ar) atravessa três resistências principais em série: a interface de contato, a condução na base e o conjunto de aletas[cite: 8].
A resistência total ($R_{tot}$) é dada por:
$$R_{tot} = \frac{R_{t,c}^{\prime\prime}}{A_{chip}} + \frac{L_b}{k A_{chip}} + R_{t,o}$$
Onde:
*   $R_{t,c}^{\prime\prime}$: Resistência térmica de contato da pasta térmica/solda ($m^2 \cdot K/W$)[cite: 8].
*   $L_b$: Espessura da base do dissipador.
*   $k$: Condutividade térmica do material (ex: Alumínio = 180 a 237 $W/m\cdot K$)[cite: 8].

A temperatura de operação do chip ($T_{chip}$) será:
$$T_{chip} = T_{\infty} + q_c \cdot R_{tot}$$

### 2.2. Desempenho do Arranjo de Aletas
A resistência equivalente do dissipador ($R_{t,o}$) depende da eficiência global da superfície ($\eta_o$)[cite: 8]:
$$R_{t,o} = \frac{1}{\eta_o h A_t}$$
A área total de troca térmica ($A_t$) e a eficiência global ($\eta_o$) são:
$$A_t = N A_f + A_b$$
$$\eta_o = 1 - \frac{N A_f}{A_t}(1 - \eta_f)$$
Onde $N$ é o número de aletas, $A_f$ é a área de uma aleta e $A_b$ é a área exposta da base[cite: 8].

### 2.3. Eficiência da Aleta Individual ($\eta_f$)
Assumindo aletas retangulares de seção uniforme e extremidade adiabática, a eficiência é[cite: 8]:
$$\eta_f = \frac{\tanh(m L_a)}{m L_a}$$
Sendo $L_a$ o comprimento da aleta e $m$ o parâmetro da aleta definido como[cite: 8]:
$$m = \sqrt{\frac{h P}{k A_c}}$$

---

## 3. Algoritmo de Otimização (Backend Python)

O backend não apenas resolve as equações, mas atua como um assistente de projeto (CAD térmico iterativo)[cite: 5]. O fluxo de execução é:

1.  **Recepção (Listener):** O script aguarda via porta Serial o JSON com os requisitos de projeto (Ex: `{"q_c": 20.0, "T_max": 85.0}`).
2.  **Varredura Paramétrica:** Utilizando NumPy, o script gera milhares de combinações de geometrias possíveis:
    *   Número de aletas ($N$): 5 a 40.
    *   Espessura da aleta ($t$): 0.5 mm a 5.0 mm.
3.  **Filtro de Viabilidade Térmica:** O script calcula o circuito térmico completo para cada combinação e descarta aquelas onde $T_{chip} > T_{max}$.
4.  **Função Custo (Otimização):** Dentre os arranjos viáveis, o software seleciona a geometria ideal baseando-se no critério de **menor massa** (minimização do volume de alumínio) ou **maior espaçamento ($S$)** para minimizar perda de carga.
5.  **Resposta:** Transmite o resultado ótimo de volta ao Arduino (Ex: `{"N": 14, "t": 1.2}`).

---

## 4. Perspectivas Futuras (Módulos 2 e 3)

Conforme a evolução do projeto, o sistema integrará:
*   **Módulo 2 (Cabeamento):** Determinação de temperatura e Raio Crítico de Isolamento sob geração volumétrica com convecção e radiação simultâneas, utilizando o método de Newton-Raphson no Python[cite: 5].
*   **Módulo 3 (Reatores Exotérmicos):** Solução de equações diferenciais de difusão de calor com condições assimétricas de contorno para localização do pico de temperatura ($dT/dx = 0$)[cite: 5].
