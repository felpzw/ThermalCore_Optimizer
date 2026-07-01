# ThermalCore Optimizer — Backend (Motor de Otimização)

Backend Python do ThermalCore Optimizer — o motor de otimização termodinâmica do
Módulo 1. Compõe-se do **modelo térmico** (núcleo de física) e do **otimizador
paramétrico** (varredura + filtro + função custo). A integração serial com a HMI
chega na PR seguinte.

## Estrutura

```
backend/
├── thermalcore/
│   ├── __init__.py
│   ├── thermal_model.py   # circuito térmico, eficiência de aleta, T_chip
│   └── optimizer.py       # varredura NumPy, filtro de viabilidade, função custo
├── tests/
│   ├── test_thermal_model.py
│   └── test_optimizer.py
├── requirements.txt
└── README.md
```

## Modelo térmico

`thermal_model.py` implementa o circuito térmico em série do chip para o ar,
seguindo a documentação técnica do projeto:

| Grandeza | Fórmula |
|----------|---------|
| Parâmetro da aleta | `m = sqrt(h P / (k A_c))` |
| Eficiência da aleta | `eta_f = tanh(m L_a) / (m L_a)` |
| Eficiência global | `eta_o = 1 - (N A_f / A_t)(1 - eta_f)` |
| Resistência do arranjo | `R_t,o = 1 / (eta_o h A_t)` |
| Resistência total | `R_tot = R''_t,c / A_chip + L_b / (k A_chip) + R_t,o` |
| Temperatura do chip | `T_chip = T_inf + q_c R_tot` |

As funções de baixo nível aceitam escalares **ou** arrays NumPy, permitindo a
varredura paramétrica vetorizada do otimizador. A função de alto nível
`evaluate(cond, geom, material)` retorna um `ThermalResult` com `t_chip`,
`r_tot`, eficiências, área de troca, espaçamento, massa e um flag de viabilidade.

## Otimizador paramétrico

`optimizer.py` implementa o fluxo de CAD térmico iterativo da documentação:

1. **Varredura paramétrica** (NumPy): malha de geometrias — `N` de 5 a 40 e
   espessura `t` de 0.5 a 5.0 mm (~3300 combinações), avaliada de uma vez via as
   funções vetorizadas do modelo.
2. **Filtro de viabilidade**: descarta `T_chip > T_max` e geometrias inviáveis
   (`N·t ≥ L_base`).
3. **Função custo**: seleciona a ótima por **menor massa** (`criterion="mass"`)
   ou **maior espaçamento** (`criterion="spacing"`).
4. **Refino não linear** (SciPy, opcional): `boundary_thickness` resolve
   `T_chip(t) = T_max` para ajustar a espessura à fronteira exata de viabilidade
   (`scipy.optimize.brentq`; sem SciPy, cai numa bisseção pura equivalente).

## Uso

```python
from thermalcore.thermal_model import Conditions, HeatSinkGeometry, evaluate
from thermalcore.optimizer import optimize, response_payload

cond = Conditions(q_c=20.0, t_max=85.0)          # requisitos do projeto

# avaliação de uma geometria específica
res = evaluate(cond, HeatSinkGeometry(n_fins=14, thickness=1.2e-3))
print(res.t_chip, res.mass, res.feasible)

# otimização (menor massa) -> payload {"N": .., "t": .. mm} para a HMI
opt = optimize(cond, criterion="mass")
print(response_payload(opt))
```

Demonstrações rápidas:

```bash
cd backend
python3 -m thermalcore.thermal_model   # modelo: varre N e imprime T_chip/massa
python3 -m thermalcore.optimizer       # otimizador: melhor geometria por critério
```

## Testes

Rodam com a stdlib (não exigem pytest):

```bash
cd backend
python3 -m unittest discover -s tests -v
```

Os testes cobrem o **modelo** (fórmulas fundamentais com valores fechados,
invariantes físicos, viabilidade geométrica, verificação ponta-a-ponta contra
reimplementação independente em `math` puro) e o **otimizador** (contagem da
malha, concordância varredura vetorizada × `evaluate` escalar, seleção por
custo, casos inviáveis, e o refino da fronteira contra uma bisseção
independente). Rodam com ou sem SciPy instalado.

## Dependências

- **NumPy** — varredura paramétrica (obrigatória).
- **SciPy** — refino não linear da fronteira (opcional; há fallback puro).

```bash
pip install -r requirements.txt
```
