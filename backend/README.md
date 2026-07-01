# ThermalCore Optimizer — Backend (Motor de Otimização)

Backend Python do ThermalCore Optimizer. Esta entrega (**PR 2**) traz o
**modelo térmico do Módulo 1** — o núcleo de física que dimensiona um dissipador
aletado. As camadas de otimização paramétrica e integração serial chegam nas PRs
seguintes.

## Estrutura

```
backend/
├── thermalcore/
│   ├── __init__.py
│   └── thermal_model.py   # circuito térmico, eficiência de aleta, T_chip
├── tests/
│   └── test_thermal_model.py
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

As funções de baixo nível aceitam escalares **ou** arrays NumPy, para permitir a
varredura paramétrica vetorizada do otimizador (PR seguinte). A função de alto
nível `evaluate(cond, geom, material)` retorna um `ThermalResult` com `t_chip`,
`r_tot`, eficiências, área de troca, espaçamento, massa e um flag de viabilidade.

## Uso

```python
from thermalcore.thermal_model import Conditions, HeatSinkGeometry, evaluate

cond = Conditions(q_c=20.0, t_max=85.0)          # requisitos do projeto
res = evaluate(cond, HeatSinkGeometry(n_fins=14, thickness=1.2e-3))
print(res.t_chip, res.mass, res.feasible)
```

Demonstração rápida:

```bash
cd backend
python3 -m thermalcore.thermal_model
```

## Testes

Rodam com a stdlib (não exigem pytest):

```bash
cd backend
python3 -m unittest discover -s tests -v
```

Os testes cobrem: valores fechados das fórmulas fundamentais, invariantes
físicos (mais aletas ⇒ menor temperatura e maior massa; maior `h` ⇒ menor
temperatura; ordenação `eta_f ≤ eta_o ≤ 1`), viabilidade geométrica, e uma
verificação ponta-a-ponta contra uma reimplementação independente em `math` puro.

## Dependências

- **NumPy** (ver `requirements.txt`).

```bash
pip install -r requirements.txt
```
