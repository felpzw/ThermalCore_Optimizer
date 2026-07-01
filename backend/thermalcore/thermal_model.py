"""Modelo térmico do ThermalCore Optimizer (Módulo 1).

Dimensiona um dissipador de calor aletado para manter um chip abaixo de sua
temperatura crítica, considerando condução unidimensional e convecção em
superfícies estendidas (aletas retangulares de seção uniforme, extremidade
adiabática).

As equações seguem a documentação técnica do projeto:

    R_tot   = R''_t,c / A_chip + L_b / (k A_chip) + R_t,o
    T_chip  = T_inf + q_c * R_tot
    R_t,o   = 1 / (eta_o * h * A_t)
    A_t     = N * A_f + A_b
    eta_o   = 1 - (N A_f / A_t) * (1 - eta_f)
    eta_f   = tanh(m L_a) / (m L_a)
    m       = sqrt(h P / (k A_c))

Convenções de geometria (base quadrada de lado ``L_base``; as aletas atravessam
toda a profundidade da base, portanto a largura da aleta ``w`` é igual a
``L_base``):

    A_c = t * w                (seção transversal da aleta)
    P   = 2 * (w + t)          (perímetro da aleta)
    A_f = 2 * w * L_a          (área lateral de uma aleta, ponta adiabática)
    A_b = w * (L_base - N * t) (área exposta da base entre aletas)

Todas as funções de baixo nível são compatíveis com escalares e arrays NumPy,
para permitir a varredura paramétrica vetorizada do otimizador (PR seguinte).

Unidades (SI): comprimentos em m, áreas em m², temperaturas em °C, potência em
W, k em W/(m·K), h em W/(m²·K), R''_t,c em m²·K/W, densidade em kg/m³.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = [
    "Material",
    "Conditions",
    "HeatSinkGeometry",
    "ThermalResult",
    "ALUMINUM",
    "fin_parameter",
    "fin_efficiency",
    "fin_lateral_area",
    "overall_efficiency",
    "array_resistance",
    "total_resistance",
    "chip_temperature",
    "evaluate",
]


# ---------------------------------------------------------------------------
# Parâmetros de entrada
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Material:
    """Material do dissipador."""

    k: float      # condutividade térmica [W/(m·K)]
    rho: float    # densidade [kg/m³]
    name: str = "custom"


# Alumínio típico (k na faixa 180–237 W/m·K citada na documentação).
ALUMINUM = Material(k=200.0, rho=2700.0, name="aluminum")


@dataclass(frozen=True)
class Conditions:
    """Requisitos de projeto e condições de contorno (fixos numa análise)."""

    q_c: float        # dissipação térmica do chip [W]
    t_max: float      # temperatura crítica do chip [°C]
    t_inf: float = 25.0     # temperatura do fluido (ar) [°C]
    h: float = 40.0         # coeficiente de convecção [W/(m²·K)]
    r_contact: float = 1.0e-4   # R''_t,c — resistência de contato [m²·K/W]
    a_chip: float = 400e-6      # área do chip / base de contato [m²] (ex.: 20x20 mm)
    l_base: float = 0.05        # lado da base quadrada do dissipador [m]
    l_b: float = 0.003          # espessura da base [m]
    fin_height: float = 0.025   # comprimento (altura) da aleta L_a [m]


@dataclass(frozen=True)
class HeatSinkGeometry:
    """Geometria variável do arranjo de aletas (varrida na otimização)."""

    n_fins: int       # número de aletas N
    thickness: float  # espessura da aleta t [m]


@dataclass(frozen=True)
class ThermalResult:
    """Resultado completo da avaliação térmica de uma geometria."""

    t_chip: float       # temperatura de operação do chip [°C]
    r_tot: float        # resistência térmica total [K/W]
    r_array: float      # resistência do arranjo de aletas R_t,o [K/W]
    eta_fin: float      # eficiência da aleta individual
    eta_overall: float  # eficiência global da superfície
    area_total: float   # área total de troca A_t [m²]
    spacing: float      # espaçamento entre aletas S [m]
    mass: float         # massa do dissipador [kg]
    feasible: bool      # geometria fisicamente válida e t_chip <= t_max


# ---------------------------------------------------------------------------
# Funções fundamentais (compatíveis com escalar e array NumPy)
# ---------------------------------------------------------------------------
def fin_parameter(h, perimeter, k, area_cross):
    """Parâmetro da aleta ``m = sqrt(h P / (k A_c))`` [1/m]."""
    return np.sqrt(h * perimeter / (k * area_cross))


def fin_efficiency(m, fin_height):
    """Eficiência da aleta ``eta_f = tanh(m L_a) / (m L_a)``.

    No limite ``m L_a -> 0`` a eficiência tende a 1 (tratado explicitamente
    para evitar divisão por zero).
    """
    mL = np.asarray(m) * fin_height
    # tanh(x)/x -> 1 quando x -> 0
    return np.where(mL > 0.0, np.tanh(mL) / np.where(mL > 0.0, mL, 1.0), 1.0)


def fin_lateral_area(width, fin_height):
    """Área lateral de convecção de uma aleta (ponta adiabática) ``A_f = 2 w L_a``."""
    return 2.0 * width * fin_height


def overall_efficiency(n_fins, area_fin, area_base, eta_fin):
    """Eficiência global ``eta_o`` e área total ``A_t = N A_f + A_b``.

    Retorna a tupla ``(eta_o, A_t)``.
    """
    area_total = n_fins * area_fin + area_base
    eta_o = 1.0 - (n_fins * area_fin / area_total) * (1.0 - eta_fin)
    return eta_o, area_total


def array_resistance(eta_overall, h, area_total):
    """Resistência do arranjo de aletas ``R_t,o = 1 / (eta_o h A_t)`` [K/W]."""
    return 1.0 / (eta_overall * h * area_total)


def total_resistance(r_contact, a_chip, l_b, k, r_array):
    """Resistência térmica total em série ``R_tot`` [K/W].

    ``R_tot = R''_t,c / A_chip + L_b / (k A_chip) + R_t,o``
    """
    r_interface = r_contact / a_chip
    r_conduction = l_b / (k * a_chip)
    return r_interface + r_conduction + r_array


def chip_temperature(t_inf, q_c, r_tot):
    """Temperatura de operação do chip ``T_chip = T_inf + q_c R_tot`` [°C]."""
    return t_inf + q_c * r_tot


# ---------------------------------------------------------------------------
# Avaliação de alto nível
# ---------------------------------------------------------------------------
def _spacing(l_base, n_fins, thickness):
    """Espaçamento (gap) entre aletas adjacentes ``S`` [m]."""
    if n_fins <= 1:
        return float(l_base - n_fins * thickness)
    return float((l_base - n_fins * thickness) / (n_fins - 1))


def _mass(material, cond, geom):
    """Massa do dissipador (base + aletas) [kg]."""
    width = cond.l_base  # aletas atravessam toda a profundidade da base
    v_base = cond.l_base * cond.l_base * cond.l_b
    v_fins = geom.n_fins * geom.thickness * width * cond.fin_height
    return material.rho * (v_base + v_fins)


def evaluate(cond: Conditions, geom: HeatSinkGeometry,
             material: Material = ALUMINUM) -> ThermalResult:
    """Avalia uma geometria de dissipador para as condições dadas.

    Calcula o circuito térmico completo e retorna um :class:`ThermalResult`.
    A geometria é marcada como ``feasible=False`` (e ``t_chip`` como infinito)
    quando as aletas não cabem na base (``N * t >= L_base``), pois nesse caso
    não há base exposta nem espaçamento físico válido.
    """
    width = cond.l_base  # largura da aleta = profundidade da base

    # Viabilidade geométrica: as aletas precisam caber na base.
    if geom.n_fins < 1 or geom.thickness <= 0.0 or geom.n_fins * geom.thickness >= cond.l_base:
        return ThermalResult(
            t_chip=float("inf"), r_tot=float("inf"), r_array=float("inf"),
            eta_fin=float("nan"), eta_overall=float("nan"),
            area_total=float("nan"), spacing=float("nan"),
            mass=_mass(material, cond, geom), feasible=False,
        )

    area_cross = geom.thickness * width
    perimeter = 2.0 * (width + geom.thickness)
    area_fin = fin_lateral_area(width, cond.fin_height)
    area_base = width * (cond.l_base - geom.n_fins * geom.thickness)

    m = fin_parameter(cond.h, perimeter, material.k, area_cross)
    eta_fin = float(fin_efficiency(m, cond.fin_height))
    eta_overall, area_total = overall_efficiency(
        geom.n_fins, area_fin, area_base, eta_fin)
    r_array = array_resistance(eta_overall, cond.h, area_total)
    r_tot = total_resistance(
        cond.r_contact, cond.a_chip, cond.l_b, material.k, r_array)
    t_chip = chip_temperature(cond.t_inf, cond.q_c, r_tot)

    return ThermalResult(
        t_chip=float(t_chip),
        r_tot=float(r_tot),
        r_array=float(r_array),
        eta_fin=eta_fin,
        eta_overall=float(eta_overall),
        area_total=float(area_total),
        spacing=_spacing(cond.l_base, geom.n_fins, geom.thickness),
        mass=_mass(material, cond, geom),
        feasible=bool(t_chip <= cond.t_max),
    )


if __name__ == "__main__":  # demonstração rápida (sanidade manual)
    cond = Conditions(q_c=20.0, t_max=85.0)
    for n in (5, 10, 14, 20, 30):
        res = evaluate(cond, HeatSinkGeometry(n_fins=n, thickness=1.2e-3))
        print(f"N={n:2d}  T_chip={res.t_chip:6.2f} C  "
              f"eta_f={res.eta_fin:.3f}  eta_o={res.eta_overall:.3f}  "
              f"R_tot={res.r_tot:.3f} K/W  massa={res.mass*1e3:6.1f} g  "
              f"{'OK' if res.feasible else 'X'}")
