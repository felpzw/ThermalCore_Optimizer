"""Motor de otimização paramétrica do ThermalCore Optimizer (Módulo 1).

Implementa o fluxo de CAD térmico iterativo descrito na documentação:

1. **Varredura paramétrica** (NumPy): gera a malha de geometrias possíveis —
   número de aletas ``N`` (5 a 40) e espessura ``t`` (0.5 a 5.0 mm).
2. **Filtro de viabilidade térmica**: descarta arranjos com ``T_chip > T_max``
   (ou geometricamente inviáveis, ``N·t ≥ L_base``).
3. **Função custo**: dentre os viáveis, seleciona a geometria ótima pelo
   critério de **menor massa** (mínimo volume de alumínio) ou **maior
   espaçamento** ``S`` (menor perda de carga).
4. **Refino não linear** (SciPy, opcional): ajusta a espessura da geometria
   escolhida até a fronteira exata de viabilidade (``T_chip = T_max``),
   reduzindo massa/aumentando espaçamento além da resolução da malha.

A avaliação térmica reusa as funções vetorizadas de :mod:`thermalcore.thermal_model`,
de modo que a varredura inteira é calculada de uma vez (sem laços Python).

SciPy é usado apenas no passo de refino e é **opcional**: se não estiver
instalado, uma bisseção em Python puro assume o lugar (mesmo resultado).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .thermal_model import (
    ALUMINUM,
    Conditions,
    HeatSinkGeometry,
    Material,
    ThermalResult,
    array_resistance,
    chip_temperature,
    evaluate,
    fin_efficiency,
    fin_parameter,
    overall_efficiency,
    total_resistance,
)

try:  # SciPy é opcional (usado só no refino da fronteira).
    from scipy.optimize import brentq as _brentq

    HAVE_SCIPY = True
except ImportError:  # pragma: no cover - depende do ambiente
    _brentq = None
    HAVE_SCIPY = False

__all__ = [
    "SweepResult",
    "OptimizationResult",
    "CRITERIA",
    "sweep",
    "optimize",
    "boundary_thickness",
    "response_payload",
    "HAVE_SCIPY",
]

# Faixas padrão da varredura (documentação técnica).
N_RANGE_DEFAULT = (5, 40)                 # número de aletas (inteiros)
THICKNESS_RANGE_DEFAULT = (0.5e-3, 5.0e-3)  # espessura da aleta [m]
N_THICKNESS_DEFAULT = 91                   # amostras de espessura (~0.05 mm)

# Critérios de otimização suportados -> sentido da otimização.
CRITERIA = {"mass": "min", "spacing": "max"}


@dataclass(frozen=True)
class SweepResult:
    """Resultado bruto da varredura paramétrica (arrays achatados alinhados)."""

    n_fins: np.ndarray      # N de cada combinação (int)
    thickness: np.ndarray   # t de cada combinação [m]
    t_chip: np.ndarray      # temperatura do chip [°C]
    mass: np.ndarray        # massa do dissipador [kg]
    spacing: np.ndarray     # espaçamento entre aletas [m]
    feasible: np.ndarray    # máscara booleana de viabilidade

    @property
    def combinations(self) -> int:
        return int(self.n_fins.size)

    @property
    def feasible_count(self) -> int:
        return int(np.count_nonzero(self.feasible))


@dataclass(frozen=True)
class OptimizationResult:
    """Geometria ótima selecionada e metadados da busca."""

    criterion: str
    geometry: Optional[HeatSinkGeometry]
    thermal: Optional[ThermalResult]
    combinations: int
    feasible_count: int
    refined: bool  # True se a espessura foi refinada na fronteira (SciPy/bisseção)


# ---------------------------------------------------------------------------
# Varredura paramétrica vetorizada
# ---------------------------------------------------------------------------
def _evaluate_grid(cond: Conditions, material: Material,
                   n_grid: np.ndarray, t_grid: np.ndarray):
    """Avalia T_chip, massa, espaçamento e viabilidade sobre a malha (vetorizado)."""
    w = cond.l_base  # largura da aleta = profundidade da base

    fits = (n_grid * t_grid) < cond.l_base  # aletas cabem na base?

    with np.errstate(divide="ignore", invalid="ignore"):
        area_cross = t_grid * w
        perimeter = 2.0 * (w + t_grid)
        area_fin = 2.0 * w * cond.fin_height
        area_base = w * (cond.l_base - n_grid * t_grid)

        m = fin_parameter(cond.h, perimeter, material.k, area_cross)
        eta_fin = fin_efficiency(m, cond.fin_height)
        eta_overall, area_total = overall_efficiency(
            n_grid, area_fin, area_base, eta_fin)
        r_array = array_resistance(eta_overall, cond.h, area_total)
        r_tot = total_resistance(
            cond.r_contact, cond.a_chip, cond.l_b, material.k, r_array)
        t_chip = chip_temperature(cond.t_inf, cond.q_c, r_tot)

        spacing = (cond.l_base - n_grid * t_grid) / (n_grid - 1)

    mass = material.rho * (
        cond.l_base * cond.l_base * cond.l_b
        + n_grid * t_grid * w * cond.fin_height)

    feasible = fits & np.isfinite(t_chip) & (t_chip <= cond.t_max)
    return t_chip, mass, spacing, feasible


def sweep(cond: Conditions, material: Material = ALUMINUM, *,
          n_range=N_RANGE_DEFAULT,
          thickness_range=THICKNESS_RANGE_DEFAULT,
          n_thickness: int = N_THICKNESS_DEFAULT) -> SweepResult:
    """Gera e avalia a malha paramétrica de geometrias (passos 1–2 do fluxo)."""
    n_values = np.arange(n_range[0], n_range[1] + 1, dtype=int)
    t_values = np.linspace(thickness_range[0], thickness_range[1], n_thickness)
    n_grid, t_grid = np.meshgrid(n_values, t_values, indexing="ij")
    n_grid = n_grid.ravel()
    t_grid = t_grid.ravel()

    t_chip, mass, spacing, feasible = _evaluate_grid(cond, material, n_grid, t_grid)
    return SweepResult(
        n_fins=n_grid, thickness=t_grid, t_chip=t_chip,
        mass=mass, spacing=spacing, feasible=feasible)


# ---------------------------------------------------------------------------
# Refino não linear da fronteira de viabilidade (SciPy opcional)
# ---------------------------------------------------------------------------
def boundary_thickness(cond: Conditions, n_fins: int,
                       material: Material = ALUMINUM, *,
                       t_lo: float = THICKNESS_RANGE_DEFAULT[0],
                       t_hi: float = THICKNESS_RANGE_DEFAULT[1],
                       tol: float = 1e-7) -> Optional[float]:
    """Menor espessura viável para ``n_fins``: resolve ``T_chip(t) = T_max``.

    Resolve a equação não linear ``T_chip(t) - T_max = 0`` no intervalo
    ``[t_lo, t_hi]`` (aletas mais finas => menos eficientes => mais quentes).
    Usa ``scipy.optimize.brentq`` quando disponível; caso contrário, bisseção.

    Retorna:
      * ``t_lo`` se já é viável na espessura mínima;
      * a espessura de fronteira, se há troca de sinal;
      * ``None`` se nem a espessura máxima do intervalo é viável.
    """
    # Limita t_hi à faixa em que as aletas ainda cabem na base (N·t < L_base);
    # acima disso a geometria é inválida e T_chip(t) seria infinito.
    t_hi = min(t_hi, cond.l_base / n_fins * (1.0 - 1e-9))
    if t_lo >= t_hi:
        return None  # nem a espessura mínima cabe nesta contagem de aletas

    def excess(t):  # T_chip(t) - T_max ; > 0 significa inviável (quente demais)
        return evaluate(cond, HeatSinkGeometry(n_fins, float(t)), material).t_chip - cond.t_max

    f_lo, f_hi = excess(t_lo), excess(t_hi)
    if f_lo <= 0.0:
        return t_lo                 # já viável na espessura mínima
    if f_hi > 0.0:
        return None                 # inviável em toda a faixa
    if HAVE_SCIPY:
        return float(_brentq(excess, t_lo, t_hi, xtol=tol))
    # Fallback: bisseção (f_lo > 0, f_hi <= 0).
    a, b = t_lo, t_hi
    while b - a > tol:
        mid = 0.5 * (a + b)
        if excess(mid) > 0.0:
            a = mid
        else:
            b = mid
    return 0.5 * (a + b)


def _refine_thickness(cond, material, n_fins, t_grid, criterion):
    """Refina a espessura da geometria escolhida até a fronteira de viabilidade.

    Menor ``t`` reduz massa **e** aumenta espaçamento, então o refino beneficia
    ambos os critérios. Retorna ``(thickness, refined_bool)``; mantém a espessura
    da malha se o refino não produzir uma geometria viável.
    """
    t_star = boundary_thickness(cond, n_fins, material, t_lo=THICKNESS_RANGE_DEFAULT[0], t_hi=t_grid)
    if t_star is None or t_star >= t_grid:
        return t_grid, False
    # Garante viabilidade após arredondamento numérico da fronteira.
    for candidate in (t_star, t_star + 1e-6, t_star + 1e-5):
        if evaluate(cond, HeatSinkGeometry(n_fins, candidate), material).feasible:
            return candidate, True
    return t_grid, False


# ---------------------------------------------------------------------------
# Otimização de alto nível
# ---------------------------------------------------------------------------
def optimize(cond: Conditions, material: Material = ALUMINUM, *,
             criterion: str = "mass",
             n_range=N_RANGE_DEFAULT,
             thickness_range=THICKNESS_RANGE_DEFAULT,
             n_thickness: int = N_THICKNESS_DEFAULT,
             refine: bool = True) -> OptimizationResult:
    """Executa o fluxo completo e retorna a geometria ótima.

    ``criterion`` deve ser ``"mass"`` (menor massa) ou ``"spacing"`` (maior
    espaçamento). Levanta ``ValueError`` para critérios desconhecidos.
    """
    if criterion not in CRITERIA:
        raise ValueError(
            f"critério desconhecido: {criterion!r} (use {sorted(CRITERIA)})")

    result = sweep(cond, material, n_range=n_range,
                   thickness_range=thickness_range, n_thickness=n_thickness)

    if result.feasible_count == 0:
        return OptimizationResult(
            criterion=criterion, geometry=None, thermal=None,
            combinations=result.combinations, feasible_count=0, refined=False)

    idx = np.where(result.feasible)[0]
    if CRITERIA[criterion] == "min":
        best = idx[np.argmin(result.mass[idx])]
    else:  # "max"
        best = idx[np.argmax(result.spacing[idx])]

    best_n = int(result.n_fins[best])
    best_t = float(result.thickness[best])

    refined = False
    if refine:
        best_t, refined = _refine_thickness(cond, material, best_n, best_t, criterion)

    geom = HeatSinkGeometry(n_fins=best_n, thickness=best_t)
    thermal = evaluate(cond, geom, material)
    return OptimizationResult(
        criterion=criterion, geometry=geom, thermal=thermal,
        combinations=result.combinations, feasible_count=result.feasible_count,
        refined=refined)


def response_payload(result: OptimizationResult) -> dict:
    """Converte o resultado no payload de resposta ao Arduino: ``{"N": .., "t": ..}``.

    ``t`` é devolvido em **milímetros** (arredondado a 2 casas), como no
    protocolo da HMI. Retorna ``{"error": "infeasible"}`` se não há solução.
    """
    if result.geometry is None:
        return {"error": "infeasible"}
    return {
        "N": int(result.geometry.n_fins),
        "t": round(result.geometry.thickness * 1e3, 2),
    }


if __name__ == "__main__":  # demonstração rápida
    cond = Conditions(q_c=20.0, t_max=85.0)
    for crit in ("mass", "spacing"):
        res = optimize(cond, criterion=crit)
        g, th = res.geometry, res.thermal
        print(f"[{crit:7s}] combos={res.combinations} viáveis={res.feasible_count} "
              f"refinado={res.refined} -> N={g.n_fins} t={g.thickness*1e3:.3f}mm "
              f"T_chip={th.t_chip:.2f}C massa={th.mass*1e3:.1f}g S={th.spacing*1e3:.2f}mm "
              f"payload={response_payload(res)}")
