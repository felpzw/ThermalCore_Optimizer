"""Testes do motor de otimização paramétrica (Módulo 1).

Executável por ``python3 -m unittest`` ou ``pytest``. Não exige SciPy: o refino
da fronteira cai no fallback de bisseção quando SciPy está ausente, e um teste
compara o valor obtido contra uma bisseção independente escrita aqui.
"""

import unittest

import numpy as np

from thermalcore.thermal_model import Conditions, HeatSinkGeometry, evaluate
from thermalcore.optimizer import (
    N_RANGE_DEFAULT,
    N_THICKNESS_DEFAULT,
    boundary_thickness,
    optimize,
    response_payload,
    sweep,
)

EASY = Conditions(q_c=20.0, t_max=85.0)               # ponto fino já viável
DEMANDING = Conditions(q_c=45.0, t_max=70.0, h=30.0)  # refino dispara; critérios divergem
IMPOSSIBLE = Conditions(q_c=80.0, t_max=75.0, h=25.0)  # nenhum arranjo viável


class TestSweep(unittest.TestCase):
    def test_combinations_count(self):
        res = sweep(EASY)
        n_expected = (N_RANGE_DEFAULT[1] - N_RANGE_DEFAULT[0] + 1) * N_THICKNESS_DEFAULT
        self.assertEqual(res.combinations, n_expected)
        self.assertEqual(res.n_fins.size, res.thickness.size)

    def test_grid_matches_scalar_evaluate(self):
        # A varredura vetorizada deve concordar com evaluate() ponto a ponto.
        res = sweep(DEMANDING)
        feas_idx = np.where(res.feasible)[0]
        # amostra espalhada de pontos viáveis
        for i in feas_idx[:: max(1, len(feas_idx) // 20)]:
            geom = HeatSinkGeometry(int(res.n_fins[i]), float(res.thickness[i]))
            scalar = evaluate(DEMANDING, geom)
            self.assertAlmostEqual(res.t_chip[i], scalar.t_chip, places=6)
            self.assertAlmostEqual(res.mass[i], scalar.mass, places=9)
            self.assertAlmostEqual(res.spacing[i], scalar.spacing, places=9)

    def test_feasible_points_respect_tmax_and_geometry(self):
        res = sweep(DEMANDING)
        # Todo ponto viável: cabe na base e T_chip <= T_max.
        fits = res.n_fins * res.thickness < DEMANDING.l_base
        self.assertTrue(np.all(fits[res.feasible]))
        self.assertTrue(np.all(res.t_chip[res.feasible] <= DEMANDING.t_max + 1e-9))


class TestOptimize(unittest.TestCase):
    def test_mass_is_global_min_over_feasible(self):
        # Sem refino, o resultado deve ser exatamente o mínimo da malha.
        res = optimize(DEMANDING, criterion="mass", refine=False)
        sw = sweep(DEMANDING)
        idx = np.where(sw.feasible)[0]
        best = idx[np.argmin(sw.mass[idx])]
        self.assertEqual(res.geometry.n_fins, int(sw.n_fins[best]))
        self.assertAlmostEqual(res.geometry.thickness, float(sw.thickness[best]))

    def test_spacing_is_global_max_over_feasible(self):
        res = optimize(DEMANDING, criterion="spacing", refine=False)
        sw = sweep(DEMANDING)
        idx = np.where(sw.feasible)[0]
        best = idx[np.argmax(sw.spacing[idx])]
        self.assertEqual(res.geometry.n_fins, int(sw.n_fins[best]))
        self.assertAlmostEqual(res.geometry.thickness, float(sw.thickness[best]))

    def test_unknown_criterion_raises(self):
        with self.assertRaises(ValueError):
            optimize(EASY, criterion="volume")

    def test_infeasible_returns_none(self):
        res = optimize(IMPOSSIBLE, criterion="mass")
        self.assertIsNone(res.geometry)
        self.assertIsNone(res.thermal)
        self.assertEqual(res.feasible_count, 0)
        self.assertEqual(response_payload(res), {"error": "infeasible"})

    def test_result_thermal_is_consistent_with_geometry(self):
        res = optimize(DEMANDING, criterion="mass")
        recomputed = evaluate(DEMANDING, res.geometry)
        self.assertAlmostEqual(res.thermal.t_chip, recomputed.t_chip, places=9)

    def test_optimum_is_feasible(self):
        for crit in ("mass", "spacing"):
            res = optimize(DEMANDING, criterion=crit)
            self.assertTrue(res.thermal.feasible)
            self.assertLessEqual(res.thermal.t_chip, DEMANDING.t_max + 1e-6)

    def test_refine_reduces_thickness_toward_boundary(self):
        # No caso exigente, o critério 'spacing' refina t até a fronteira.
        coarse = optimize(DEMANDING, criterion="spacing", refine=False)
        fine = optimize(DEMANDING, criterion="spacing", refine=True)
        self.assertTrue(fine.refined)
        self.assertLessEqual(fine.geometry.thickness, coarse.geometry.thickness + 1e-12)
        # Refino leva T_chip para junto de T_max (fronteira de viabilidade).
        self.assertAlmostEqual(fine.thermal.t_chip, DEMANDING.t_max, delta=0.5)
        self.assertTrue(fine.thermal.feasible)


class TestBoundaryThickness(unittest.TestCase):
    def test_returns_tlo_when_thin_end_feasible(self):
        # Caso fácil: N=5 já é viável na espessura mínima.
        t_star = boundary_thickness(EASY, n_fins=5)
        self.assertAlmostEqual(t_star, 0.5e-3)

    def test_none_when_whole_range_infeasible(self):
        self.assertIsNone(boundary_thickness(IMPOSSIBLE, n_fins=20))

    def test_interior_boundary_hits_tmax(self):
        # N=20 no caso exigente exige t > 0.5 mm; na fronteira T_chip == T_max.
        t_star = boundary_thickness(DEMANDING, n_fins=20)
        self.assertIsNotNone(t_star)
        self.assertGreater(t_star, 0.5e-3)
        t_chip = evaluate(DEMANDING, HeatSinkGeometry(20, t_star)).t_chip
        self.assertAlmostEqual(t_chip, DEMANDING.t_max, delta=1e-3)

    def test_matches_independent_bisection(self):
        # Verificação independente do solver ativo (SciPy ou fallback).
        n = 20
        lo, hi = 0.5e-3, 2.0e-3  # hi dentro do limite geométrico (N·t < L_base)

        def excess(t):
            return evaluate(DEMANDING, HeatSinkGeometry(n, t)).t_chip - DEMANDING.t_max

        a, b = lo, hi
        for _ in range(80):  # bisseção independente
            mid = 0.5 * (a + b)
            if excess(mid) > 0.0:
                a = mid
            else:
                b = mid
        ref = 0.5 * (a + b)
        self.assertAlmostEqual(boundary_thickness(DEMANDING, n_fins=n), ref, places=6)


class TestResponsePayload(unittest.TestCase):
    def test_payload_format(self):
        res = optimize(EASY, criterion="mass")
        payload = response_payload(res)
        self.assertIsInstance(payload["N"], int)
        self.assertIsInstance(payload["t"], float)
        # t em mm, 2 casas
        self.assertAlmostEqual(payload["t"], round(res.geometry.thickness * 1e3, 2))


if __name__ == "__main__":
    unittest.main()
