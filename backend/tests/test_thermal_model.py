"""Testes do modelo térmico (Módulo 1).

Executável tanto por ``python3 -m unittest`` quanto por ``pytest``.

Estratégia: além de invariantes físicos, a cadeia completa é recomputada de
forma independente com ``math`` puro (caminho de código distinto do módulo, que
usa NumPy) e comparada ao resultado de :func:`evaluate`.
"""

import math
import unittest

from thermalcore.thermal_model import (
    ALUMINUM,
    Conditions,
    HeatSinkGeometry,
    array_resistance,
    chip_temperature,
    evaluate,
    fin_efficiency,
    fin_parameter,
    overall_efficiency,
    total_resistance,
)


def _reference_evaluate(cond, geom, material):
    """Recomputa T_chip com math puro — implementação independente de referência."""
    w = cond.l_base
    a_c = geom.thickness * w
    perim = 2.0 * (w + geom.thickness)
    a_f = 2.0 * w * cond.fin_height
    a_b = w * (cond.l_base - geom.n_fins * geom.thickness)

    m = math.sqrt(cond.h * perim / (material.k * a_c))
    mL = m * cond.fin_height
    eta_f = math.tanh(mL) / mL
    a_t = geom.n_fins * a_f + a_b
    eta_o = 1.0 - (geom.n_fins * a_f / a_t) * (1.0 - eta_f)
    r_to = 1.0 / (eta_o * cond.h * a_t)
    r_tot = cond.r_contact / cond.a_chip + cond.l_b / (material.k * cond.a_chip) + r_to
    return cond.t_inf + cond.q_c * r_tot, r_tot, eta_f, eta_o


class TestFundamentalFunctions(unittest.TestCase):
    def test_fin_parameter_hand_value(self):
        # h=40, P=0.1024, k=200, A_c=6e-5 -> m = sqrt(4.096/0.012)
        m = fin_parameter(40.0, 0.1024, 200.0, 6.0e-5)
        self.assertAlmostEqual(float(m), math.sqrt(4.096 / 0.012), places=6)

    def test_fin_efficiency_known_value(self):
        # m*L_a = 1  ->  tanh(1)/1
        self.assertAlmostEqual(float(fin_efficiency(1.0, 1.0)), math.tanh(1.0), places=9)

    def test_fin_efficiency_limit_short_fin(self):
        # m*L_a -> 0  =>  eta_f -> 1
        self.assertAlmostEqual(float(fin_efficiency(1e-9, 1e-3)), 1.0, places=9)

    def test_fin_efficiency_between_zero_and_one(self):
        for mL in (0.1, 0.5, 1.0, 3.0, 10.0):
            eff = float(fin_efficiency(mL, 1.0))
            self.assertGreater(eff, 0.0)
            self.assertLessEqual(eff, 1.0)

    def test_overall_efficiency_bounds(self):
        # eta_o deve ficar entre eta_f e 1 (aletas nunca pioram a base).
        eta_f = 0.8
        eta_o, a_t = overall_efficiency(10, 2.0, 5.0, eta_f)
        self.assertAlmostEqual(a_t, 25.0)
        self.assertGreaterEqual(eta_o, eta_f)
        self.assertLessEqual(eta_o, 1.0)

    def test_array_resistance(self):
        self.assertAlmostEqual(array_resistance(0.5, 40.0, 0.1), 1.0 / (0.5 * 40.0 * 0.1))

    def test_total_resistance_components(self):
        # R_tot = R''/A + L_b/(k A) + R_to
        r = total_resistance(1e-4, 4e-4, 3e-3, 200.0, 1.0)
        expected = 1e-4 / 4e-4 + 3e-3 / (200.0 * 4e-4) + 1.0
        self.assertAlmostEqual(r, expected, places=9)

    def test_chip_temperature(self):
        self.assertAlmostEqual(chip_temperature(25.0, 20.0, 2.0), 65.0)


class TestEvaluate(unittest.TestCase):
    def setUp(self):
        self.cond = Conditions(q_c=20.0, t_max=85.0)

    def test_matches_independent_reference(self):
        for n in (5, 10, 14, 20, 30):
            geom = HeatSinkGeometry(n_fins=n, thickness=1.2e-3)
            res = evaluate(self.cond, geom, ALUMINUM)
            t_ref, r_ref, ef_ref, eo_ref = _reference_evaluate(self.cond, geom, ALUMINUM)
            self.assertAlmostEqual(res.t_chip, t_ref, places=6, msg=f"N={n}")
            self.assertAlmostEqual(res.r_tot, r_ref, places=6, msg=f"N={n}")
            self.assertAlmostEqual(res.eta_fin, ef_ref, places=6, msg=f"N={n}")
            self.assertAlmostEqual(res.eta_overall, eo_ref, places=6, msg=f"N={n}")

    def test_more_fins_lowers_temperature(self):
        temps = [evaluate(self.cond, HeatSinkGeometry(n, 1.2e-3)).t_chip
                 for n in (5, 10, 20, 30)]
        for a, b in zip(temps, temps[1:]):
            self.assertGreater(a, b)

    def test_more_fins_increases_mass(self):
        masses = [evaluate(self.cond, HeatSinkGeometry(n, 1.2e-3)).mass
                  for n in (5, 10, 20, 30)]
        for a, b in zip(masses, masses[1:]):
            self.assertLess(a, b)

    def test_higher_convection_lowers_temperature(self):
        geom = HeatSinkGeometry(14, 1.2e-3)
        low_h = evaluate(Conditions(q_c=20.0, t_max=85.0, h=20.0), geom).t_chip
        high_h = evaluate(Conditions(q_c=20.0, t_max=85.0, h=80.0), geom).t_chip
        self.assertGreater(low_h, high_h)

    def test_efficiency_ordering(self):
        res = evaluate(self.cond, HeatSinkGeometry(14, 1.2e-3))
        self.assertLessEqual(res.eta_fin, res.eta_overall)
        self.assertLessEqual(res.eta_overall, 1.0)
        self.assertGreater(res.eta_fin, 0.0)

    def test_spacing_positive_and_shrinks_with_more_fins(self):
        s10 = evaluate(self.cond, HeatSinkGeometry(10, 1.2e-3)).spacing
        s30 = evaluate(self.cond, HeatSinkGeometry(30, 1.2e-3)).spacing
        self.assertGreater(s10, 0.0)
        self.assertGreater(s10, s30)

    def test_infeasible_when_fins_do_not_fit(self):
        # base de 50 mm; 60 aletas de 1 mm não cabem (>= 50 mm ocupados).
        res = evaluate(self.cond, HeatSinkGeometry(n_fins=60, thickness=1.0e-3))
        self.assertFalse(res.feasible)
        self.assertEqual(res.t_chip, float("inf"))

    def test_feasible_flag_respects_t_max(self):
        # q_c alto o suficiente para estourar t_max com poucas aletas.
        hot = Conditions(q_c=60.0, t_max=45.0)
        res = evaluate(hot, HeatSinkGeometry(n_fins=5, thickness=1.2e-3))
        self.assertEqual(res.feasible, res.t_chip <= hot.t_max)


if __name__ == "__main__":
    unittest.main()
