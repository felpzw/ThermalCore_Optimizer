"""Backend do ThermalCore Optimizer — motor de otimização termodinâmica."""

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
    fin_lateral_area,
    fin_parameter,
    overall_efficiency,
    total_resistance,
)

__all__ = [
    "ALUMINUM",
    "Conditions",
    "HeatSinkGeometry",
    "Material",
    "ThermalResult",
    "array_resistance",
    "chip_temperature",
    "evaluate",
    "fin_efficiency",
    "fin_lateral_area",
    "fin_parameter",
    "overall_efficiency",
    "total_resistance",
]
