"""Backend do ThermalCore Optimizer — motor de otimização termodinâmica."""

from .optimizer import (
    CRITERIA,
    HAVE_SCIPY,
    OptimizationResult,
    SweepResult,
    boundary_thickness,
    optimize,
    response_payload,
    sweep,
)
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
    # thermal_model
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
    # optimizer
    "CRITERIA",
    "HAVE_SCIPY",
    "OptimizationResult",
    "SweepResult",
    "boundary_thickness",
    "optimize",
    "response_payload",
    "sweep",
]
