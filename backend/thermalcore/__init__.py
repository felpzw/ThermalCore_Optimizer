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
from .serial_bridge import (
    SerialLineTransport,
    open_serial,
    parse_request,
    process_line,
    serve,
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
    # serial_bridge
    "SerialLineTransport",
    "open_serial",
    "parse_request",
    "process_line",
    "serve",
]
