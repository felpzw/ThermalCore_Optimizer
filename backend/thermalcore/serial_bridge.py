"""Ponte serial entre a HMI (Arduino) e o motor de otimização (Módulo 1).

Fecha o loop do sistema: escuta os requisitos de projeto enviados pela HMI em
JSON, roda o otimizador e devolve a geometria ótima, também em JSON.

Protocolo (linhas terminadas em ``\\n``, 9600 baud por padrão):

    RX (da HMI):     {"q_c":20.00,"T_max":85.00}
    TX (para a HMI): {"N":14,"t":1.20}          (t em milímetros)

O transporte é abstraído por uma interface mínima (``read_line`` / ``write_line``),
de modo que o loop de serviço :func:`serve` é testável sem hardware. O adaptador
concreto sobre ``pyserial`` (:class:`SerialLineTransport`) é criado por
:func:`open_serial`, que importa ``pyserial`` sob demanda — a dependência é
opcional para tudo que não seja a comunicação física real.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from typing import Callable, Optional, Protocol

from .optimizer import optimize, response_payload
from .thermal_model import ALUMINUM, Conditions, Material

__all__ = [
    "LineTransport",
    "SerialLineTransport",
    "parse_request",
    "process_line",
    "serve",
    "open_serial",
    "main",
]


# ---------------------------------------------------------------------------
# Protocolo de aplicação (JSON)
# ---------------------------------------------------------------------------
def parse_request(line: str, base: Optional[Conditions] = None) -> Conditions:
    """Interpreta um request da HMI e devolve as :class:`Conditions` do projeto.

    Aceita as chaves ``q_c`` e ``T_max`` (também ``t_max``). Os demais parâmetros
    (h, geometria da base, etc.) vêm de ``base`` — permitindo ao operador fixar as
    condições de contorno enquanto a HMI só varia carga e temperatura crítica.

    Levanta ``ValueError`` para JSON inválido ou campos ausentes/mal formados.
    """
    try:
        data = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ValueError(f"JSON inválido: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("request deve ser um objeto JSON")

    try:
        q_c = float(data["q_c"])
        t_max = float(data["T_max"] if "T_max" in data else data["t_max"])
    except KeyError as exc:
        raise ValueError(f"campo obrigatório ausente: {exc}") from exc
    except (TypeError, ValueError) as exc:
        raise ValueError(f"campo numérico inválido: {exc}") from exc

    base = base if base is not None else Conditions(q_c=q_c, t_max=t_max)
    return replace(base, q_c=q_c, t_max=t_max)


def process_line(line: str, base: Optional[Conditions] = None,
                 material: Material = ALUMINUM, criterion: str = "mass") -> str:
    """Processa uma linha de request e retorna a linha de resposta JSON.

    Nunca levanta: erros de protocolo viram ``{"error":"bad_request"}`` e casos
    sem solução viável, ``{"error":"infeasible"}`` — mantendo o loop de serviço
    vivo diante de entradas ruins.
    """
    try:
        cond = parse_request(line, base)
    except ValueError:
        return json.dumps({"error": "bad_request"}, separators=(",", ":"))

    result = optimize(cond, material, criterion=criterion)
    return json.dumps(response_payload(result), separators=(",", ":"))


# ---------------------------------------------------------------------------
# Transporte
# ---------------------------------------------------------------------------
class LineTransport(Protocol):
    """Transporte orientado a linhas usado por :func:`serve`.

    ``read_line`` devolve a próxima linha (sem exigir o ``\\n``), ``""`` quando
    não há dados no momento (timeout), ou ``None`` quando o canal foi encerrado
    (encerra o loop). ``write_line`` envia uma linha de resposta.
    """

    def read_line(self) -> Optional[str]: ...

    def write_line(self, line: str) -> None: ...


class SerialLineTransport:
    """Adaptador de :class:`LineTransport` sobre um objeto ``pyserial.Serial``.

    Recebe qualquer objeto com ``readline()`` / ``write()`` / ``flush()`` (o
    ``Serial`` do pyserial ou um duble em teste), o que mantém este adaptador
    testável sem porta física.
    """

    def __init__(self, port) -> None:
        self._port = port

    def read_line(self) -> Optional[str]:
        raw = self._port.readline()
        if not raw:
            return ""  # timeout sem dados; o loop apenas segue aguardando
        return raw.decode("utf-8", errors="replace")

    def write_line(self, line: str) -> None:
        self._port.write((line + "\n").encode("utf-8"))
        self._port.flush()


def open_serial(port: str, baudrate: int = 9600,
                timeout: float = 1.0) -> SerialLineTransport:
    """Abre a porta serial física e devolve o transporte. Requer ``pyserial``."""
    try:
        import serial  # importação tardia: pyserial é opcional
    except ImportError as exc:  # pragma: no cover - depende do ambiente
        raise RuntimeError(
            "pyserial não instalado — necessário para a comunicação serial real "
            "(pip install pyserial)") from exc
    return SerialLineTransport(serial.Serial(port, baudrate, timeout=timeout))


# ---------------------------------------------------------------------------
# Loop de serviço
# ---------------------------------------------------------------------------
def serve(transport: LineTransport, base: Optional[Conditions] = None,
          material: Material = ALUMINUM, criterion: str = "mass",
          logger: Optional[Callable[[str], None]] = None) -> int:
    """Escuta requests no transporte e responde até o canal encerrar.

    Retorna o número de requests atendidos. Linhas vazias (timeouts) são
    ignoradas; ``read_line() is None`` encerra o loop.
    """
    served = 0
    while True:
        line = transport.read_line()
        if line is None:
            break                    # canal encerrado
        line = line.strip()
        if not line:
            continue                 # timeout / linha vazia
        if logger:
            logger(f"RX: {line}")
        response = process_line(line, base, material, criterion)
        transport.write_line(response)
        served += 1
        if logger:
            logger(f"TX: {response}")
    return served


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Listener serial do ThermalCore Optimizer (Módulo 1).")
    parser.add_argument("port", help="porta serial (ex.: /dev/ttyUSB0, COM3)")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="baud rate")
    parser.add_argument("-c", "--criterion", default="mass",
                        choices=("mass", "spacing"), help="critério de otimização")
    parser.add_argument("--h", type=float, default=None,
                        help="coef. de convecção h [W/m²K] (sobrescreve o padrão)")
    args = parser.parse_args(argv)

    base = Conditions(q_c=0.0, t_max=0.0)
    if args.h is not None:
        base = replace(base, h=args.h)

    transport = open_serial(args.port, baudrate=args.baud)
    print(f"ThermalCore listener em {args.port} @ {args.baud} baud "
          f"(critério={args.criterion}). Ctrl-C para sair.")
    try:
        serve(transport, base=base, criterion=args.criterion, logger=print)
    except KeyboardInterrupt:
        print("\nencerrado.")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
