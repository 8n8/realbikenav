""" The code to run on the microbit. """

# pylint: disable=import-error

from typing import Dict, Set, Tuple  # noqa: F401

from mypy_extensions import TypedDict


class OutputSlashState(TypedDict):
    """ It represents the data for sending to the output. """
    display: Set[Tuple[int, int]]
    direction: float
    acceleration: Tuple[int, int, int]
    buttonA: bool
    buttonB: bool


class RawInput(TypedDict):
    """ It represents all the raw inputs. """
    display: bytes
    compass_heading: int
    acceleration: Tuple[int, int, int]
    buttonA: bool
    buttonB: bool


class ParsedInput(TypedDict):
    """ It represents the all the input readings after parsing. """
    display: Set[Tuple[int, int]]
    direction: float
    acceleration: Tuple[int, int, int]
    buttonA: bool
    buttonB: bool
