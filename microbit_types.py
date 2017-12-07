""" The code to run on the microbit. """

# pylint: disable=import-error

from typing import Callable, Dict, Set, Tuple  # noqa: F401

from mypy_extensions import TypedDict


class OutputSlashState(TypedDict):
    """ It represents the data for sending to the output. """
    display: Set[Tuple[int, int]]
    direction: float
    acceleration: Tuple[int, int, int]
    buttonAcounter: int
    buttonBcounter: int


class RawInput(TypedDict):
    """ It represents all the raw inputs. """
    new_input: bool
    display: bytes
    compass_heading: int
    acceleration: Tuple[int, int, int]
    buttonA: bool
    buttonB: bool


class ParsedInput(TypedDict):
    """ It represents the all the input readings after parsing. """
    new_input: bool
    display: Set[Tuple[int, int]]
    direction: float
    acceleration: Tuple[int, int, int]
    buttonA: bool
    buttonB: bool


OSS = OutputSlashState
PI = ParsedInput
