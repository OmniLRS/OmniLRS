from __future__ import annotations

__author__ = "Shamistan Karimov, Bach Nguyen, Elian Neppel"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from typing import Any, Dict, Protocol


class Transport(Protocol):
    def start(self) -> None: ...
    def publish(self, frame: Dict[str, Any]) -> None: ...
    def close(self) -> None: ...
