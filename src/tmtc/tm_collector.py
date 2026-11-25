__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import copy
import threading
from dataclasses import dataclass
from typing import Any, Iterable, Mapping


@dataclass
class TMEntry:
    value: Any = None
    updated_since_send: bool = False


class TMCollector:
    """
    Holds the latest (single) value of every defined TM parameter

    Functions as a bridge between the flight software and Yamcs publication:
        - flight software calls update()/update_many() whenever new data is available
        - the publishing logic periodically calls take_snapshot(), packs the values into a CCSDS packet and sends it to Yamcs
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()  # protects the stored values in case updates come from threads other than the sender's (for example ROS2 callbacks)
        self._entries: dict[str, TMEntry] = {}

    def define_parameter(self, name: str) -> None:
        self.define_parameters([name])

    def define_parameters(self, names: Iterable[str]) -> None:
        names = list(names)
        with self._lock:
            # validate first, so that a duplicate name does not leave a partial definition behind
            seen = set()
            for name in names:
                if name in self._entries or name in seen:
                    raise ValueError(f"TM parameter '{name}' is already defined")
                seen.add(name)
            for name in names:
                self._entries[name] = TMEntry()

    def is_defined(self, name: str) -> bool:
        return name in self._entries

    def get_parameter_names(self) -> list[str]:
        # preserves definition order, which can be relied upon for the packet layout
        return list(self._entries)

    def update(self, name: str, value: Any) -> None:
        with self._lock:
            self._set(name, value)

    def update_many(self, values: Mapping[str, Any]) -> None:
        with self._lock:
            # validate first, so that a bad name does not leave a partial update behind
            for name in values:
                self._check_defined(name)
            for name, value in values.items():
                self._set(name, value)

    def get(self, name: str) -> Any:
        with self._lock:
            self._check_defined(name)
            return copy.deepcopy(self._entries[name].value)

    def take_snapshot(self) -> dict[str, TMEntry]:
        """
        Returns copies of all entries and resets their updated_since_send flags.
        Intended to be called by the publishing logic each time a packet is built.
        The flags in the returned entries tell which values were updated since the previous snapshot.
        """
        with self._lock:
            snapshot = copy.deepcopy(self._entries)
            for entry in self._entries.values():
                entry.updated_since_send = False
            return snapshot

    def _set(self, name: str, value: Any) -> None:
        self._check_defined(name)
        entry = self._entries[name]
        entry.value = copy.deepcopy(value)
        entry.updated_since_send = True

    def _check_defined(self, name: str) -> None:
        if name not in self._entries:
            raise KeyError(f"TM parameter '{name}' is not defined")
