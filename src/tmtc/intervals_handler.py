
__author__ = "Aleksa Stanivuk"
__status__ = "development"

from enum import Enum
import omni.timeline
import omni.kit.app

class IntervalName(Enum):
    # use for intervals that are repeatedly created or removed
    CAMERA_STREAMING = "camera_streaming"
    STOP_ROBOT = "stop_robot"
    OBC_STATE = "obc_state"
    CONTROLLED_DRIVE = "controlled_drive"
    NEUTRON_COUNT = "neutron_count"

class IntervalsHandler:
    def __init__(self):
        self._intervals = {}
        self._timeline = omni.timeline.get_timeline_interface()
        self._update_stream = omni.kit.app.get_app().get_update_event_stream()

    def add_new_interval(self, *, name: str, seconds: int, is_repeating: bool, execute_immediately: bool, function, f_args=()):
        if name in self._intervals:
            raise ValueError(f"Interval '{name}' already exists")

        interval = {
            "seconds": seconds,
            "next_time": self._timeline.get_current_time() + seconds,
            "repeat": is_repeating,
            "execute_immediately": execute_immediately,
            "func": function,
            "args": f_args,
            "sub": None,
        }

        def callback(e, _interval=interval, _name=name):
            now = self._timeline.get_current_time()
            if now < interval["next_time"]:
                return

            _interval["func"](*_interval["args"])

            if _interval["repeat"]:
                _interval["next_time"] = now + _interval["seconds"]
            else:
                if _interval["sub"] is not None:
                    _interval["sub"].unsubscribe()
                self._intervals.pop(_name, None)

        interval["sub"] = self._update_stream.create_subscription_to_pop(
                callback,
                name= name + "_callback", 
            )

        self._intervals[name] = interval

        if interval["execute_immediately"]: # makes sense for repeating intervals, as not to have to wait for the first interval to pass before executing func
            interval["func"](*interval["args"])

    def update_next_time(self, interval_name, new_interval_time=None):
        if interval_name not in self._intervals:
            raise ValueError(f"Interval '{interval_name}' does not exist")

        interval = self._intervals[interval_name]
        now = self._timeline.get_current_time()

        if new_interval_time is None:
            interval["next_time"] = now + interval["seconds"]
        else:
            interval["next_time"] = now + new_interval_time

    def does_exist(self, interval_name):
        return interval_name in self._intervals
    
    def remove_interval(self, interval_name):
        if interval_name not in self._intervals:
            return
        
        interval = self._intervals[interval_name]

        if interval["sub"] is not None:
            interval["sub"].unsubscribe()

        self._intervals.pop(interval_name, None)