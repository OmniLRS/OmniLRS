__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import multiprocessing.managers
from typing import List, Tuple, Dict
import multiprocessing
import numpy as np
import dataclasses
import threading
import logging
import signal
import time
import copy
import PIL
import cv2

from src.terrain_management.large_scale_terrain.crater_generation import (
    CraterBuilder,
)

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")


@dataclasses.dataclass
class InterpolatorConf:
    """
    Configuration for the interpolator.

    Args:
        source_resolution (float): The resolution of the source data (meters per pixel).
        target_resolution (float): The resolution of the target data (meters per pixel).
        source_padding (int): The padding of the source data (pixels).
        method (str): The interpolation method. Can be one of "nearest", "linear", "bicubic", "area".
    """

    source_resolution: float = dataclasses.field(default_factory=float)
    target_resolution: float = dataclasses.field(default_factory=float)
    source_padding: int = dataclasses.field(default_factory=int)
    method: str = dataclasses.field(default_factory=str)

    def __post_init__(self):
        self.method = self.method.lower()
        self.update()
        if self.source_padding < 2:
            logger.warn("Padding may be too small for interpolation.")
            self.source_padding = 2

        if self.method == "bicubic":
            self.method = cv2.INTER_CUBIC
            if self.fx < 1.0:
                logger.warn("Bicubic interpolation with downscaling. Consider using a different method.")
        elif self.method == "nearest":
            self.method = cv2.INTER_NEAREST
        elif self.method == "linear":
            self.method = cv2.INTER_LINEAR
        elif self.method == "area":
            self.method = cv2.INTER_AREA
            if self.fx > 1.0:
                logger.warn("Area interpolation with upscaling. Consider using a different method.")
        else:
            raise ValueError(f"Invalid interpolation method: {self.method}")

    def update(self):
        self.fx = self.source_resolution / self.target_resolution
        self.fy = self.source_resolution / self.target_resolution
        self.target_padding = int(self.source_padding * self.fx)


class Interpolator:
    """
    Interpolator base class.

    The Interpolator class is responsible for interpolating the terrain data.
    """

    def __init__(self, settings: InterpolatorConf):
        """
        Args:
            settings (InterpolatorConf): The settings for the interpolator.
        """

        self.settings = settings
        self.settings.update()

    def interpolate(self, data: np.ndarray) -> np.ndarray:
        """
        Interpolates the given data using the settings of the interpolator.

        Args:
            data (np.ndarray): The terrain data to interpolate.

        Returns:
            np.ndarray: The interpolated terrain data.

        TO BE IMPLEMENTED BY SUBCLASSES.
        """

        raise NotImplementedError


class CPUInterpolator(Interpolator):
    """
    An interpolator that uses the CPU to interpolate the terrain data.
    This interpolator uses the cv2 library to interpolate the data.
    In our testing, cv2 was generating nasty banding artifacts when
    upscaling the data. So unless the upsampling factor is small, and
    the data isn't smooth to start with, I'd recommend using the PIL
    interpolator instead.

    The cv2 interpolator is faster than the PIL interpolator by about 4 times.
    """

    def __init__(self, settings: InterpolatorConf):
        """
        Args:
            settings (InterpolatorCfg): The settings for the interpolator.
        """

        super().__init__(settings)

    def interpolate(self, data: np.ndarray) -> np.ndarray:
        """
        Interpolates the given data using the settings of the interpolator.
        The interpolation is done using cv2 resize.

        Args:
            data (np.ndarray): The terrain data to interpolate.

        Returns:
            np.ndarray: The interpolated terrain data.
        """

        return cv2.resize(
            data,
            (0, 0),
            fx=self.settings.fx,
            fy=self.settings.fy,
            interpolation=self.settings.method,
        )[
            int(self.settings.source_padding * self.settings.fx) : -int(
                self.settings.source_padding * self.settings.fy
            ),
            int(self.settings.source_padding * self.settings.fx) : -int(
                self.settings.source_padding * self.settings.fy
            ),
        ]


class CPUInterpolator_PIL(Interpolator):
    """
    An interpolator that uses the CPU to interpolate the terrain data.
    This interpolator uses the PIL library to interpolate the data.
    I found PIL to be more accurate than cv2 for bicubic interpolation.

    That's because cv2 uses an alpha value of -0.75 for its bicubic interpolation,
    when it should be -0.5.

    PIL uses a value of -0.5 for bicubic interpolation, which provides more accurate results,
    but is about 4 times slower than cv2.
    """

    def __init__(self, settings: InterpolatorConf):
        """
        Args:
            settings (InterpolatorCfg): The settings for the interpolator.
        """

        super().__init__(settings)

        if self.settings.method == cv2.INTER_CUBIC:
            self.settings.method = PIL.Image.BICUBIC
        else:
            raise ValueError(
                "PIL interpolation only supports bicubic interpolation. Use the CV2 interpolator instead. It's faster."
            )

    def interpolate(self, data: np.ndarray) -> np.ndarray:
        image = PIL.Image.fromarray(data)
        image = image.resize(
            (int(data.shape[1] * self.settings.fx), int(data.shape[0] * self.settings.fy)),
            resample=self.settings.method,
        )
        return np.array(image)[
            int(self.settings.source_padding * self.settings.fx) : -int(
                self.settings.source_padding * self.settings.fy
            ),
            int(self.settings.source_padding * self.settings.fx) : -int(
                self.settings.source_padding * self.settings.fy
            ),
        ]


class BaseWorker(multiprocessing.Process):
    """
    BaseWorker class. This class is used to create worker processes that can be used
    to distribute the work across multiple processes.
    """

    def __init__(
        self,
        queue_size: int,
        output_queue: multiprocessing.JoinableQueue,
        parent_thread: threading.Thread,
        thread_timeout: float = 1.0,
        **kwargs,
    ):
        """
        Args:
            queue_size (int): The size of the input queue.
            output_queue (multiprocessing.JoinableQueue): The output queue.
            parent_thread (threading.Thread): The parent thread.
            thread_timeout (float): The timeout for the worker thread. (seconds)
            **kwargs: Additional arguments.
        """

        super().__init__()
        self.stop_event = multiprocessing.Event()
        self.input_queue = multiprocessing.JoinableQueue(maxsize=queue_size)
        self.output_queue = output_queue
        self.daemon = True
        self.parent_thread = parent_thread
        self.thread_timeout = thread_timeout

    def get_input_queue_length(self) -> int:
        """
        Returns the length of the input queue.

        Returns:
            int: The length of the input queue.
        """

        return self.input_queue.qsize()

    def is_input_queue_empty(self) -> bool:
        """
        Checks if the input queue is empty.

        Returns:
            bool: True if the input queue is empty, False otherwise.
        """

        return self.input_queue.empty()

    def is_input_queue_full(self):
        """
        Checks if the input queue is full.

        Returns:
            bool: True if the input queue is full, False otherwise.
        """

        return self.input_queue.full()

    def run(self) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.

        TO BE IMPLEMENTED BY SUBCLASSES.
        """

        raise NotImplementedError

    def shutdown(self) -> None:
        """
        Shuts down the worker process.
        """

        logger.debug("Shutting down worker.")
        self.input_queue.put(((0, 0), None))
        self.stop_event.set()
        self.join()
        logger.debug("Worker joined.")
        while not self.input_queue.empty():
            self.input_queue.get()
        logger.debug("Worker input queue emptied.")
        try:
            while True:
                self.input_queue.task_done()
        except:
            pass
        self.input_queue.join()
        logger.debug("Worker input queue joined.")


@dataclasses.dataclass
class WorkerManagerConf:
    """
    Configuration for the worker manager.

    Args:
        num_workers (int): The number of workers.
        worker_queue_size (int): The size of the worker queue.
        input_queue_size (int): The size of the input queue.
        output_queue_size (int): The size of the output queue.
    """

    num_workers: int = dataclasses.field(default_factory=int)
    worker_queue_size: int = dataclasses.field(default_factory=int)
    input_queue_size: int = dataclasses.field(default_factory=int)
    output_queue_size: int = dataclasses.field(default_factory=int)


class BaseWorkerManager:
    """
    BaseWorkerManager class. This class is used to manage the worker processes.
    It is responsible for distributing the work across the workers and collecting the results.
    It ensures that the workers are running and that the work is distributed in a balanced way.
    I.e. the workers have a similar load.
    """

    def __init__(
        self,
        settings: WorkerManagerConf = WorkerManagerConf(),
        worker_class=None,
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
        **kwargs,
    ):
        """
        Args:
            settings (WorkerManagerConf): The settings for the worker manager.
            worker_class (BaseWorker): The worker class to use.
            parent_thread (threading.Thread): The parent thread.
            thread_timeout (float): The timeout for the worker thread. (seconds)
            **kwargs: Additional arguments.
        """

        self.manager = multiprocessing.Manager()
        # Create the input and output queues
        self.input_queue = multiprocessing.JoinableQueue(maxsize=settings.input_queue_size)
        self.output_queue = self.manager.Queue(maxsize=settings.output_queue_size)

        # Create the workers
        self.num_workers = settings.num_workers
        self.worker_queue_size = settings.worker_queue_size
        self.worker_class = worker_class
        self.parent_thread = parent_thread
        self.thread_timeout = thread_timeout
        self.kwargs = kwargs

        self.instantiate_workers(**kwargs)

        # Start the manager thread
        self.stop_event = threading.Event()
        self.manager_thread = threading.Thread(target=self.dispatch_jobs, daemon=True)
        self.manager_thread.start()

    def instantiate_workers(
        self,
        **kwargs,
    ) -> None:
        """
        Instantiates the worker processes.

        Args:
            **kwargs: Additional arguments. Used to pass the objects the workers need.
        """

        self.workers = [
            self.worker_class(
                self.worker_queue_size, self.output_queue, self.parent_thread, self.thread_timeout, **kwargs
            )
            for _ in range(self.num_workers)
        ]
        for worker in self.workers:
            worker.start()

    def get_load_per_worker(self) -> Dict[str, int]:
        """
        Returns the load of each worker.
        The dictionary contains the load of each worker in the form of:
            {
                "worker_0": 10,
                "worker_1": 20,
                ...
            }

        Where the key is the worker name and the value is the number of jobs
        in the worker's input queue.

        Returns:
            dict: A dictionary with the load of each worker.
        """

        return {"worker_{}".format(i): worker.get_input_queue_length() for i, worker in enumerate(self.workers)}

    def get_input_queue_length(self) -> int:
        """
        Returns the length of the input queue.

        Returns:
            int: The length of the input queue.
        """

        return self.input_queue.qsize()

    def get_output_queue_length(self) -> int:
        """
        Returns the length of the output queue.

        Returns:
            int: The length of the output queue.
        """

        return self.output_queue.qsize()

    def is_input_queue_empty(self) -> bool:
        """
        Checks if the input queue is empty.

        Returns:
            bool: True if the input queue is empty, False otherwise.
        """

        return self.input_queue.empty()

    def is_output_queue_empty(self) -> bool:
        """
        Checks if the output queue is empty.

        Returns:
            bool: True if the output queue is empty, False otherwise.
        """

        return self.output_queue.empty()

    def is_input_queue_full(self) -> bool:
        """
        Checks if the input queue is full.

        Returns:
            bool: True if the input queue is full, False otherwise.
        """

        return self.input_queue.full()

    def is_output_queue_full(self) -> bool:
        """
        Checks if the output queue is full.

        Returns:
            bool: True if the output queue is full, False otherwise.
        """

        return self.output_queue.full()

    def get_shortest_queue_index(self) -> int:
        """
        Returns the index of the worker with the shortest input queue.
        This is used to distribute the work in a balanced way.

        Returns:
            int: The index of the worker with the shortest input queue.
        """

        return min(
            range(len(self.workers)),
            key=lambda i: self.workers[i].get_input_queue_length(),
        )

    def are_workers_done(self) -> bool:
        """
        Checks if all the workers are done.
        This is not a reliable way to check if the workers are done.

        Returns:
            bool: True if all the workers are done, False otherwise.
        """

        return all([worker.is_input_queue_empty() for worker in self.workers]) and self.is_output_queue_empty()

    def dispatch_jobs(self, parent_thread: threading.Thread) -> None:
        """
        Dispatches the jobs to the workers.

        TO BE IMPLEMENTED BY SUBCLASSES.
        """

        raise NotImplementedError

    def process_data(self, coords: Tuple[float, float], data) -> None:
        """
        Processes the data by adding it to the worker manager's input queue.

        Args:
            coords (Tuple[float, float]): The coordinates of the data.
            data: The data to process.
        """

        self.input_queue.put((coords, data))

    def collect_results(self) -> List[Tuple[Tuple[float, float], np.ndarray]]:
        """
        Collects the results from the workers.

        Returns:
            List[Tuple[Tuple[float, float], np.ndarray]]: A list of tuples with the
            coordinates and the data.
        """

        results = []

        has_items = True
        while has_items:
            try:
                results.append(self.output_queue.get_nowait())
            except:
                has_items = False
        return results

    def shutdown(self) -> None:
        """
        Shuts down the worker manager and its workers.
        """

        logger.debug("Shutting down manager...")
        self.input_queue.put(((0, 0), None))
        self.manager_thread.join()
        logger.debug("Joined manager thread.")
        while not self.input_queue.empty():
            self.input_queue.get()
            self.input_queue.task_done()
        try:
            while True:
                self.input_queue.task_done()
        except:
            pass
        logger.debug("Emptied input queue.")
        self.input_queue.join()
        logger.debug("Joined input queue.")
        while not self.output_queue.empty():
            self.output_queue.get()
        logger.debug("Emptied output queue.")
        self.manager.shutdown()
        logger.debug("Multiprocessing.Manager shutdown complete.")
        logger.debug("Manager shutdown complete.")

    def shutdown_workers(self) -> None:
        """
        Shuts down the workers.
        """

        logger.debug("Shutting down workers...")
        for worker in self.workers:
            worker.shutdown()
        logger.debug("Workers shutdown complete.")

    def __del__(self) -> None:
        """
        Destructor.
        """

        # Try except to avoid multiple calls to shutdown
        try:
            self.shutdown()
        except Exception as e:
            pass


class CraterBuilderWorker(BaseWorker):
    """
    CraterBuilderWorker class. This class is responsible for generating the craters.
    """

    def __init__(
        self,
        queue_size: int = 10,
        output_queue: multiprocessing.Queue = multiprocessing.JoinableQueue(),
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
        builder: CraterBuilder = None,
    ) -> None:
        """
        Args:
            thread_timeout (float): The timeout for the worker thread. (seconds)
            builder (CraterBuilder): The crater builder.
        """

        super().__init__(queue_size, output_queue, parent_thread, thread_timeout)
        self.builder = copy.copy(builder)

    def run(self) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.
        It takes craters metadata and coordinates from the input queue
        and generates images with inprinted craters.
        """

        while not self.stop_event.is_set():
            try:
                coords, crater_meta_data = self.input_queue.get(timeout=self.thread_timeout)
                self.input_queue.task_done()
                if crater_meta_data is None:
                    break
                data_not_in_queue = True
                out = (coords, self.builder.generate_craters(crater_meta_data, coords))
                while data_not_in_queue:
                    try:
                        self.output_queue.put(out, timeout=0.1)
                        data_not_in_queue = False
                    except Exception as e:
                        pass
            except Exception as e:
                pass
        logger.debug("Crater Builder Worker exited processing loop.")


class CraterBuilderManager(BaseWorkerManager):
    """
    CraterBuilderManager class. This class is responsible for managing the crater
    builder workers. It is responsible for distributing the work across the workers
    and collecting the results.
    """

    def __init__(
        self,
        settings: WorkerManagerConf = WorkerManagerConf(),
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
        builder: CraterBuilder = None,
    ) -> None:
        """
        Args:
            settings (WorkerManagerCfg): The settings for the worker manager.
            parent_thread (threading.Thread): The parent thread.
            thread_timeout (float): The timeout for the worker thread. (seconds)
            builder (CraterBuilder): The crater builder.
        """

        super().__init__(
            settings=settings,
            worker_class=CraterBuilderWorker,
            builder=builder,
            parent_thread=parent_thread,
            thread_timeout=thread_timeout,
        )

    def dispatch_jobs(self) -> None:
        """
        Dispatches the jobs to the workers.
        The jobs are dispatched in a balanced way such that the workers have a similar load.
        """

        # Assigns the jobs such that the workers have a balanced load
        while (not self.stop_event.is_set()) and (self.parent_thread.is_alive()):
            try:
                coords, crater_metadata = self.input_queue.get(timeout=self.thread_timeout)
                if crater_metadata is None:  # Check for shutdown signal
                    self.input_queue.task_done()
                    break
                self.workers[self.get_shortest_queue_index()].input_queue.put((coords, crater_metadata))
                self.input_queue.task_done()
            except:
                pass
        self.shutdown_workers()
        logger.debug("Crater Builder Manager exited processing loop.")


class BicubicInterpolatorWorker(BaseWorker):
    """
    BicubicInterpolatorWorker class. This class is responsible for interpolating the
    terrain data.
    """

    def __init__(
        self,
        queue_size: int = 10,
        output_queue: multiprocessing.JoinableQueue = multiprocessing.JoinableQueue(),
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
        interp: Interpolator = None,
    ):
        """
        Args:
            thead_timeout (float): The timeout for the worker thread. (seconds)
            interp (Interpolator): The interpolator.
        """

        super().__init__(queue_size, output_queue, parent_thread, thread_timeout)
        self.interpolator = copy.copy(interp)

    def run(self) -> None:
        """
        The main function of the worker process.
        This function is called when the worker process is started.
        It takes terrain data from the input queue and interpolates it.
        """

        while not self.stop_event.is_set():
            try:
                coords, data = self.input_queue.get(timeout=self.thread_timeout)
                self.input_queue.task_done()
                if data is None:
                    break
                out = (coords, self.interpolator.interpolate(data))
                data_not_in_queue = True
                while data_not_in_queue:
                    try:
                        self.output_queue.put(out, timeout=0.1)
                        data_not_in_queue = False
                    except Exception as e:
                        pass
            except Exception as e:
                pass
        logger.debug("Bicubic Interpolator Worker exited processing loop.")


class BicubicInterpolatorManager(BaseWorkerManager):
    """
    BicubicInterpolatorManager class. This class is responsible for managing the
    bicubic interpolator workers. It is responsible for distributing the work across
    the workers and collecting the results.
    """

    def __init__(
        self,
        settings: WorkerManagerConf = WorkerManagerConf(),
        interp: Interpolator = None,
        num_cv2_threads: int = 4,
        parent_thread: threading.Thread = None,
        thread_timeout: float = 1.0,
    ):
        """
        Args:
            settings (WorkerManagerCfg): The settings for the worker manager.
            interp (Interpolator): The interpolator.
            num_cv2_threads (int): The number of threads to use for cv2.
            parent_thread (threading.Thread): The parent thread.
            thread_timeout (float): The timeout for the worker thread. (seconds
        """

        super().__init__(
            settings=settings,
            worker_class=BicubicInterpolatorWorker,
            interp=interp,
            parent_thread=parent_thread,
            thread_timeout=thread_timeout,
        )
        cv2.setNumThreads(num_cv2_threads)

    def dispatch_jobs(self) -> None:
        """
        Dispatches the jobs to the workers.
        The jobs are dispatched in a balanced way such that the workers have a similar load.
        """

        while (not self.stop_event.is_set()) and (self.parent_thread.is_alive()):
            try:
                coords, data = self.input_queue.get(timeout=self.thread_timeout)
                self.input_queue.task_done()
                if data is None:
                    break
                self.workers[self.get_shortest_queue_index()].input_queue.put((coords, data))
            except:
                pass
        self.shutdown_workers()
        logger.debug("Bicubic Interpolator manager exited processing loop.")


class ThreadMonitor:
    def __init__(self, is_simulation_alive: callable = lambda: True, close_simulation: callable = lambda: None):
        # Catch SIGINT
        self.ctrl_c = False
        signal.signal(signal.SIGINT, self.catch_sigint)
        # Catch Simulation shutdown
        self.is_simulation_alive = is_simulation_alive
        self.close_simulation = close_simulation
        # Start the monitor thread
        self.event = threading.Event()
        self.thread = threading.Thread(target=self.monitor_main_thread)
        self.thread.start()
        self.shutdowns = []

    def catch_sigint(self, sig, frame) -> None:
        """
        Catch the ctrl-c signal.
        """

        self.ctrl_c = True

    def add_shutdowns(self, *args):
        self.shutdowns += list(args)

    def apply_shutdowns(self):
        logger.debug("Applying registered shutdowns.")
        for shutdown in self.shutdowns:
            shutdown()

    def apply_shutdowns_in_different_process(self):
        logger.debug("Applying registered shutdowns in different process.")
        for shutdown in self.shutdowns:
            p = multiprocessing.Process(target=shutdown)
            p.start()
            p.join()

    def monitor_main_thread(self) -> None:
        """
        Monitors the main thread.
        This function is used to monitor the main thread and check if it is still alive.

        It monitors three things:
         - The main python thread.
         - The simulation thread.
         - The ctrl-c signal. This was added as it seems that the simulation was completely ignoring the ctrl-c signal.
        """

        while not self.event.is_set():
            time.sleep(1)
            if not threading.main_thread().is_alive():
                logger.debug("Main thread is dead, shutting down workers.")
                break
            if not self.is_simulation_alive():
                logger.debug("Simulation is dead, trying to shut down workers.")
                logger.warn(
                    "When the simulation is exited by clicking the close button, Isaac is aggressively killing everyting including the threads in charge of cleaning up..."
                )
                logger.warn("If the simulation window does not close it means that not all threads exited.")
                logger.warn("Use ps aux | grep run.py to find the remaining threads.")
                logger.warn(
                    "Use kill -9 PID to kill the remaining threads. PID being the PIDs returned by the previous command."
                )
                break
            if self.ctrl_c:
                logger.debug("Ctrl-C caught, shutting down workers.")
                break
        self.apply_shutdowns()
        logger.debug("Thread monitor exiting.")
