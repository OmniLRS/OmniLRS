"""Unit tests for src/tmtc/images_handler.py with a fake Yamcs storage client (no Isaac Sim, no Yamcs server).

python -m pytest -v test/test_images_handler.py
"""

import datetime
import io
from types import SimpleNamespace

import pytest

Image = pytest.importorskip("PIL.Image")

from src.tmtc import yamcs_naming  # noqa: E402
from src.tmtc.images_handler import ImagesHandler  # noqa: E402

CAMERAS = yamcs_naming.parse_cameras(
    [{"name": "NavCam_left", "prim_path": "/l", "type": ["rgba"], "resolutions": {"low": [4, 3]}}]
)
STREAMING = yamcs_naming.bucket_name("NavCam_left", "streaming")


class FakeStorage:
    """Mimics the yamcs-client StorageClient and the server-side bucket limits (put fails when over a limit)."""

    def __init__(self, max_objects=None, max_size=None, existing=None, fail=False):
        self.max_objects = max_objects
        self.max_size = max_size
        self.fail = fail
        self.buckets = {name: dict(objects) for name, objects in (existing or {}).items()}
        self._clock = 0

    def _now(self):
        self._clock += 1
        return datetime.datetime.fromtimestamp(self._clock)

    def get_bucket(self, name, create=False):
        assert create
        self.buckets.setdefault(name, {})
        return SimpleNamespace(
            name=name,
            max_object_count=self.max_objects,
            max_size=self.max_size,
            list_objects=lambda: SimpleNamespace(objects=self._listing(name)),
        )

    def _listing(self, bucket):
        return [
            SimpleNamespace(
                name=obj, size=size, created=created, delete=lambda obj=obj: self.remove_object(bucket, obj)
            )
            for obj, (size, created) in self.buckets[bucket].items()
        ]

    def list_buckets(self):
        return iter(SimpleNamespace(name=name) for name in self.buckets)

    def upload_object(self, bucket, name, file_obj, content_type=None):
        if self.fail:
            raise RuntimeError("storage unavailable")
        size = len(file_obj.read())
        objects = {k: v for k, v in self.buckets[bucket].items() if k != name}
        if self.max_objects is not None and len(objects) + 1 > self.max_objects:
            raise RuntimeError("Maximum number of objects in the bucket exceeded")
        if self.max_size is not None and sum(s for s, _ in objects.values()) + size > self.max_size:
            raise RuntimeError("Maximum bucket size exceeded")
        self.buckets[bucket][name] = (size, self._now())

    def remove_object(self, bucket, name):
        del self.buckets[bucket][name]


class FakeProcessor:
    def __init__(self):
        self.values = {}

    def set_parameter_values(self, values):
        self.values.update(values)


def _handler(storage, clear=True):
    images_conf = {
        "no_data_image_path": "unused.png",
        "clear_buckets_on_startup": clear,
        "buckets": {"lander": {"name": "images_lander", "path": "/Lander/camera/images_oncommand"}},
    }
    processor = FakeProcessor()
    handler = ImagesHandler(storage, processor, "localhost:8090", images_conf, "http://nginx", CAMERAS)
    return handler, processor


def _image():
    return Image.new("RGB", (8, 8))


def test_creates_every_bucket_and_publishes_metadata():
    storage = FakeStorage()
    handler, processor = _handler(storage)
    assert set(storage.buckets) == {
        STREAMING,
        yamcs_naming.bucket_name("NavCam_left", "oncommand"),
        "images_lander",
    }

    handler.save_camera_image(_image(), "NavCam_left", "streaming")
    path = yamcs_naming.meta_path("NavCam_left", "streaming")
    assert processor.values[path + "/number"] == 0
    assert processor.values[path + "/url_storage"] == f"/storage/buckets/{STREAMING}/objects/{STREAMING}_0000.png"
    assert list(storage.buckets[STREAMING]) == [f"{STREAMING}_0000.png"]


def test_deletes_oldest_objects_to_stay_under_the_object_limit():
    storage = FakeStorage(max_objects=3)
    handler, _ = _handler(storage)
    for _ in range(5):
        handler.save_camera_image(_image(), "NavCam_left", "streaming")
    assert list(storage.buckets[STREAMING]) == [f"{STREAMING}_{i:04d}.png" for i in (2, 3, 4)]


def test_deletes_oldest_objects_to_stay_under_the_size_limit():
    png = io.BytesIO()
    _image().save(png, format="PNG")
    storage = FakeStorage(max_size=2 * png.tell() + 1)
    handler, _ = _handler(storage)
    for _ in range(4):
        handler.save_camera_image(_image(), "NavCam_left", "streaming")
    assert list(storage.buckets[STREAMING]) == [f"{STREAMING}_{i:04d}.png" for i in (2, 3)]


def test_kept_objects_count_towards_the_limits():
    old = {
        STREAMING: {"old_a.png": (1, datetime.datetime(2020, 1, 2)), "old_b.png": (1, datetime.datetime(2020, 1, 1))}
    }
    storage = FakeStorage(max_objects=2, existing=old)
    handler, _ = _handler(storage, clear=False)
    handler.save_camera_image(_image(), "NavCam_left", "streaming")
    assert set(storage.buckets[STREAMING]) == {"old_a.png", f"{STREAMING}_0000.png"}  # oldest (old_b) went first


def test_upload_failure_is_logged_and_skipped(capsys):
    storage = FakeStorage(fail=True)
    handler, processor = _handler(storage)
    handler.save_image(_image(), "lander")
    assert "images_lander failed" in capsys.readouterr().out
    assert processor.values == {}
    assert handler.image_count("lander") == 0


def test_warns_about_leftover_image_buckets_only(capsys, monkeypatch):
    existing = {"images_NavCam_right_streaming": {}, "stacks": {}, "images_lander": {}}
    handler, _ = _handler(FakeStorage(existing=existing))
    monkeypatch.setattr(handler, "_snap_no_data_image", lambda bucket_name: None)
    handler.snap_no_data_images()
    out = capsys.readouterr().out
    assert "images_NavCam_right_streaming" in out
    assert "stacks" not in out
    assert "images_lander" not in out
