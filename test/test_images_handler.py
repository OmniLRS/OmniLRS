"""Unit tests for src/tmtc/images_handler.py with a fake Yamcs storage client (no Isaac Sim, no Yamcs server).

python -m pytest -v test/test_images_handler.py
"""

from types import SimpleNamespace

import pytest

Image = pytest.importorskip("PIL.Image")

from src.tmtc import yamcs_naming  # noqa: E402
from src.tmtc.images_handler import ImagesHandler  # noqa: E402

CAMERAS = yamcs_naming.parse_cameras(
    [{"name": "NavCam_left", "prim_path": "/l", "type": ["rgba"], "resolutions": {"low": [4, 3]}}]
)
STREAMING = yamcs_naming.bucket_name("NavCam_left", "streaming")
ONCOMMAND = yamcs_naming.bucket_name("NavCam_left", "oncommand")


class FakeStorage:
    """Mimics the yamcs-client StorageClient calls ImagesHandler makes, on buckets that already exist (they are
    defined on the Yamcs side)."""

    def __init__(self, existing=None, fail=False, page_size=None):
        self.buckets = {name: [] for name in (STREAMING, ONCOMMAND, "images_lander")}
        self.buckets.update({name: list(objects) for name, objects in (existing or {}).items()})
        self.fail = fail
        self.page_size = page_size  # objects returned per listing, None = all

    def list_objects(self, bucket_name):
        objects = self.buckets[bucket_name][: self.page_size]
        return SimpleNamespace(objects=[SimpleNamespace(name=name) for name in objects])

    def remove_object(self, bucket_name, object_name):
        self.buckets[bucket_name].remove(object_name)

    def upload_object(self, bucket_name, object_name, file_obj, content_type=None):
        if self.fail:
            raise RuntimeError("Maximum bucket size exceeded")
        assert content_type == "image/png"
        assert file_obj.read(8) == b"\x89PNG\r\n\x1a\n"
        self.buckets[bucket_name].append(object_name)


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


def test_uploads_and_publishes_metadata():
    storage = FakeStorage()
    handler, processor = _handler(storage)

    handler.save_camera_image(_image(), camera_name="NavCam_left", kind="streaming")
    path = yamcs_naming.meta_path("NavCam_left", "streaming")
    assert processor.values[path + "/number"] == 0
    assert processor.values[path + "/name"] == f"{STREAMING}_0000.png"
    assert processor.values[path + "/url_storage"] == f"/storage/buckets/{STREAMING}/objects/{STREAMING}_0000.png"
    assert (
        processor.values[path + "/url_full"]
        == f"http://localhost:8090/api/storage/buckets/{STREAMING}/objects/{STREAMING}_0000.png"
    )
    assert storage.buckets[STREAMING] == [f"{STREAMING}_0000.png"]


def test_keeps_every_image():
    storage = FakeStorage()
    handler, _ = _handler(storage)
    for _ in range(50):
        handler.save_camera_image(_image(), camera_name="NavCam_left", kind="streaming")
    assert storage.buckets[STREAMING] == [f"{STREAMING}_{i:04d}.png" for i in range(50)]


def test_clear_old_images_empties_every_bucket():
    storage = FakeStorage(existing={STREAMING: ["a.png", "b.png", "c.png"], "images_lander": ["d.png"]}, page_size=2)
    handler, _ = _handler(storage, clear=True)
    handler.clear_old_images()
    assert storage.buckets == {STREAMING: [], ONCOMMAND: [], "images_lander": []}


def test_clear_old_images_keeps_images_when_not_configured():
    storage = FakeStorage(existing={STREAMING: ["a.png"]})
    handler, _ = _handler(storage, clear=False)
    handler.clear_old_images()
    assert storage.buckets[STREAMING] == ["a.png"]


def test_role_image_goes_to_its_bucket():
    storage = FakeStorage()
    handler, processor = _handler(storage)
    handler.save_image(_image(), role="lander")
    assert storage.buckets["images_lander"] == ["images_lander_0000.png"]
    assert processor.values["/Lander/camera/images_oncommand/number"] == 0
    assert handler.image_count("lander") == 1


def test_upload_failure_is_logged_and_skipped(capsys):
    storage = FakeStorage(fail=True)
    handler, processor = _handler(storage)
    handler.save_image(_image(), role="lander")
    assert "images_lander failed" in capsys.readouterr().out
    assert processor.values == {}
    assert handler.image_count("lander") == 0
