"""Unit tests for the SDG writers in src/labeling/rep_utils.py (no Isaac Sim required).

pytest -v test/test_labeling_writers.py
"""

import cv2
import numpy as np

from src.labeling.rep_utils import writerFactory


def _segmentation_payload(value: int, **info) -> dict:
    # Replicator returns segmentation as an (H, W) uint32 id map plus an info dict.
    return {"data": np.full((4, 6), value, dtype=np.uint32), "info": info}


def _writer_kwargs(root) -> dict:
    # Mirrors AutonomousLabeling.formatWriterConfig: same root and prefix for every annotator of a camera.
    return {
        "root_path": str(root),
        "prefix": "camera_",
        "element_per_folder": 1000,
        "image_format": "png",
        "annot_format": "json",
    }


def test_instance_writer_does_not_overwrite_semantic_output(tmp_path):
    semantic = writerFactory("semantic_segmentation", **_writer_kwargs(tmp_path))
    instance = writerFactory("instance_segmentation", **_writer_kwargs(tmp_path))

    semantic.write(_segmentation_payload(1, idToLabels={"1": {"class": "rock"}}))
    instance.write(_segmentation_payload(7, idToLabels={"7": "/World/rock_0"}, idToSemantics={"7": {"class": "rock"}}))

    semantic_png = tmp_path / "camera_semantic_segmentation" / "0" / "0000.png"
    instance_png = tmp_path / "camera_instance_segmentation" / "0" / "0000.png"
    assert instance_png.exists()
    # The semantic mask must still hold the semantic ids (1), not the instance ids (7).
    assert np.all(cv2.imread(str(semantic_png), cv2.IMREAD_UNCHANGED)[..., 2] == 1)


def test_normals_writer_stores_float16_xyz(tmp_path):
    w = writerFactory("normals", root_path=str(tmp_path), prefix="camera_", element_per_folder=1000)
    data = np.zeros((4, 6, 4), np.float32)
    data[..., 2] = 1.0
    w.write(data)
    out = np.load(tmp_path / "camera_normals" / "0" / "0000.npz")["normals"]
    assert out.dtype == np.float16 and out.shape == (4, 6, 3)
    assert np.all(out[..., 2] == 1.0)
