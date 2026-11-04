"""Measure image-to-map PnP localization on Lunaryard terrain.

A nadir reference image and depth map provide the 3D feature map. Query images
are matched with SIFT, localized with RANSAC PnP, and scored against the camera poses.

Run from the repository root (headless; no middleware or rover required):
    pixi run --frozen -e default python benchmarks/pnp/benchmark.py --run baseline
Re-analyse saved renders without Isaac Sim:
    pixi run --frozen -e default python benchmarks/pnp/benchmark.py --run baseline --analyze-only
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
OUTPUTS = Path(__file__).resolve().parent / "outputs"

SEED = 20261023
CENTER = (20.0, 20.0)  # Terrain coordinates in metres.
FOCAL, APERTURE = 20.0, 20.0  # Millimetres; square sensor with a 53.1-degree field of view.
REF_RES, REF_HEIGHT = 2048, 10.0  # Pixels and metres; nominal 10 x 10 m footprint.
QUERY_RES = 768

# Pose error thresholds: metres and degrees.
MAX_POS_ERR, MAX_ROT_ERR_DEG = 0.10, 2.0
RATIO, MIN_INLIERS = 0.75, 12
RANSAC = dict(iterationsCount=2000, reprojectionError=3.0, confidence=0.999, flags=0)


def intrinsics(res: int) -> np.ndarray:
    """Returns the intrinsic matrix for a square image."""
    f = FOCAL / APERTURE * res
    c = (res - 1) / 2.0
    return np.array([[f, 0, c], [0, f, c], [0, 0, 1]], dtype=np.float64)


def euler_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Converts yaw, pitch and roll in degrees to a rotation matrix."""
    from scipy.spatial.transform import Rotation

    return Rotation.from_euler("ZYX", [yaw, pitch, roll], degrees=True).as_matrix()


def query_poses():
    """Generates eight nadir poses on a 2 m grid and sixteen random tilted poses."""
    rng = np.random.default_rng(SEED)
    cx, cy = CENTER
    poses = []
    for dx, dy in [(0, 0), (2, 0), (-2, 0), (0, 2), (0, -2), (2, 2), (-2, -2), (2, -2)]:
        poses.append(dict(xy=(cx + dx, cy + dy), height=3.0, ypr=(0.0, 0.0, 0.0)))
    for _ in range(16):
        poses.append(
            dict(
                xy=tuple(rng.uniform(-2.0, 2.0, 2) + CENTER),
                height=float(rng.uniform(2.5, 3.5)),
                ypr=(float(rng.uniform(0, 360)), float(rng.uniform(-15, 15)), float(rng.uniform(-15, 15))),
            )
        )
    return poses


# --------------------------------------------------------------------------- rendering


def render(output: Path, environment: str):
    """Renders the reference and queries, then saves their poses and configuration."""
    os.chdir(REPO)
    sys.path.insert(0, str(REPO))
    from hydra import compose, initialize_config_dir

    from run import instantiateConfigs, omegaconfToDict

    with initialize_config_dir(config_dir=os.path.join(REPO, "cfg"), version_base="1.3"):
        cfg = compose(
            config_name="config",
            overrides=[
                f"environment={environment}",
                "rendering.renderer.headless=True",
                "++environment.monitoring_cameras_settings.enabled=False",
                "++environment.static_assets_settings=null",
            ],
        )
    resolved_cfg = omegaconfToDict(cfg)
    if resolved_cfg["environment"]["name"] != "Lunaryard":
        raise ValueError("This benchmark supports Lunaryard environments only.")
    if resolved_cfg["environment"]["rocks_settings"]["enable"]:
        raise ValueError("Disable rock scattering in the environment config for this terrain-only benchmark.")
    cfg = instantiateConfigs(resolved_cfg)

    from isaacsim import SimulationApp

    launch_config = cfg["rendering"]["renderer"].__dict__.copy()
    tbn = launch_config.pop("tbn_frame_mode")
    app = SimulationApp(launch_config)

    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.replicator.core")

    import carb
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom

    from src.configurations.simulator_mode_enum import SimulatorMode
    from src.environments.lunaryard import LunaryardController
    from src.environments.rendering import set_tbn_frame_mode
    from src.terrain_management.large_scale_terrain.pxr_utils import set_xform_ops

    set_tbn_frame_mode(tbn)
    # Keep exposure fixed across the reference and queries.
    carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)

    World(stage_units_in_meters=1.0)
    env = LunaryardController(mode=SimulatorMode.YAMCS, **cfg["environment"])
    env.load()
    stage = env.stage
    from scipy.spatial.transform import Rotation

    # Keep the initial sun pose from env.load(); capture does not advance the stellar engine.

    def make_camera(path, res):
        cam = UsdGeom.Camera.Define(stage, path)
        cam.CreateFocalLengthAttr(FOCAL)
        cam.CreateHorizontalApertureAttr(APERTURE)
        cam.CreateVerticalApertureAttr(APERTURE)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
        rp = rep.create.render_product(path, (res, res))
        return cam, rp

    def place(cam, xy, height, ypr):
        # USD cameras look along local -Z with +Y up; identity rotation is nadir.
        pos = Gf.Vec3d(xy[0], xy[1], float(env.get_terrain_height(xy)) + height)
        q = Rotation.from_matrix(euler_zyx(*ypr)).as_quat()
        set_xform_ops(cam.GetPrim(), pos, Gf.Quatd(q[3], Gf.Vec3d(*q[:3])), Gf.Vec3d(1, 1, 1))

    def world_pose(cam):
        """Camera-to-world pose in the OpenCV convention (x right, y down, z forward)."""
        m = np.array(UsdGeom.Xformable(cam).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        R = m[:3, :3].T @ np.diag([1.0, -1.0, -1.0])
        return R, m[3, :3]

    ref_cam, ref_rp = make_camera("/PnPBench/Reference", REF_RES)
    qry_cam, qry_rp = make_camera("/PnPBench/Query", QUERY_RES)
    ref_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    ref_depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    qry_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    ref_rgb.attach(ref_rp)
    ref_depth.attach(ref_rp)
    qry_rgb.attach(qry_rp)

    def settle(n=2):
        for _ in range(n):
            rep.orchestrator.step(rt_subframes=8)

    place(ref_cam, CENTER, REF_HEIGHT, (0.0, 0.0, 0.0))
    poses = query_poses()
    manifest = {
        "environment": environment,
        "config": resolved_cfg,
        "protocol": protocol(),
        "lighting": {
            "sun_local_to_world": np.array(
                UsdGeom.Xformable(env._sun_lux.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            ).tolist(),
            "sun_intensity": env._sun_lux.GetIntensityAttr().Get(),
            "sun_angle_deg": env._sun_lux.GetAngleAttr().Get(),
            "automatic_exposure": carb.settings.get_settings().get("/rtx/post/histogram/enabled"),
        },
        "queries": [],
    }

    for _ in range(60):
        app.update()
    settle(4)
    Image.fromarray(ref_rgb.get_data()[..., :3]).save(output / "reference.png")
    np.save(output / "reference_depth.npy", ref_depth.get_data().astype(np.float32))
    R, C = world_pose(ref_cam)
    manifest["reference"] = dict(R=R.tolist(), C=C.tolist())

    for i, p in enumerate(poses):
        place(qry_cam, p["xy"], p["height"], p["ypr"])
        settle()
        Image.fromarray(qry_rgb.get_data()[..., :3]).save(output / f"query_{i:02d}.png")
        R, C = world_pose(qry_cam)
        manifest["queries"].append(dict(**p, R=R.tolist(), C=C.tolist()))

    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    rep.orchestrator.stop()
    return app


# --------------------------------------------------------------------------- analysis


def protocol():
    """Returns the settings recorded with each run and checked before analysis."""
    return {
        "version": 2,
        "seed": SEED,
        "center_m": list(CENTER),
        "focal_mm": FOCAL,
        "aperture_mm": APERTURE,
        "reference_resolution": REF_RES,
        "reference_height_m": REF_HEIGHT,
        "query_resolution": QUERY_RES,
        "lighting": "environment_initial_pose",
        "automatic_exposure": False,
        "reference_features": 8000,
        "query_features": 400,
        "ratio": RATIO,
        "minimum_inliers": MIN_INLIERS,
        "ransac": RANSAC,
        "max_position_error_m": MAX_POS_ERR,
        "max_rotation_error_deg": MAX_ROT_ERR_DEG,
    }


def rotation_error_deg(Ra, Rb):
    """Returns the angular difference between two rotation matrices in degrees."""
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def solve(obj, img, K):
    """Returns camera-to-world rotation, position and inlier count, or None."""
    import cv2

    if len(obj) < MIN_INLIERS:
        return None
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img, K, None, **RANSAC)
    if not ok or inl is None or len(inl) < MIN_INLIERS:
        return None
    inl = inl[:, 0]
    rvec, tvec = cv2.solvePnPRefineLM(obj[inl], img[inl], K, None, rvec, tvec)
    Rw2c = cv2.Rodrigues(rvec)[0]
    return Rw2c.T, (-Rw2c.T @ tvec).ravel(), len(inl)


def check_solver(manifest, K, rng):
    """Checks pose recovery using synthetic 3D points and 0.5-pixel image noise."""
    for q in manifest["queries"]:
        R, C = np.array(q["R"]), np.array(q["C"])
        cam = np.c_[rng.uniform(-1, 1, (100, 2)), np.ones(100)] * rng.uniform(2, 4, (100, 1))
        world = cam @ R.T + C
        pix = cam @ K.T
        pix = pix[:, :2] / pix[:, 2:] + rng.normal(0, 0.5, (100, 2))
        est = solve(world, pix, K)
        if est is None or np.linalg.norm(est[1] - C) >= 0.01 or rotation_error_deg(est[0], R) >= 0.2:
            raise RuntimeError("Solver check failed: pose error exceeds 1 cm or 0.2 degrees, or no pose was found.")


def read_gray(path: Path) -> np.ndarray:
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Cannot read benchmark image: {path}")
    return image


def analyze(output: Path) -> None:
    """Matches saved images, estimates query poses and writes results.json."""
    import cv2

    with open(output / "manifest.json") as f:
        manifest = json.load(f)
    if manifest["protocol"] != protocol():
        raise ValueError("Saved protocol differs from this script. Use the script revision that produced this run.")
    K_ref, K_q = intrinsics(REF_RES), intrinsics(QUERY_RES)
    check_solver(manifest, K_q, np.random.default_rng(SEED))
    cv2.setRNGSeed(SEED)

    sift = cv2.SIFT_create(nfeatures=protocol()["query_features"])
    sift_ref = cv2.SIFT_create(nfeatures=protocol()["reference_features"])
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    R_ref, C_ref = np.array(manifest["reference"]["R"]), np.array(manifest["reference"]["C"])

    ref = read_gray(output / "reference.png")
    depth = np.load(output / "reference_depth.npy")
    if ref.shape != (REF_RES, REF_RES) or depth.shape != ref.shape:
        raise ValueError("Reference RGB/depth dimensions do not match the saved protocol.")
    kp, des = sift_ref.detectAndCompute(ref, None)

    # distance_to_image_plane is axial depth, not distance along the viewing ray.
    uv = np.array([k.pt for k in kp], dtype=float).reshape(-1, 2)
    ij = np.clip(np.round(uv).astype(int), 0, REF_RES - 1)
    z = depth[ij[:, 1], ij[:, 0]].astype(np.float64)
    valid = np.isfinite(z) & (z > 0)
    rays = np.c_[(uv - K_ref[:2, 2]) / K_ref[0, 0], np.ones(len(uv))]
    map_xyz = ((rays * z[:, None]) @ R_ref.T + C_ref)[valid]
    des = des[valid] if des is not None else None

    rows = []
    for i, q in enumerate(manifest["queries"]):
        img = read_gray(output / f"query_{i:02d}.png")
        if img.shape != (QUERY_RES, QUERY_RES):
            raise ValueError(f"Query {i} dimensions do not match the saved protocol.")
        qkp, qdes = sift.detectAndCompute(img, None)
        best = {}
        if qdes is not None and des is not None and len(des) >= 2:
            for pair in matcher.knnMatch(qdes, des, k=2):
                if len(pair) == 2 and pair[0].distance < RATIO * pair[1].distance:
                    m = pair[0]
                    if m.trainIdx not in best or m.distance < best[m.trainIdx].distance:
                        best[m.trainIdx] = m  # Keep one query observation per map point.
        ms = list(best.values())
        obj = np.array([map_xyz[m.trainIdx] for m in ms], dtype=np.float64).reshape(-1, 3)
        pix = np.array([qkp[m.queryIdx].pt for m in ms], dtype=np.float64).reshape(-1, 2)
        est = solve(obj, pix, K_q)

        R_gt, C_gt = np.array(q["R"]), np.array(q["C"])
        row = dict(query=i, matches=len(ms), inliers=0, outcome="no_pose")
        if est is not None:
            pos_err = float(np.linalg.norm(est[1] - C_gt))
            rot_err = rotation_error_deg(est[0], R_gt)
            ok = pos_err <= MAX_POS_ERR and rot_err <= MAX_ROT_ERR_DEG
            row.update(
                inliers=est[2],
                outcome="correct" if ok else "wrong",
                pos_err_m=pos_err,
                rot_err_deg=rot_err,
                offset_xy_m=(est[1] - C_gt)[:2].tolist(),
            )
        rows.append(row)

    counts = {k: sum(r["outcome"] == k for r in rows) for k in ("correct", "wrong", "no_pose")}
    returned = [r for r in rows if r["outcome"] != "no_pose"]
    results = dict(
        run=output.name,
        environment=manifest["environment"],
        opencv_version=cv2.__version__,
        solver_control="passed",
        reference_features=len(map_xyz),
        counts=counts,
        median_matches=float(np.median([r["matches"] for r in rows])),
        median_pos_err_returned_m=float(np.median([r["pos_err_m"] for r in returned])) if returned else None,
        median_rot_err_returned_deg=float(np.median([r["rot_err_deg"] for r in returned])) if returned else None,
        per_query=rows,
    )
    (output / "results.json").write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")
    print(json.dumps(results | {"per_query": "(see results.json)"}, indent=2), flush=True)
    print(f"Results: {output / 'results.json'}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", required=True, help="unique output name, e.g. baseline or new-terrain")
    parser.add_argument(
        "--environment", default="lunaryard_40m_workshop", help="Lunaryard config under cfg/environment"
    )
    parser.add_argument("--analyze-only", action="store_true", help="skip rendering, reuse saved images")
    args = parser.parse_args()
    if args.run in (".", "..") or Path(args.run).name != args.run:
        parser.error("--run must be a single folder name")
    output = OUTPUTS / args.run
    if args.analyze_only:
        analyze(output)
    else:
        output.mkdir(parents=True, exist_ok=False)
        app = render(output, args.environment)
        try:
            analyze(output)
        finally:
            app.close()  # exits the process
