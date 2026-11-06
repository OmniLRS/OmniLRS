"""Check start-image recognition along a square route using Pragyaan's camera.

Run from the repository root:
    pixi run --frozen -e default python benchmarks/square_route.py --run baseline
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
OUTPUTS = Path(__file__).resolve().parent / "outputs" / "square_route"
START = (18.0, 18.0)
SIDE = 5.0
STEP = 0.5
TURN_STEP = 15
SEED = 20261023
FEATURES = 1000
RATIO = 0.75
MIN_INLIERS = 20
MIN_INLIER_RATIO = 0.5
MIN_COVERAGE = 0.1
MAX_ALIGNMENT_ERROR = 0.05  # Fraction of the image diagonal.
REPROJECTION_ERROR = 3.0
START_RADIUS = 1.0  # Matches beyond this distance are false start recognitions.


def route() -> list[dict]:
    """Samples four straight legs and in-place left turns, ending at the starting pose."""
    x, y = START
    poses = [dict(xy=[x, y], yaw_deg=0)]
    for leg, (dx, dy) in enumerate(((1, 0), (0, 1), (-1, 0), (0, -1))):
        for _ in range(round(SIDE / STEP)):
            x, y = x + dx * STEP, y + dy * STEP
            poses.append(dict(xy=[x, y], yaw_deg=leg * 90))
        for angle in range(TURN_STEP, 91, TURN_STEP):
            poses.append(dict(xy=[x, y], yaw_deg=(leg * 90 + angle) % 360))
    return poses


def protocol() -> dict:
    return dict(
        version=1,
        start_m=list(START),
        side_m=SIDE,
        step_m=STEP,
        turn_step_deg=TURN_STEP,
        seed=SEED,
        features=FEATURES,
        ratio=RATIO,
        minimum_inliers=MIN_INLIERS,
        minimum_inlier_ratio=MIN_INLIER_RATIO,
        minimum_coverage=MIN_COVERAGE,
        maximum_alignment_error=MAX_ALIGNMENT_ERROR,
        reprojection_error_px=REPROJECTION_ERROR,
        start_radius_m=START_RADIUS,
        automatic_exposure=False,
    )


# --------------------------------------------------------------------------- rendering


def copy_camera(stage, asset_path: Path, camera_suffix: str):
    """Copies the asset camera's optics and upright mounting pose, facing along +X."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    layer = Sdf.Layer.FindOrOpen(str(asset_path))
    if not layer or not layer.defaultPrim:
        raise ValueError(f"Robot asset has no default prim: {asset_path}")
    asset = Usd.Stage.OpenMasked(layer, Usd.StagePopulationMask(["/" + layer.defaultPrim]))
    root = asset.GetDefaultPrim()
    if not root:
        raise ValueError(f"Robot asset has no default prim: {asset_path}")
    source = UsdGeom.Camera.Get(asset, str(root.GetPath()) + camera_suffix)
    if not source:
        raise ValueError(f"Camera not found in robot asset: {camera_suffix}")

    matrix = UsdGeom.Xformable(source).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    root_matrix = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # Keep the asset's upright orientation; remove its spawn translation and camera scale.
    pose = Gf.Transform(matrix)
    pose.SetTranslation(matrix.ExtractTranslation() - root_matrix.ExtractTranslation())
    pose.SetScale(Gf.Vec3d(1))
    mount = pose.GetMatrix()
    forward = -np.array(mount)[2, :3]
    if np.linalg.norm(forward[:2]) < 1e-6 or forward[2] >= 0:
        raise ValueError("Expected a forward/downward-facing rover camera in the upright USD asset.")
    heading = np.degrees(np.arctan2(forward[1], forward[0]))
    mount = mount * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), -float(heading)))

    camera = UsdGeom.Camera.Define(stage, "/LoopBench/Camera")
    for name in UsdGeom.Camera.GetSchemaAttributeNames(includeInherited=False):
        value = source.GetPrim().GetAttribute(name).Get()
        if value is not None:
            camera.GetPrim().GetAttribute(name).Set(value)
    return camera, mount


def render(output: Path, environment: str):
    os.chdir(REPO)
    sys.path.insert(0, str(REPO))
    from hydra import compose, initialize_config_dir

    from run import instantiateConfigs, omegaconfToDict

    with initialize_config_dir(config_dir=str(REPO / "cfg"), version_base="1.3"):
        config = compose(
            config_name="config",
            overrides=[
                f"environment={environment}",
                "robot=pragyaan",
                "rendering.renderer.headless=True",
                "++environment.monitoring_cameras_settings.enabled=False",
                "++environment.static_assets_settings=null",
            ],
        )
    resolved = omegaconfToDict(config)
    settings = resolved["environment"]
    if settings["name"] != "Lunaryard" or settings["rocks_settings"]["enable"]:
        raise ValueError("Select a Lunaryard environment with rock scattering disabled.")
    robot = settings["robots_settings"]["parameters"]
    height = robot["pose"]["height_above_terrain_override"]
    if height is None or height <= 0:
        raise ValueError("Pragyaan needs a positive height_above_terrain_override.")
    camera_root = f"{settings['robots_settings']['robots_root']}/{robot['robot_name']}"
    camera_path = robot["camera"]["prim_path"]
    if not camera_path.startswith(camera_root + "/"):
        raise ValueError("Configured camera path is not inside the rover.")
    config = instantiateConfigs(resolved)

    from isaacsim import SimulationApp

    launch = config["rendering"]["renderer"].__dict__.copy()
    tbn = launch.pop("tbn_frame_mode")
    app = SimulationApp(launch)

    import carb
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.replicator.core")
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom

    from src.configurations.simulator_mode_enum import SimulatorMode
    from src.environments.lunaryard import LunaryardController
    from src.environments.rendering import set_tbn_frame_mode

    set_tbn_frame_mode(tbn)
    carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)
    World(stage_units_in_meters=1.0)
    env = LunaryardController(mode=SimulatorMode.YAMCS, **config["environment"])
    env.load()
    camera, mount = copy_camera(env.stage, REPO / robot["usd_path"], camera_path[len(camera_root) :])
    transform = camera.AddTransformOp()
    resolution = robot["camera"]["resolutions"]["low"]
    product = rep.create.render_product(str(camera.GetPath()), tuple(resolution))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(product)
    manifest = dict(
        config=resolved,
        protocol=protocol(),
        resolution=resolution,
        camera_path=camera_path,
        camera_mount=np.array(mount).tolist(),
        camera_attributes={
            name: camera.GetPrim().GetAttribute(name).Get()
            for name in ("focalLength", "horizontalAperture", "verticalAperture")
        },
        sun_local_to_world=np.array(
            UsdGeom.Xformable(env._sun_lux.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        ).tolist(),
        sun_intensity=env._sun_lux.GetIntensityAttr().Get(),
        sun_angle_deg=env._sun_lux.GetAngleAttr().Get(),
        frames=[],
    )
    for _ in range(60):
        app.update()
    for i, pose in enumerate(route()):
        x, y = pose["xy"]
        z = float(env.get_terrain_height((x, y))) + height
        rover = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), pose["yaw_deg"]))
        rover.SetTranslateOnly(Gf.Vec3d(x, y, z))
        transform.Set(mount * rover)
        for _ in range(4 if i == 0 else 2):
            rep.orchestrator.step(rt_subframes=8)
        Image.fromarray(rgb.get_data()[..., :3]).save(output / f"frame_{i:03d}.png")
        manifest["frames"].append(
            dict(
                **pose,
                camera_to_world=np.array(camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default())).tolist(),
            )
        )
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    rep.orchestrator.stop()
    return app


# --------------------------------------------------------------------------- analysis


def match_start(reference, query, reference_shape, query_shape) -> tuple[dict, list]:
    """Returns the recognition result and the homography's inlier matches."""
    import cv2

    kp_ref, des_ref = reference
    kp_query, des_query = query
    result = dict(matches=0, inliers=0, inlier_ratio=0.0, coverage=0.0, alignment_error=None, recognized=False)
    if des_ref is None or des_query is None or len(des_ref) < 2:
        return result, []
    best = {}
    for pair in cv2.BFMatcher(cv2.NORM_L2).knnMatch(des_query, des_ref, k=2):
        if len(pair) == 2 and pair[0].distance < RATIO * pair[1].distance:
            match = pair[0]
            if match.trainIdx not in best or match.distance < best[match.trainIdx].distance:
                best[match.trainIdx] = match
    matches = list(best.values())
    result["matches"] = len(matches)
    if len(matches) < 4:
        return result, []
    src = np.float32([kp_ref[m.trainIdx].pt for m in matches])
    dst = np.float32([kp_query[m.queryIdx].pt for m in matches])
    homography, mask = cv2.findHomography(src, dst, cv2.RANSAC, REPROJECTION_ERROR)
    if homography is None or mask is None:
        return result, []
    height, width = reference_shape
    corners = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
    warped = cv2.perspectiveTransform(corners[None], homography)[0]
    alignment_error = float(np.max(np.linalg.norm(warped - corners, axis=1)) / np.hypot(width, height))
    if not np.isfinite(alignment_error):
        return result, []
    inliers = mask.ravel().astype(bool)
    count = int(inliers.sum())
    coverage = (
        min(
            cv2.contourArea(cv2.convexHull(points[inliers])) / (shape[0] * shape[1])
            for points, shape in ((src, reference_shape), (dst, query_shape))
        )
        if count >= 3
        else 0.0
    )
    result.update(
        inliers=count,
        inlier_ratio=count / len(matches),
        coverage=float(coverage),
        alignment_error=alignment_error,
        recognized=bool(
            count >= MIN_INLIERS
            and count / len(matches) >= MIN_INLIER_RATIO
            and coverage >= MIN_COVERAGE
            and alignment_error <= MAX_ALIGNMENT_ERROR
        ),
    )
    return result, [match for match, keep in zip(matches, inliers) if keep]


def read_gray(path: Path, resolution) -> np.ndarray:
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Cannot read benchmark image: {path}")
    if image.shape != (resolution[1], resolution[0]):
        raise ValueError(f"Image dimensions differ from the manifest: {path}")
    return image


def analyze(output: Path) -> None:
    import cv2

    manifest = json.loads((output / "manifest.json").read_text())
    if manifest["protocol"] != protocol():
        raise ValueError("Saved protocol differs. Use the script revision that produced this run.")
    if [(f["xy"], f["yaw_deg"]) for f in manifest["frames"]] != [(p["xy"], p["yaw_deg"]) for p in route()]:
        raise ValueError("Saved frames do not match the square route.")
    cv2.setRNGSeed(SEED)
    sift = cv2.SIFT_create(nfeatures=FEATURES)
    first = read_gray(output / "frame_000.png", manifest["resolution"])
    reference = sift.detectAndCompute(first, None)
    start = np.array(manifest["frames"][0]["camera_to_world"])[3, :3]
    rows = []
    for i, frame in enumerate(manifest["frames"][1:], 1):
        image = read_gray(output / f"frame_{i:03d}.png", manifest["resolution"])
        query = sift.detectAndCompute(image, None)
        row, inlier_matches = match_start(reference, query, first.shape, image.shape)
        position = np.array(frame["camera_to_world"])[3, :3]
        distance = float(np.linalg.norm(position - start))
        row.update(
            frame=i, distance_from_start_m=distance, false_start_match=row["recognized"] and distance > START_RADIUS
        )
        visualization = output / f"false_match_{i:03d}.png"
        if row["false_start_match"]:
            # Show every tenth inlier to keep the connecting lines readable.
            drawing = cv2.drawMatches(
                image,
                query[0],
                first,
                reference[0],
                inlier_matches[::10],
                None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            )
            if not cv2.imwrite(str(visualization), drawing):
                raise OSError(f"Cannot write false-match visualization: {visualization}")
            row["visualization"] = visualization.name
        elif visualization.exists():
            visualization.unlink()
        rows.append(row)
    results = dict(
        run=output.name,
        opencv_version=cv2.__version__,
        reference_features=len(reference[0]),
        false_start_matches=sum(row["false_start_match"] for row in rows),
        frames_far_from_start=sum(row["distance_from_start_m"] > START_RADIUS for row in rows),
        final_return_recognized=rows[-1]["recognized"],
        per_frame=rows,
    )
    (output / "results.json").write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")
    print(json.dumps(results | {"per_frame": "(see results.json)"}, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", required=True, help="unique output folder name")
    parser.add_argument("--environment", default="lunaryard_40m_workshop", help="Lunaryard config")
    parser.add_argument("--analyze-only", action="store_true", help="analyze saved images without Isaac Sim")
    args = parser.parse_args()
    if not args.run or args.run in (".", "..") or Path(args.run).name != args.run:
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
            app.close()
