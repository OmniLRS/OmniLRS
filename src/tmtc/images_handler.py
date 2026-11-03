__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import io
from collections import OrderedDict

from PIL import Image

from src.tmtc import yamcs_naming


class ImagesHandler:
    """Uploads images to Yamcs storage buckets and publishes their metadata parameters.

    Buckets: one per robot camera and kind (named by yamcs_naming), one per role of images.buckets in the controller
    yaml (e.g. lander, monitoring), plus roles added later with add_bucket (e.g. a payload).
    Every bucket is created on the server if missing; with images.clear_buckets_on_startup its old objects are deleted.
    Each image is encoded as PNG in memory and uploaded; then number, name and url_* are set under the bucket's
    metadata space system.

    Buckets created through the API get the server's default limits (Yamcs 5.13: 100 MB and 1000 objects), so before
    each upload the oldest objects of the bucket are deleted until the new image fits. A failed upload is logged and
    skipped, so it never stops the interval or TC thread that saved the image.
    """

    NO_DATA_RESOLUTION = (640, 480)

    # NOTE
    # image_0 is no no_data image
    # image_1 is the first image with real camera view
    INIT_COUNT = 0

    def __init__(self, storage_client, yamcs_processor, yamcs_address, images_conf, url_full_nginx, cameras):
        self._storage = storage_client
        self._yamcs_processor = yamcs_processor
        self._yamcs_address = yamcs_address
        self.URL_FULL_NGINX = url_full_nginx
        self._NO_DATA_IMAGE_PATH = images_conf["no_data_image_path"]
        self._clear_on_startup = images_conf["clear_buckets_on_startup"]
        self._buckets = {}  # bucket name -> metadata space system
        self._counter = {}
        self._limits = {}  # bucket name -> (max object count, max size in bytes); None = no limit
        self._stored = {}  # bucket name -> OrderedDict(object name -> size), oldest first
        self._roles = {}  # role -> bucket name
        for _, _, bucket_name, path in yamcs_naming.camera_buckets(cameras):
            self._add_bucket(bucket_name, path)
        for role, bucket_conf in images_conf["buckets"].items():
            self.add_bucket(role, bucket_conf)

    def add_bucket(self, role: str, bucket_conf) -> None:
        """Register a bucket by role; bucket_conf has the bucket `name` and the metadata space system `path`."""
        if role in self._roles:
            raise Exception(f"Bucket role {role} already exists.")
        self._add_bucket(bucket_conf["name"], bucket_conf["path"])
        self._roles[role] = bucket_conf["name"]

    def _add_bucket(self, bucket_name, path):
        if bucket_name in self._buckets:
            raise Exception(f"Bucket with name {bucket_name} already exists.")
        bucket = self._storage.get_bucket(bucket_name, create=True)
        objects = sorted(bucket.list_objects().objects, key=lambda obj: obj.created)
        if self._clear_on_startup:
            for obj in objects:
                obj.delete()
            objects = []
        self._buckets[bucket_name] = path
        self._counter[bucket_name] = self.INIT_COUNT
        self._limits[bucket_name] = (bucket.max_object_count, bucket.max_size)
        self._stored[bucket_name] = OrderedDict((obj.name, obj.size) for obj in objects)

    def image_count(self, role: str) -> int:
        """Number of images uploaded so far to the bucket of that role."""
        return self._counter[self._roles[role]]

    def snap_no_data_images(self):
        """Called once every bucket is registered: warns about leftover image buckets, then uploads a placeholder
        image to each bucket."""
        self._warn_leftover_buckets()
        for bucket_name in self._buckets:
            self._snap_no_data_image(bucket_name)

    def _warn_leftover_buckets(self):
        """Warn about image buckets on the server that OmniLRS no longer uses (e.g. a renamed camera, or a kind its
        type no longer produces). They are kept: delete them by hand if their images are not needed."""
        leftovers = sorted(
            bucket.name
            for bucket in self._storage.list_buckets()
            if bucket.name.startswith(yamcs_naming.BUCKET_PREFIX) and bucket.name not in self._buckets
        )
        if leftovers:
            print(f"WARNING: Yamcs storage has image buckets this robot does not use (kept): {', '.join(leftovers)}")

    def _snap_no_data_image(self, bucket_name):
        image_path = self._NO_DATA_IMAGE_PATH
        img = Image.open(image_path).convert("RGB").resize((self.NO_DATA_RESOLUTION[0], self.NO_DATA_RESOLUTION[1]))
        self._save(img, bucket_name)

    def save_camera_image(self, image: Image, camera_name: str, kind: str):
        """Image of a robot camera; kind is one of yamcs_naming.camera_kinds(camera)."""
        self._save(image, yamcs_naming.bucket_name(camera_name, kind))

    def save_image(self, image: Image, role: str):
        """Image for a bucket role (images.buckets in the controller yaml, or added with add_bucket)."""
        self._save(image, self._roles[role])

    def _save(self, image: Image, bucket_name: str):
        try:
            image_name = self._upload(image, bucket_name)
        except Exception as e:
            print(f"Image upload to bucket {bucket_name} failed, image skipped: {e}")
            return
        self._inform_yamcs(image_name, bucket_name)
        self._counter[bucket_name] += 1

    def _upload(self, image: Image, bucket_name: str) -> str:
        image_name = f"{bucket_name}_{self._counter[bucket_name]:04d}.png"
        png = io.BytesIO()
        image.save(png, format="PNG")
        size = png.tell()
        png.seek(0)
        self._make_room(bucket_name, image_name, size)
        self._storage.upload_object(bucket_name, image_name, png, content_type="image/png")
        self._stored[bucket_name][image_name] = size
        return image_name

    def _make_room(self, bucket_name: str, image_name: str, size: int):
        """Delete the oldest objects of the bucket until an upload of `size` bytes as `image_name` fits its limits."""
        stored = self._stored[bucket_name]
        stored.pop(image_name, None)  # an upload under an existing name replaces that object
        max_objects, max_size = self._limits[bucket_name]
        while stored and (
            (max_objects is not None and len(stored) >= max_objects)
            or (max_size is not None and sum(stored.values()) + size > max_size)
        ):
            oldest, _ = stored.popitem(last=False)
            self._storage.remove_object(bucket_name, oldest)

    def _inform_yamcs(self, image_name, bucket_name):
        counter_number = self._counter[bucket_name]
        path = self._buckets[bucket_name]
        url_storage = f"/storage/buckets/{bucket_name}/objects/{image_name}"
        url_full = "http://" + self._yamcs_address + f"/api{url_storage}"
        url_full_nginx = self.URL_FULL_NGINX + f"/api{url_storage}"
        self._yamcs_processor.set_parameter_values(
            {
                path + "/number": counter_number,
                path + "/name": image_name,
                path + "/url_storage": url_storage,
                path + "/url_full": url_full,
                path + "/url_full_nginx": url_full_nginx,
            }
        )
