__author__ = "Aleksa Stanivuk"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import io

from PIL import Image

from src.tmtc import yamcs_naming


class ImagesHandler:
    """Uploads images to Yamcs storage buckets and publishes their metadata parameters.

    Buckets: one per robot camera and kind (named by yamcs_naming), one per role of images.buckets in the controller
    yaml (e.g. lander, monitoring), plus roles added later with add_bucket (e.g. a payload).
    The buckets are defined on the Yamcs side (etc/yamcs.yaml, generated together with the MDB) with limits sized to
    keep every image of a mission; the startup check (src/tmtc/yamcs_contract.py) verifies they exist. Images are
    never deleted, except by clear_old_images() at startup when images.clear_buckets_on_startup is set.
    Each image is encoded as PNG in memory and uploaded; then number, name and url_* are set under the bucket's
    metadata space system. A failed upload (e.g. a full bucket) is logged and skipped, so it never stops the interval
    or TC thread that saved the image.
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
        self._roles = {}  # role -> bucket name
        for _, _, bucket_name, path in yamcs_naming.camera_buckets(cameras):
            self._add_bucket(bucket_name, path)
        for role, bucket_conf in images_conf["buckets"].items():
            self.add_bucket(role=role, bucket_conf=bucket_conf)

    def add_bucket(self, role: str, bucket_conf) -> None:
        """Register a bucket by role; bucket_conf has the bucket `name` and the metadata space system `path`."""
        if role in self._roles:
            raise Exception(f"Bucket role {role} already exists.")
        self._add_bucket(bucket_conf["name"], bucket_conf["path"])
        self._roles[role] = bucket_conf["name"]

    def _add_bucket(self, bucket_name, path):
        if bucket_name in self._buckets:
            raise Exception(f"Bucket with name {bucket_name} already exists.")
        self._buckets[bucket_name] = path
        self._counter[bucket_name] = self.INIT_COUNT

    def image_count(self, role: str) -> int:
        """Number of images uploaded so far to the bucket of that role."""
        return self._counter[self._roles[role]]

    def clear_old_images(self):
        """At startup, before any upload: with images.clear_buckets_on_startup, delete the objects of every bucket."""
        if not self._clear_on_startup:
            return
        for bucket_name in self._buckets:
            # Repeat the listing until the bucket is empty, in case a listing does not return every object
            while objects := self._storage.list_objects(bucket_name).objects:
                for obj in objects:
                    self._storage.remove_object(bucket_name, obj.name)

    def snap_no_data_images(self):
        """Upload a placeholder image to every bucket."""
        for bucket_name in self._buckets:
            self._snap_no_data_image(bucket_name)

    def _snap_no_data_image(self, bucket_name):
        image_path = self._NO_DATA_IMAGE_PATH
        img = Image.open(image_path).convert("RGB").resize((self.NO_DATA_RESOLUTION[0], self.NO_DATA_RESOLUTION[1]))
        self._save(img, bucket_name)

    def save_camera_image(self, image: Image.Image, camera_name: str, kind: str):
        """Image of a robot camera; kind is one of yamcs_naming.camera_kinds(camera)."""
        self._save(image, yamcs_naming.bucket_name(camera_name, kind))

    def save_image(self, image: Image.Image, role: str):
        """Image for a bucket role (images.buckets in the controller yaml, or added with add_bucket)."""
        self._save(image, self._roles[role])

    def _save(self, image: Image.Image, bucket_name: str):
        image_name = f"{bucket_name}_{self._counter[bucket_name]:04d}.png"
        png = io.BytesIO()
        image.save(png, format="PNG")
        png.seek(0)
        try:
            self._storage.upload_object(bucket_name, image_name, png, content_type="image/png")
        except Exception as e:
            print(f"Image upload to bucket {bucket_name} failed, image skipped: {e}")
            return
        self._inform_yamcs(image_name, bucket_name)
        self._counter[bucket_name] += 1

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
