__author__ = "Aleksa Stanivuk"
__status__ = "development"

import omni.kit.app
import os
from PIL import Image

class ImagesHandler:
    
    NO_DATA_RESOLUTION = (640, 480)
    INIT_COUNT = 0 

    def __init__(self, yamcs_processor, yamcs_address, images_conf, url_full_nginx):
        self._yamcs_processor = yamcs_processor
        self._yamcs_address = yamcs_address
        self.URL_FULL_NGINX = url_full_nginx
        self._NO_DATA_IMAGE_PATH = images_conf["no_data_image_path"]
        self._buckets = {}
        self._counter = {}
        self._init_buckets_and_counter(images_conf["buckets"])

    def _init_buckets_and_counter(self, buckets_conf):
        for bucket in buckets_conf:
            self.add_bucket(bucket["name"], bucket["path"])

    def add_bucket(self, bucket_name, path, init_count=INIT_COUNT):
        if bucket_name not in self._buckets:
            self._init_bucket(bucket_name, path, init_count)
        else:
            raise Exception(f"Bucket with name {bucket_name} already exists.")
        
    def _init_bucket(self, bucket_name, path, init_count):
        self._buckets[bucket_name] = path
        self._counter[bucket_name] = init_count

    #NOTE incrementing after saving because image_0 is NO_DATA image
    # should we start the counter with -1, and then increment the counter before saving
    # we can revert back to 
    # Do we want first image to be 0, or 1 ? Do we want NO_DATA to be 0? Does not make sense to have NO_DATA as 1
    #TODO We should find a way to show 'default' image as NO_DATA in case there is really no images in the storage
    def snap_no_data_images(self):
        for bucket_name in self._buckets:
            self.snap_no_data_image(bucket_name)

    def snap_no_data_image(self, bucket_name):
        image_path = self._NO_DATA_IMAGE_PATH 
        img = Image.open(image_path).convert('RGB').resize((self.NO_DATA_RESOLUTION[0], self.NO_DATA_RESOLUTION[1]))
        self.save_image(img, bucket_name)

    def save_image(self, image:Image, bucket_name:str):
        image_name = self._save_image_locally(image, bucket_name)
        self._counter[bucket_name] += 1
        self._inform_yamcs(image_name, bucket_name)

    def _save_image_locally(self, image, bucket) -> str:
        counter_number = self._counter[bucket]
        image_name = f"{bucket}_{counter_number:04d}.png"
        IMG_DIR = f"/tmp/{bucket}"
        os.makedirs(IMG_DIR, exist_ok=True)   # creates directory if missing
        img_path = f"{IMG_DIR}/{image_name}" 
        image.save(img_path)

        return image_name

    def _inform_yamcs(self, image_name, bucket):
        counter_number = self._counter[bucket]
        url_storage = f"/storage/buckets/{bucket}/objects/{image_name}"
        url_full = "http://" + self._yamcs_address + f"/api{url_storage}"
        url_full_nginx = self.URL_FULL_NGINX + f"/api{url_storage}" 
        self._yamcs_processor.set_parameter_values({
            self._buckets[bucket] + "/number": counter_number,
            self._buckets[bucket] + "/name": image_name,
            self._buckets[bucket] + "/url_storage": url_storage,
            self._buckets[bucket] + "/url_full": url_full,
            self._buckets[bucket] + "/url_full_nginx": url_full_nginx,
        })