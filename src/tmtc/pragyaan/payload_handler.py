__author__ = "Aleksa Stanivuk"
__status__ = "development"

from src.robots.subsystems_manager import PowerState
from PIL import Image, ImageDraw, ImageFont
import os
import omni.kit.app
from src.tmtc.yamcs_TMTC import ImagesHandler

class PayloadHandler:

    def __init__(self, images_handler:ImagesHandler, payload_conf):
        self._images_handler = images_handler
        self._init_apxs(payload_conf)

    def _init_apxs(self, payload_conf):
        self._apxs_conf = payload_conf["apxs"]
        self._APXS_WIDTH = payload_conf["apxs"]["resolution"][0]
        self._APXS_HEIGHT = payload_conf["apxs"]["resolution"][1]
        self._images_handler.add_bucket(payload_conf["apxs"]["bucket"]["name"], payload_conf["apxs"]["bucket"]["path"])

    def snap_apxs(self, apxs_power_state:PowerState=PowerState.OFF):
        APXS_BUCKET = self._apxs_conf["bucket"]["name"]
        APXS_COUNT = self._images_handler._counter[APXS_BUCKET]
        text_to_write = f"{APXS_BUCKET}_{APXS_COUNT}"

        if APXS_COUNT == 0:
            apxs_background_path = os.path.join(self._apxs_conf["samples_dir"], self._apxs_conf["no_data"])
        elif APXS_COUNT > 0 and apxs_power_state == PowerState.ON:
            apxs_background_path = os.path.join(self._apxs_conf["samples_dir"], self._apxs_conf["measurement"])
        else:
            return

        img = Image.open(apxs_background_path).convert('RGB').resize((self._APXS_WIDTH, self._APXS_HEIGHT))
        draw = ImageDraw.Draw(img)
        self._draw_text(draw, text=text_to_write, fill="black", position='top-right')
        self._images_handler.save_image(img, APXS_BUCKET)
    
    def _draw_text(self, draw: ImageDraw.ImageDraw, text: str, fill, position: str = "center"):
        APXS_FONT = ImageFont.load_default(self._APXS_HEIGHT//15) 
        bbox = draw.textbbox((0, 0), text, font=APXS_FONT)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]

        if position == "center":
            x = (self._APXS_WIDTH - text_width) // 2
            y = (self._APXS_HEIGHT - text_height) // 2
        elif position == "top-right":
            margin = 40
            x = self._APXS_WIDTH - text_width - margin
            y = margin
        else:
            x, y = position

        draw.text((x, y), text, fill=fill, font=APXS_FONT)