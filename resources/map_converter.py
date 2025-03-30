import os

import struct
import glob
import numpy as np
from PIL import Image, ImageOps, ImageEnhance
from pathlib import Path


def arr_to_byte(arr):
    b = 0
    for i, a in enumerate(arr):
        if a != 0:
            b |= 1 << i
    return b


def rgb_to_16bit(rgb):
    r = int(rgb[0] >> 3) & 0x1F
    g = int(rgb[1] >> 2) & 0x3F
    b = int(rgb[2] >> 3) & 0x1F
    return struct.pack("H", (r << 11) + (g << 5) + b)


def convert_mono_tile(im):
    im = im.convert("1")
    im = ImageOps.invert(im)
    data = np.array(im.getdata(), dtype=np.uint8)
    data = [data[i : i + 8] for i in range(0, len(data), 8)]
    data = bytearray(map(arr_to_byte, data))
    assert len(data) == 512
    out_path = Path(p.split("/")[0] + "_c" + os.sep + os.sep.join(p.split("/")[1:]))
    out_path_ = out_path.with_suffix(".bin")
    out_path_.parents[0].mkdir(parents=True, exist_ok=True)
    return data, im


def convert_rgb_tile(im):
    im = im.convert("RGB")
    converter = ImageEnhance.Contrast(im)
    im = converter.enhance(2.0)
    # converter = ImageEnhance.Color(im)
    # im = converter.enhance(2.0)
    data = np.array(im.getdata(), dtype=np.uint8)
    out = bytearray()
    for rgb in data:
        out.extend(rgb_to_16bit(rgb))
    return out, im


paths = glob.glob("map4/**/*.png", recursive=True)

for p in paths:
    with Image.open(p) as im:
        im = im.rotate(-90)
        im = im.transpose(Image.FLIP_LEFT_RIGHT)
        data, im = convert_rgb_tile(im)
        out_path = Path(p.split("/")[0] + "_c" + os.sep + os.sep.join(p.split("/")[1:]))
        out_path_ = out_path.with_suffix(".bin")
        out_path_.parents[0].mkdir(parents=True, exist_ok=True)
        # im.save(out_path)
        with out_path_.open("wb") as f:
            f.write(data)
