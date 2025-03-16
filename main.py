import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imread
import random

from PIL import Image
import pillow_heif

import os
import sys

from camera_parameter import CameraParameter
from camera_calibration import CameraCalibration

# File paths
HEIF_IMAGE_FOLDER_PATH = 'sample/images/'
XMP_IMAGE_METADATA_FOLDER_PATH = 'sample/images/'
RC_CAMERA_PARAMETERS_FILE_PATH = 'sample/camera_parameters.csv'
RC_POINT_CLOUD_FILE_PATH = 'sample/point_cloud.ply'

def main():
    # Check for args
    image_file_path = None
    if len(sys.argv) > 1:
        image_file_path = sys.argv[1]

    camera_paramters = CameraParameter.from_parameters_file(RC_CAMERA_PARAMETERS_FILE_PATH)

    camera = None
    if image_file_path is None:
        camera = random.choice(list(camera_paramters))
    else:
        found_camera_parameter = False
        for params in camera_paramters:
            if params.name == image_file_path.split('/')[-1]:
                camera = params
                found_camera_parameter = True

        if not found_camera_parameter:
            print(f'Camera parameter not found for {image_file_path}')
            return

    if camera is None: return

    # Load reference image
    heif_file = pillow_heif.read_heif(os.path.join(HEIF_IMAGE_FOLDER_PATH, camera.name))
    image = Image.frombytes(
        heif_file.mode,
        heif_file.size,
        heif_file.data,
        "raw",
    )

if __name__ == "__main__":
    main()
