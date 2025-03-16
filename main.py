import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imread
import random
import open3d as o3d

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
    # Referenced from: https://stackoverflow.com/questions/54395735/how-to-work-with-heic-image-file-types-in-python
    heif_file = pillow_heif.read_heif(os.path.join(HEIF_IMAGE_FOLDER_PATH, camera.name))
    reference_image = Image.frombytes(
        heif_file.mode,
        heif_file.size,
        heif_file.data,
        "raw",
    )

    camera_calibration = CameraCalibration.from_file(os.path.join(XMP_IMAGE_METADATA_FOLDER_PATH, camera.name.split('.')[0] + '.xmp'))

    # Juicy mathematics
    rotation_matrix = np.identity(4, dtype=np.float32)
    rotation_matrix[:3, :3] = camera_calibration.rotation_matrix

    translation_matrix = np.array([
        [1, 0, 0, -camera_calibration.position[0]],
        [0, 1, 0, -camera_calibration.position[1]],
        [0, 0, 1, -camera_calibration.position[2]],
        [0, 0, 0, 1]
    ])

    camera_matrix = rotation_matrix @ translation_matrix

    nx = reference_image.width
    ny = reference_image.height

    viewport_matrix = np.array([
        [-nx / 2, 0, 0, (nx - 1) / 2],
        [0, ny / 2, 0, (ny - 1) / 2],
        [0, 0, 0.5, 0.5],
    ])

    focal_length_pixels = camera_calibration.focal_length_35mm * nx / 36

    # Calculate principal points in pixel coordinates
    principal_point_x = (camera_calibration.principal_point_u * 0.5 + 0.5) * nx
    principal_point_y = (camera_calibration.principal_point_v * 0.5 + 0.5) * ny

    K = np.array([
        [focal_length_pixels, 0, principal_point_x],
        [0, focal_length_pixels * camera_calibration.aspect_ratio, principal_point_y],
        [0, 0, 1]
    ])

    near, far = 0.1, 100.0
    projection_matrix = np.zeros((4, 4))
    projection_matrix[0, 0] = 2 * K[0, 0] / nx
    projection_matrix[0, 2] = 2 * K[0, 2] / nx - 1
    projection_matrix[1, 1] = 2 * K[1, 1] / ny
    projection_matrix[1, 2] = 2 * K[1, 2] / ny - 1
    projection_matrix[2, 2] = -(far + near) / (far - near)
    projection_matrix[2, 3] = -2 * far * near / (far - near)
    projection_matrix[3, 2] = -1

    # Reading point cloud
    pcd = o3d.io.read_point_cloud(RC_POINT_CLOUD_FILE_PATH)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    points = np.hstack((points, np.ones((points.shape[0], 1))))

    points_after_CM = camera_matrix @ points.T
    points_after_PM = projection_matrix @ points_after_CM

    # Normalize by w-coordinate (important for perspective division)
    with np.errstate(divide='ignore', invalid='ignore'):
        points_normalized = points_after_PM / points_after_PM[3]

    points_after_VP = viewport_matrix @ points_normalized

    fig, [ax, ax2] = plt.subplots(1, 2, figsize=(9, 9))

    ax.scatter(points_after_VP[0], points_after_VP[1], 
               c=colors, marker='.', s=1)
    ax.set_xlim(0, nx)
    ax.set_ylim(0, ny)
    ax.set_title(f"Point cloud projection")
    ax.set_aspect('equal', adjustable='box')

    # Open image from path using Pillow
    ax2.imshow(reference_image)
    ax2.set_title("Reference image")

    plt.show()

if __name__ == "__main__":
    main()
