import numpy as np
import matplotlib.pyplot as plt
import random
import open3d as o3d
from PIL import Image
import pillow_heif

import os
import sys

from camera_calibration import CameraCalibration
from utils import get_files_with_extension

# File paths
HEIF_IMAGE_FOLDER_PATH = 'sample/images/'
XMP_IMAGE_METADATA_FOLDER_PATH = 'sample/images/'
RC_POINT_CLOUD_FILE_PATH = 'sample/point_cloud.ply'

# @deprecated - cameraParameteres are no longer used instead .xmp metadata files are automatically extracted
RC_CAMERA_PARAMETERS_FILE_PATH = 'sample/camera_parameters.csv'

def plot_point_cloud_projection(camera_calibration, reference_image, 
                               points, colors, color_ax, reference_ax, depth_ax):
    """
    Plot point cloud projection and reference image on given axes.
    Parameters:
    -----------
    camera_calibration : CameraCalibration object
        The camera calibration data
    camera_parameter : CameraParameter object
        The camera parameter data
    reference_image : PIL.Image
        The reference image to display
    points : numpy.ndarray
        The 3D point cloud points
    colors : numpy.ndarray
        The colors corresponding to the points
    color_ax : matplotlib.axes.Axes
        The axis for plotting the point cloud projection
    reference_ax : matplotlib.axes.Axes
        The axis for plotting the reference image
    """
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

    # Camera intrinsic matrix
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

    # Ensure points are in homogeneous coordinates
    if points.shape[1] == 3:
        points = np.hstack((points, np.ones((points.shape[0], 1))))

    points_after_CM = camera_matrix @ points.T
    points_after_PM = projection_matrix @ points_after_CM

    # Normalize by w-coordinate (important for perspective division)
    with np.errstate(divide='ignore', invalid='ignore'):
        points_normalized = points_after_PM / points_after_PM[3]

    points_after_VP = viewport_matrix @ points_normalized

    # Plot point cloud projection
    color_ax.scatter(points_after_VP[0], points_after_VP[1], 
              c=colors, marker='.', s=1)
    color_ax.set_xlim(0, nx)
    color_ax.set_ylim(0, ny)
    color_ax.set_title(f"Point cloud projection")
    color_ax.set_aspect('equal', adjustable='box')

    if depth_ax:
        depths = points_after_CM[2]
        min_depth = np.min(depths)
        max_depth = np.max(depths)
        normalized_depths = (depths - min_depth) / (max_depth - min_depth)
        normalized_depths = 1 - normalized_depths

        # Plot point cloud projection
        depth_ax.set_facecolor('black')
        depth_ax.scatter(points_after_VP[0], points_after_VP[1], 
                  c=normalized_depths, marker='.', s=1, cmap="gray")
        depth_ax.set_xlim(0, nx)
        depth_ax.set_ylim(0, ny)
        depth_ax.set_title(f"Depth map")
        depth_ax.set_aspect('equal', adjustable='box')

    if reference_ax:
        # Plot reference image
        reference_ax.imshow(reference_image) 
        reference_ax.set_title("Reference image")

def do_stuff_with_camera(reference_id, points, colors, color_ax, reference_ax, depth_ax):
    # Load reference image
    # Referenced from: https://stackoverflow.com/questions/54395735/how-to-work-with-heic-image-file-types-in-python
    heif_file = pillow_heif.read_heif(os.path.join(HEIF_IMAGE_FOLDER_PATH, reference_id + '.heif'))
    reference_image = Image.frombytes(
        heif_file.mode,
        heif_file.size,
        heif_file.data,
        "raw",
    )

    camera_calibration = CameraCalibration.from_file(os.path.join(XMP_IMAGE_METADATA_FOLDER_PATH, reference_id + '.xmp'))
    plot_point_cloud_projection(camera_calibration, reference_image, points, colors, color_ax, reference_ax, depth_ax)
    pass

def main():
    # Check for args
    image_file_path = None
    if len(sys.argv) > 1:
        image_file_path = sys.argv[1]

    # camera_parameters = CameraParameter.from_parameters_file(RC_CAMERA_PARAMETERS_FILE_PATH)
    image_file_name_list = [file.split('.')[0] for file in get_files_with_extension(XMP_IMAGE_METADATA_FOLDER_PATH, '.xmp')]

    selected_reference_ids = None
    if image_file_path is None:
        selected_reference_ids = []
        for i in range(2):
            selected_reference_ids.append(random.choice(image_file_name_list))
    else:
        found_camera_parameter = False
        if image_file_path.split('/')[-1].split('.')[0] in image_file_name_list:
            selected_reference_ids = [image_file_path.split('/')[-1].split('.')[0]]
            found_camera_parameter = True

        if not found_camera_parameter:
            print(f'Camera parameter not found for {image_file_path}')
            return

    if selected_reference_ids is None: return

    # Reading point cloud
    pcd = o3d.io.read_point_cloud(RC_POINT_CLOUD_FILE_PATH)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    if len(selected_reference_ids) > 1:
        _, axes = plt.subplots(2, 3, figsize=(18, 9))
        axes_pairs = axes.reshape(2, 3)

        for i, reference_id in enumerate(selected_reference_ids):
            do_stuff_with_camera(reference_id, points, colors, axes_pairs[i][0], axes_pairs[i][1], axes_pairs[i][2])
    else:
        reference_id = selected_reference_ids[0]
        _, [color_ax, reference_ax, depth_ax] = plt.subplots(1, 3, figsize=(9, 9))
        do_stuff_with_camera(reference_id, points, colors, color_ax, reference_ax, depth_ax)
    
    plt.show()

if __name__ == "__main__":
    main()
