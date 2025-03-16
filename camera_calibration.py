import xml.etree.ElementTree as ET
import numpy as np
from dataclasses import dataclass

@dataclass
class CameraCalibration:
    version: str
    pose_prior: str
    coordinates: str
    distortion_model: str
    focal_length_35mm: float
    skew: float
    aspect_ratio: float
    principal_point_u: float
    principal_point_v: float
    calibration_prior: str
    calibration_group: int
    distortion_group: int
    in_texturing: int
    in_meshing: int
    
    # Camera matrices
    rotation_matrix: np.ndarray
    position: np.ndarray
    distortion_coefficients: np.ndarray

    @classmethod
    def from_xmp(cls, xml_string: str) -> 'CameraCalibration':
        # Define namespaces
        namespaces = {
            'x': 'adobe:ns:meta/',
            'rdf': 'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
            'xcr': 'http://www.capturingreality.com/ns/xcr/1.1#'
        }
        
        # Parse XML
        root = ET.fromstring(xml_string)
        
        # Find the description element
        description = root.find('.//rdf:Description', namespaces)
        
        # Extract rotation matrix
        rotation_elem = description.find('./xcr:Rotation', namespaces)
        rotation_values = [float(x) for x in rotation_elem.text.split()]
        rotation_matrix = np.array(rotation_values).reshape(3, 3)
        
        # Extract position
        position_elem = description.find('./xcr:Position', namespaces)
        position_values = [float(x) for x in position_elem.text.split()]
        position = np.array(position_values)
        
        # Extract distortion coefficients
        distortion_elem = description.find('./xcr:DistortionCoeficients', namespaces)
        distortion_values = [float(x) for x in distortion_elem.text.split()]
        distortion_coefficients = np.array(distortion_values)
        
        # Create and return the data class instance
        return cls(
            version=description.attrib.get(f'{{{namespaces["xcr"]}}}Version'),
            pose_prior=description.attrib.get(f'{{{namespaces["xcr"]}}}PosePrior'),
            coordinates=description.attrib.get(f'{{{namespaces["xcr"]}}}Coordinates'),
            distortion_model=description.attrib.get(f'{{{namespaces["xcr"]}}}DistortionModel'),
            focal_length_35mm=float(description.attrib.get(f'{{{namespaces["xcr"]}}}FocalLength35mm')),
            skew=float(description.attrib.get(f'{{{namespaces["xcr"]}}}Skew')),
            aspect_ratio=float(description.attrib.get(f'{{{namespaces["xcr"]}}}AspectRatio')),
            principal_point_u=float(description.attrib.get(f'{{{namespaces["xcr"]}}}PrincipalPointU')),
            principal_point_v=float(description.attrib.get(f'{{{namespaces["xcr"]}}}PrincipalPointV')),
            calibration_prior=description.attrib.get(f'{{{namespaces["xcr"]}}}CalibrationPrior'),
            calibration_group=int(description.attrib.get(f'{{{namespaces["xcr"]}}}CalibrationGroup')),
            distortion_group=int(description.attrib.get(f'{{{namespaces["xcr"]}}}DistortionGroup')),
            in_texturing=int(description.attrib.get(f'{{{namespaces["xcr"]}}}InTexturing')),
            in_meshing=int(description.attrib.get(f'{{{namespaces["xcr"]}}}InMeshing')),
            rotation_matrix=rotation_matrix,
            position=position,
            distortion_coefficients=distortion_coefficients
        )

    @classmethod
    def from_file(cls, file_path: str) -> 'CameraCalibration':
        with open(file_path, 'r') as file:
            xml_string = file.read()
        return cls.from_xmp(xml_string)
