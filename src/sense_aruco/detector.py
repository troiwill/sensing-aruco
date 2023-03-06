from __future__ import annotations
import cv2
import numpy as np
import os
import yaml


class ArucoDetector:
    def __init__(self) -> ArucoDetector:
        self.__calib_mat = None
        self.__calib_dst = None

        self.__aruco_dict = None
        self.__aruco_params = None
        self.__family_name = None

        self.__marker_side_len = None

        self.set_camera_params(calib_mat=np.eye(3), calib_dst=np.zeros((5,)))
        self.set_marker_search_family("DICT_ARUCO_ORIGINAL")
        self.set_marker_side_len(0.1)

        # Ensure we set the default estimation for now.
        self.__estimate_parameters = cv2.aruco.EstimateParameters().create()
        self.__estimate_parameters.pattern = cv2.aruco.CCW_center
        self.__estimate_parameters.useExtrinsicGuess = False
        self.__estimate_parameters.solvePnPMethod = cv2.SOLVEPNP_ITERATIVE

    @property
    def distortion_param(self) -> np.ndarray:
        """
        Returns the camera distortion coefficients.
        """
        return self.__calib_dst.flatten().copy()

    @property
    def family_name(self) -> str:
        """
        Returns the ArUco family name that the estimator will detect.
        """
        return self.__family_name

    @property
    def camera_matrix(self) -> np.ndarray:
        """
        Returns the camera intrinsics matrix (with shape (3,3)).
        """
        return self.__calib_mat.copy()

    @property
    def marker_side_length(self) -> float:
        """
        Returns the expected marker side length.
        """
        return self.__marker_side_len

    def detect(self, image: np.ndarray) -> np.ndarray | None:
        """
        Estimates the IDs and four corners of each detected aruco marker in an
        image. If no marker was not found, this function returns an empty list.
        """
        # Sanity check.
        if image is None:
            return None

        # Estimate the corners and IDs of the markers. Ignore rejected markers.
        corners, marker_ids, _ = cv2.aruco.detectMarkers(
            image=image, dictionary=self.__aruco_dict, parameters=self.__aruco_params
        )

        N_CORNERS = len(corners)
        if N_CORNERS == 0:
            return None

        dtypes = [("mid", np.uint32), ("corners", np.float64, (4, 2))]
        detections = np.empty((N_CORNERS,), dtype=dtypes)

        detections["mid"] = np.array(marker_ids).flatten()
        detections["corners"] = np.array(corners).reshape(-1, 4, 2)

        return detections

    def estimate_pose(self, detections: np.ndarray) -> np.ndarray | None:
        """
        Estimates the pose of the marker given an array of detections.
        """
        # Sanity checks.
        if detections is None:
            return None
        if len(detections) == 0:
            return None

        # Estimate the pose of all the seen markers.
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners=detections["corners"],
            markerLength=self.__marker_side_len,
            cameraMatrix=self.__calib_mat,
            distCoeffs=self.__calib_dst,
            estimateParameters=self.__estimate_parameters,
        )

        # Sanity check.
        if len(detections) != len(rvecs):
            raise Exception("Failed to estimate pose of all detected markers.")

        # Create an array of estimates.
        dtypes = [
            ("mid", np.uint32),
            ("tvec", np.float64, (3,)),
            ("rvec", np.float64, (3,)),
        ]
        estimates = np.empty((len(detections)), dtype=dtypes)
        estimates["mid"] = detections["mid"].copy()
        estimates["tvec"] = np.array(tvecs).reshape(-1, 3)
        estimates["rvec"] = np.array(rvecs).reshape(-1, 3)

        return estimates

    @staticmethod
    def get_family_dictionaries() -> dict[str, int]:
        """
        Returns a dictionary containing the Aruco families.
        """
        return dict(
            {
                "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
                "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
                "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
                "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
                "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
                "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
                "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
                "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
                "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
                "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
                "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
                "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
                "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
                "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
                "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
                "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
                "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            }
        )

    def load_camera_params(self, paramfilepath: str) -> None:
        """
        Loads the camera calibration parameters from the specifed
        """
        if not os.path.exists(paramfilepath):
            raise Exception(
                f"The calibration parameters file path ({paramfilepath}) does not exist."
            )
        # end if

        # Load the camera parameters from the saved file
        with open(paramfilepath, "r") as f:
            yamlfile = yaml.safe_load(f)
            calib_mat = np.array(yamlfile["K"]["data"]).reshape((3, 3))
            calib_dst = np.array(yamlfile["D"]["data"]).reshape((1, 5))

            self.set_camera_params(calib_mat=calib_mat, calib_dst=calib_dst)

    def set_camera_params(self, calib_mat: np.ndarray, calib_dst: np.ndarray) -> None:
        """
        Sets the camera calibration parameters.
        """
        assert type(calib_mat) is np.ndarray and calib_mat.shape == (3, 3)
        assert type(calib_dst) is np.ndarray and np.squeeze(calib_dst).shape == (5,)

        self.__calib_mat = calib_mat.copy()
        self.__calib_dst = calib_dst.reshape(1, 5).copy()

    def set_marker_side_len(self, marker_side_len: float) -> None:
        """
        Sets the side length of the markers to observe.
        """
        if isinstance(marker_side_len, float) and marker_side_len > 0.0:
            self.__marker_side_len = marker_side_len
        else:
            raise Exception("Marker side length must be a positive float.")

    def set_marker_search_family(self, family_name: str) -> None:
        """
        Sets the marker family to search for in images.
        """
        family_dict = ArucoDetector.get_family_dictionaries()
        if family_dict.get(family_name, None) is None:
            raise Exception(f"ArUco family '{family_name}' is not supported!")

        self.__family_name = family_name
        self.__aruco_dict = cv2.aruco.Dictionary_get(family_dict[family_name])
        self.__aruco_params = cv2.aruco.DetectorParameters_create()
