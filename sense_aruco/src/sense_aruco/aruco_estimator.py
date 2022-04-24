from __future__ import print_function
import cv2
import numpy as np
import os


class ArucoMarkerEstimator:

    def __init__(self, paramfilepath=None, family_name=None,
        marker_side_len=None):
        self.__calib_mat = None
        self.__calib_dst = None
        self.__aruco_dict = None
        self.__aruco_params = None
        self.__marker_side_len = None

        if paramfilepath is not None:
            self.load_camera_params(paramfilepath)

        if family_name is not None:
            self.set_marker_search_family(family_name)

        if marker_side_len is not None:
            self.set_marker_side_len(marker_side_len)
    #end def

    def detect(self, image, get_pose=False):
        """
        Estimates the IDs and four corners of each detected aruco marker in an 
        image. If no marker was not found, this function returns an empty list.
        """
        # Estimate the corners and IDs of the markers. Ignore rejected markers.
        corners, marker_ids, _ = cv2.aruco.detectMarkers(
            image, self.__aruco_dict, parameters=self.__aruco_params,
            cameraMatrix=self.__calib_mat, distCoeff=self.__calib_dst)

        # Get the number of corners.
        n_corners = len(corners)
        if n_corners == 0:
            return list()

        # Get the marker poses if requested.
        if get_pose and self.has_calib_params():
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.__marker_side_len, self.__calib_mat,
                self.__calib_dst)
        else:
            rvecs, tvecs = list([None] * n_corners), list([None] * n_corners)
        #end if

        # Aggregate all the markers into a list of dictionaries.
        marker_list = list()
        marker_ids = marker_ids.flatten()
        for m_corners, m_id, rvec, tvec \
            in zip(corners, marker_ids, rvecs, tvecs):
            data = dict()
            data['corners'] = m_corners.reshape((4,2)).astype(np.int32)
            data['mid'] = int(m_id)

            # Compute the marker pose if requested.
            if get_pose:
                pose = np.eye(4)
                pose[:,3] = tvec.flatten()
                pose[:3,:3], _ = cv2.Rodrigues(rvec.flatten())
                data['pose'] = pose
            #end if

            marker_list.append(data)
        #end for
        
        return marker_list
    #end def

    @staticmethod
    def get_family_dictionaries():
        """
        Returns a 
        """
        return dict({
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
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        })
    #end def

    def has_calib_params(self):
        return self.__calib_dst is not None and self.__calib_mat is not None

    def load_camera_params(self, paramfilepath):
        """
        Loads the camera calibration parameters from the specifed 
        """
        if not os.path.exists(paramfilepath):
            raise Exception("The calibration parameters file path " \
                + "({}) does not exist.".format(paramfilepath))
        #end if

        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(paramfilepath, cv2.FILE_STORAGE_READ)
        self.__calib_mat = cv_file.getNode('K').mat()
        self.__calib_dst = cv_file.getNode('D').mat()
        cv_file.release()
    #end def

    def set_marker_side_len(self, marker_side_len):
        """
        Sets the side length of the markers to observe.
        """
        if isinstance(marker_side_len, float) and marker_side_len > 0.:
            self.__marker_side_len = marker_side_len
        else:
            raise Exception("Marker side length must be a positive float.")
    #end def

    def set_marker_search_family(self, family_name):
        """
        Sets the marker family to search for in images.
        """
        family_dict = ArucoMarkerEstimator.get_family_dictionaries()
        if family_dict.get(family_name, None) is None:
            raise Exception("ArUCo tag of '{}' is not supported!".format(
                family_name))
        #end if

        self.__aruco_dict = cv2.aruco.Dictionary_get(family_dict[family_name])
        self.__aruco_params = cv2.aruco.DetectorParameters_create()
    #end def
#end class
