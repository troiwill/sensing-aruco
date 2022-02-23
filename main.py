########################################################################
#
# Copyright (c) 2021, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import sys
import numpy as np
import pyzed.sl as sl
import cv2
from cv2 import aruco
import pandas as pd
import matplotlib.pyplot as plt

help_string = "[s] Save side by side image [d] Save Depth, [n] Change Depth format, [p] Save Point Cloud, [m] Change Point Cloud format, [q] Quit"
prefix_point_cloud = "Cloud_"
prefix_depth = "Depth_"
path = "./"

count_save = 0
mode_point_cloud = 0
mode_depth = 0
point_cloud_format_ext = ".ply"
depth_format_ext = ".png"


def point_cloud_format_name():
    global mode_point_cloud
    if mode_point_cloud > 3:
        mode_point_cloud = 0
    switcher = {
        0: ".xyz",
        1: ".pcd",
        2: ".ply",
        3: ".vtk",
    }
    return switcher.get(mode_point_cloud, "nothing")


def depth_format_name():
    global mode_depth
    if mode_depth > 2:
        mode_depth = 0
    switcher = {
        0: ".png",
        1: ".pfm",
        2: ".pgm",
    }
    return switcher.get(mode_depth, "nothing")


def save_point_cloud(zed, filename):
    print("Saving Point Cloud...")
    tmp = sl.Mat()
    zed.retrieve_measure(tmp, sl.MEASURE.XYZRGBA)
    saved = (tmp.write(filename + point_cloud_format_ext) == sl.ERROR_CODE.SUCCESS)
    if saved:
        print("Done")
    else:
        print("Failed... Please check that you have permissions to write on disk")


def save_depth(zed, filename):
    print("Saving Depth Map...")
    tmp = sl.Mat()
    zed.retrieve_measure(tmp, sl.MEASURE.DEPTH)
    saved = (tmp.write(filename + depth_format_ext) == sl.ERROR_CODE.SUCCESS)
    if saved:
        print("Done")
    else:
        print("Failed... Please check that you have permissions to write on disk")


def save_sbs_image(zed, filename):
    image_sl_left = sl.Mat()
    zed.retrieve_image(image_sl_left, sl.VIEW.LEFT)
    image_cv_left = image_sl_left.get_data()

    image_sl_right = sl.Mat()
    zed.retrieve_image(image_sl_right, sl.VIEW.RIGHT)
    image_cv_right = image_sl_right.get_data()

    sbs_image = np.concatenate((image_cv_left, image_cv_right), axis=1)

    cv2.imwrite(filename, sbs_image)


def process_key_event(zed, key):
    global mode_depth
    global mode_point_cloud
    global count_save
    global depth_format_ext
    global point_cloud_format_ext

    if key == 100 or key == 68:
        save_depth(zed, path + prefix_depth + str(count_save))
        count_save += 1
    elif key == 110 or key == 78:
        mode_depth += 1
        depth_format_ext = depth_format_name()
        print("Depth format: ", depth_format_ext)
    elif key == 112 or key == 80:
        save_point_cloud(zed, path + prefix_point_cloud + str(count_save))
        count_save += 1
    elif key == 109 or key == 77:
        mode_point_cloud += 1
        point_cloud_format_ext = point_cloud_format_name()
        print("Point Cloud format: ", point_cloud_format_ext)
    elif key == 104 or key == 72:
        print(help_string)
    elif key == 115:
        print("1112321321111111111")
        save_sbs_image(zed, "ZED_image" + str(count_save) + ".png")
        count_save += 1
        save_point_cloud(zed, path + prefix_point_cloud + str(count_save))
        count_save += 1
        save_depth(zed, path + prefix_depth + str(count_save))
        count_save += 1
    else:
        a = 0


def print_help():
    print(" Press 's' to save Side by side images")
    print(" Press 'p' to save Point Cloud")
    print(" Press 'd' to save Depth image")
    print(" Press 'm' to switch Point Cloud format")
    print(" Press 'n' to switch Depth format")


def main():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    input_type = sl.InputType()
    if len(sys.argv) >= 2:
        input_type.set_from_svo_file(sys.argv[1])
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        zed.close()
        exit(1)

    # Display help in console
    print_help()

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width / 2
    image_size.height = image_size.height / 2

    # Declare your sl.Mat matrices
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    point_cloud = sl.Mat()
    take_picture = 0
    key = ' '
    while key != 113:
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image, depth image in the half-resolution
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
            # Retrieve the RGBA point cloud in half resolution
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, image_size)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv = image_zed.get_data()
            gray = cv2.cvtColor(image_ocv, cv2.COLOR_RGB2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            corners2 = np.array([c[0] for c in corners])
        #    print(corners2)
            depth_image_ocv = depth_image_zed.get_data()
#            cv2.imshow("Image", image_ocv)
#            cv2.imshow("Depth", depth_image_ocv)
            data = pd.DataFrame({"x": corners2[:, :, 0].flatten(), "y": corners2[:, :, 1].flatten()},
                                index=pd.MultiIndex.from_product(
                                    [ids.flatten(), ["c{0}".format(i) for i in np.arange(4) + 1]],
                                    names=["marker", ""]))

            data = data.unstack().swaplevel(0, 1, axis=1).stack()
            data["m1"] = data[["c1", "c2"]].mean(axis=1)
            data["m2"] = data[["c2", "c3"]].mean(axis=1)
            data["m3"] = data[["c3", "c4"]].mean(axis=1)
            data["m4"] = data[["c4", "c1"]].mean(axis=1)
            data["o"] = data[["m1", "m2", "m3", "m4"]].mean(axis=1)
            print(data)
            count_save = 9
#            if take_picture == 0:
#                save_sbs_image(zed, "ZED_image" + str(count_save) + ".png")
#                save_point_cloud(zed, path + prefix_point_cloud + str(count_save))
#                save_depth(zed, path + prefix_depth + str(count_save))
#                take_picture = take_picture + 1
            key = cv2.waitKey(10)

            process_key_event(zed, key)

    cv2.destroyAllWindows()
    zed.close()

    print("\nFINISH")


def read_AR_marker():
    frame = cv2.imread("ZED5.png")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    plt.figure()
    plt.imshow(frame_markers)
    for i in range(len(ids)):
        c = corners[i][0]
        plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label="id={0}".format(ids[i]))
    plt.legend()
    plt.show()

    corners2 = np.array([c[0] for c in corners])
    print(corners2)
    data = pd.DataFrame({"x": corners2[:, :, 0].flatten(), "y": corners2[:, :, 1].flatten()},
                        index=pd.MultiIndex.from_product(
                            [ids.flatten(), ["c{0}".format(i) for i in np.arange(4) + 1]],
                            names=["marker", ""]))

    data = data.unstack().swaplevel(0, 1, axis=1).stack()
    data["m1"] = data[["c1", "c2"]].mean(axis=1)
    data["m2"] = data[["c2", "c3"]].mean(axis=1)
    data["m3"] = data[["c3", "c4"]].mean(axis=1)
    data["m4"] = data[["c4", "c1"]].mean(axis=1)
    data["o"] = data[["m1", "m2", "m3", "m4"]].mean(axis=1)
    print(data)


if __name__ == "__main__":
    read_AR_marker()
#    main()


def quad_area(data):
    l = data.shape[0] // 2
    corners = data[["c1", "c2", "c3", "c4"]].values.reshape(l, 2, 4)
    c1 = corners[:, :, 0]
    c2 = corners[:, :, 1]
    c3 = corners[:, :, 2]
    c4 = corners[:, :, 3]
    e1 = c2 - c1
    e2 = c3 - c2
    e3 = c4 - c3
    e4 = c1 - c4
    a = -.5 * (np.cross(-e1, e2, axis=1) + np.cross(-e3, e4, axis=1))
    return a
