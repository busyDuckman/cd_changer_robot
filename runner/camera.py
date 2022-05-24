# ----------------------------------------------------------------------------------------
# Code to capture images from a web cam.
#
# Author: Warren Creemers
# See license.txt for license info.
# ----------------------------------------------------------------------------------------
import cv2
from cv2 import VideoCapture


def get_last_camera(max_cams_to_test=20):
    last_cam = None
    print("Finding cameras:")
    for cam_num in range(max_cams_to_test):
        print(f"  - looking for camera {cam_num}: ", end="")
        cam = get_camera(cam_num)
        if cam is None:
            print("(not found)")
            return last_cam
        last_cam = cam
        print(cam)
    return last_cam


def get_camera(cam_num) -> VideoCapture:
    if cam_num < 0:
        return get_last_camera()
    try:
        cam = cv2.VideoCapture(cam_num)
        result, image = cam.read()
        if result:
            return cam
        return None
    except cv2.error:
        return None


def get_image(cam):
    if cam is not None:
        result, image = cam.read()
        if result:
            print("  - get_image, took an image from the camera.")
            return image
        else:
            print("  - Error: get_image, result was False")
    else:
        print("  - Error: get_image, camera was None")
    return None


def save_image(img, image_file):
    if img is not None:
        print(f"  - saving file ({image_file}).")
        cv2.imwrite(image_file, img)
    else:
        print(f"  - Error: no image to save ({image_file}).")


def capture_and_save_image(cam, image_file):
    if cam is not None:
        save_image(get_image(cam), image_file)


if __name__ == '__main__':
    cam = get_camera(-1)
    img = get_image(cam)
    save_image(img, "c:\\share\\cd_robot\\test.jpg")