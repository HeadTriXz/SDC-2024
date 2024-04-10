from config import config
from constants import CameraResolution
from lane_assist.preprocessing.calibrate import CameraCalibrator
from utils.video_stream import VideoStream


def calibrate_cameras() -> None:
    """A script to calibrate the cameras."""
    cam_left = VideoStream(config.camera_ids.left, resolution=CameraResolution.FHD)
    cam_center = VideoStream(config.camera_ids.center, resolution=CameraResolution.FHD)
    cam_right = VideoStream(config.camera_ids.right, resolution=CameraResolution.FHD)

    cam_left.start()
    cam_center.start()
    cam_right.start()

    if not cam_left.has_next() or not cam_center.has_next() or not cam_right.has_next():
        raise ValueError("Could not capture images from cameras")

    left_image = cam_left.next()
    center_image = cam_center.next()
    right_image = cam_right.next()

    calibrator = CameraCalibrator([left_image, center_image, right_image], input_shape=(1280, 720))
    calibrator.calibrate()
    calibrator.save(config.calibration.save_dir)

    cam_left.stop()
    cam_center.stop()
    cam_right.stop()


if __name__ == "__main__":
    calibrate_cameras()
