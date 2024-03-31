import numpy as np


def stitch_images(base_image: np.ndarray, new_image: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """Merge a new image onto a base image with the given offset.

    :param base_image: The base image onto which the new image will be merged.
    :param new_image: The new image to merge onto the base image.
    :param offset: The offset for the new image.
    :return: The merged image.
    """
    # Calculate dimensions for merging
    new_height, new_width = new_image.shape[:2]
    base_height, base_width = base_image.shape[:2]
    offset_x, offset_y = offset

    # Calculate the region of interest (ROI) for merging
    roi_top = max(offset_y, 0)
    roi_bottom = min(offset_y + new_height, base_height)
    roi_left = max(offset_x, 0)
    roi_right = min(offset_x + new_width, base_width)

    # Calculate the cropped region of the new image
    crop_top = roi_top - offset_y
    crop_bottom = crop_top + (roi_bottom - roi_top)
    crop_left = roi_left - offset_x
    crop_right = crop_left + (roi_right - roi_left)

    image = new_image[crop_top:crop_bottom, crop_left:crop_right]
    image_mask = image != 0

    # Merge the images
    merged_image = np.copy(base_image)
    merged_image[roi_top:roi_bottom, roi_left:roi_right][image_mask] = image[image_mask]

    return merged_image
