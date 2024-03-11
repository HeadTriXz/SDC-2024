import numpy as np
from matplotlib import pyplot as plt


def list_images(images: list[np.ndarray], cols: int = 1, rows: int = 4, cmap: str = None) -> None:
    """Plot a list of images."""
    plt.figure(figsize=(10, 11))
    for i, image in enumerate(images):
        plt.subplot(rows, cols, i + 1)
        cmap = "gray" if len(image.shape) == 2 else cmap
        plt.imshow(image, cmap=cmap, aspect="equal")
    plt.tight_layout(pad=0, h_pad=0, w_pad=0)
    plt.show()


def cut_image(image: np.ndarray, x: int, y: int, width: int, height: int) -> np.ndarray:
    """Cut image to specified width and height."""
    return image[y : y + height, x : x + width]
