import matplotlib.pyplot as plt
import numpy as np
import cv2
import glob
from enum import IntEnum


#Stap 1: foto's weergeven
def list_images(images, cols=1, rows=3, cmap=None):
    plt.figure(figsize=(10, 11))
    for i, image in enumerate(images):
        plt.subplot(rows, cols, i+1)
        cmap = 'gray' if len(image.shape) == 2 else cmap
        plt.imshow(image, cmap=cmap, aspect='equal')  
        plt.xticks([])
        plt.yticks([])
    plt.tight_layout(pad=0, h_pad=0, w_pad=0)
    plt.show()

test_images = [plt.imread(img) for img in glob.glob('lanes/down_Images/*.jpg')]
list_images(test_images)



#Stap 2: horizon eraf kroppen, aantaal pixels van de bovenkant
def top_crop(test_image): # 350
    cropped_img = test_image[225:-750, :, :]  # Crop top - bottom

    return cropped_img

cropped_img = list(map(top_crop, test_images)) 
list_images(cropped_img)



#Stap 3: zwart wit maken
def white_filter(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)

    # zebrapad filter
    row_sum = np.sum(binary, axis=1) / 255 / binary.shape[1]
    binary[row_sum > 0.08, :] = 0
    
    return binary

black_white_images = list(map(white_filter, cropped_img)) 
list_images(black_white_images)   



#Stap 4: Canny edge + blur filter
def canny_detector(image, low_threshold=100, high_threshold=255):
    blurred = cv2.GaussianBlur(image, (21, 21), 3)
    edges = cv2.Canny(blurred, low_threshold, high_threshold)

    return edges

edge_detected_images = list(map(canny_detector, black_white_images))
list_images(edge_detected_images)



#Stap 5: Hough lines 
def detect_road_lines(image):
    #lines = cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=80, minLineLength=80, maxLineGap=60)
    lines = cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=100, minLineLength=100, maxLineGap=50)

    return lines

filtered_road_line_images = list(map(detect_road_lines, edge_detected_images))



#!! hough lines plotten !!
def draw_lines(image, lines, color=[255, 0, 0], thickness=15):

    image = np.copy(image)
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(image, (x1, y1), (x2, y2), color, thickness)
    return image

line_images = []
for image, lines in zip(cropped_img, filtered_road_line_images):
    line_images.append(draw_lines(image, lines)) 

list_images(line_images)



class LineType(IntEnum):
    Edge = 0
    Lane = 1
    Stop = 2

class Line:
    points: np.ndarray # [[x, y], [x, y], ...]
    type: LineType

    def __init__(self, points: np.ndarray, type: LineType) -> None:
        self.points = points
        self.type = type




def draw_left_and_right_lines(image, lines):
    image = np.copy(image)
    left_points = np.empty((0, 2))
    right_points = np.empty((0, 2))
    
    # Separating lines into left and right groups
    for line in lines:
        x1, y1, x2, y2 = line[0] 
        if (x2 - x1) == 0:
            continue  # Avoid division by zero
        slope = (y2 - y1) / (x2 - x1)
        if slope < 0:
            left_points = np.vstack((left_points, [[x1, y1], [x2, y2]]))
        elif slope > 0:
            right_points = np.vstack((right_points, [[x1, y1], [x2, y2]]))

    # Sonteren van Lage pixel naar hoge
    sorted_left_Points = left_points[left_points[:,1].argsort()[::-1]]
    sorted_right_Points = right_points[right_points[:,1].argsort()[::-1]]

    # Onderbroken lijn check
    if len(sorted_left_Points) < 13:
        left_line = Line(sorted_left_Points, LineType.Edge)
    else:
        left_line = Line(sorted_left_Points, LineType.Lane)

    if len(sorted_right_Points) < 13:
        right_line = Line(sorted_right_Points, LineType.Edge)
    else:
        right_line = Line(sorted_right_Points, LineType.Lane)
    



    print(len(sorted_left_Points))
    print(len(sorted_right_Points))


    # Draw left points in blue
    for point in left_points:
        cv2.circle(image, (int(point[0]), int(point[1])), 10, (255, 0, 0), -1)

    # Draw right points in red
    for point in right_points:
        cv2.circle(image, (int(point[0]), int(point[1])), 10, (0, 0, 255), -1)


    return image

# Call the function to draw left and right points
left_right_points_images = []
for image, lines in zip(cropped_img, filtered_road_line_images):
    left_right_points_images.append(draw_left_and_right_lines(image, lines))

# Display the images with left and right points
list_images(left_right_points_images)