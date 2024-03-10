import numpy as np

from lane_assist.line_detection import Line, LineType

CORNER = [
    Line(
        np.array(
            [
                [312, 882],
                [295, 864],
                [281, 846],
                [307, 720],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [477, 882],
                [473, 864],
                [467, 846],
                [457, 828],
                [445, 810],
                [430, 792],
                [413, 774],
                [393, 756],
                [370, 738],
                [345, 720],
            ]
        ),
        line_type=LineType.SOLID,
    ),
]

STRAIGHT = [
    Line(
        np.array(
            [
                [315, 882],
                [315, 864],
                [316, 846],
                [316, 828],
                [317, 810],
                [317, 792],
                [317, 774],
                [318, 756],
                [318, 738],
                [318, 720],
                [319, 702],
                [319, 684],
                [320, 666],
                [320, 648],
                [321, 630],
                [322, 612],
                [322, 594],
                [323, 576],
                [324, 558],
                [324, 540],
                [325, 522],
                [326, 504],
                [326, 486],
                [327, 468],
                [327, 450],
                [328, 432],
                [328, 414],
                [336, 108],
                [336, 90],
                [336, 72],
                [336, 54],
                [337, 36],
                [337, 18],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [468, 882],
                [469, 864],
                [470, 846],
                [471, 828],
                [472, 810],
                [473, 792],
                [474, 774],
                [475, 756],
                [476, 738],
                [477, 720],
                [478, 702],
                [479, 684],
                [480, 666],
                [481, 648],
                [482, 630],
                [483, 612],
                [484, 594],
                [485, 576],
                [486, 558],
                [487, 540],
                [488, 522],
                [489, 504],
                [490, 486],
                [491, 468],
                [492, 450],
                [493, 432],
                [493, 414],
                [512, 108],
                [513, 90],
                [513, 72],
                [514, 54],
                [515, 36],
                [516, 18],
            ]
        ),
        line_type=LineType.SOLID,
    ),
]
CROSSING = [
    Line(
        np.array(
            [
                [104, 558],
                [101, 540],
                [98, 522],
                [95, 504],
                [92, 486],
                [89, 468],
                [86, 450],
                [83, 432],
                [80, 414],
                [73, 396],
                [75, 378],
                [72, 360],
                [69, 342],
                [66, 324],
                [64, 306],
                [61, 288],
                [58, 270],
                [56, 252],
                [53, 234],
                [50, 216],
                [47, 198],
                [45, 180],
                [41, 162],
                [39, 144],
                [35, 126],
                [32, 108],
                [31, 90],
                [28, 72],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [328, 864],
                [327, 846],
                [326, 828],
                [297, 486],
                [293, 468],
                [291, 450],
                [290, 432],
                [286, 378],
                [284, 360],
                [282, 342],
                [281, 324],
                [277, 270],
                [276, 252],
                [274, 234],
                [272, 216],
                [271, 198],
                [268, 144],
                [266, 126],
                [264, 108],
                [262, 90],
                [260, 72],
            ]
        ),
        line_type=LineType.DASHED,
    ),
    Line(
        np.array(
            [
                [483, 882],
                [482, 864],
                [481, 846],
                [480, 828],
                [474, 558],
                [473, 540],
                [472, 522],
                [472, 504],
                [471, 486],
                [471, 468],
                [470, 450],
                [470, 432],
                [469, 414],
                [469, 396],
                [469, 378],
                [468, 360],
                [468, 342],
                [467, 324],
                [466, 306],
                [465, 288],
                [465, 270],
                [464, 252],
                [464, 234],
                [463, 216],
                [463, 198],
                [462, 180],
                [462, 162],
                [461, 144],
                [460, 126],
                [460, 108],
                [459, 90],
                [458, 72],
                [458, 54],
                [457, 36],
                [456, 18],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [655, 504],
                [645, 486],
                [642, 468],
                [643, 450],
                [644, 432],
                [644, 414],
                [645, 396],
                [646, 378],
                [645, 360],
                [647, 342],
                [646, 324],
                [654, 306],
                [655, 288],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [719, 558],
                [721, 540],
                [723, 522],
                [715, 504],
                [726, 486],
                [729, 468],
                [730, 450],
                [732, 432],
                [733, 414],
                [733, 396],
                [735, 360],
                [733, 324],
                [734, 306],
                [737, 288],
                [740, 270],
                [738, 252],
                [738, 234],
                [739, 216],
                [740, 198],
                [741, 180],
                [739, 162],
                [741, 144],
                [741, 126],
                [743, 108],
                [744, 72],
                [744, 36],
            ]
        ),
        line_type=LineType.SOLID,
    ),
]


STOP_LINE = [
    Line(
        np.array(
            [
                [315, 882],
                [324, 864],
                [316, 846],
                [316, 828],
                [317, 810],
                [317, 792],
                [317, 774],
                [318, 756],
                [318, 738],
                [318, 720],
                [319, 702],
                [319, 684],
                [320, 666],
                [320, 648],
                [321, 630],
                [322, 612],
                [322, 594],
                [323, 576],
                [323, 558],
                [324, 540],
                [324, 522],
                [325, 504],
                [326, 486],
                [326, 468],
                [327, 450],
                [328, 432],
                [328, 414],
                [336, 108],
                [336, 90],
                [336, 72],
                [336, 54],
                [336, 36],
                [336, 18],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [468, 882],
                [459, 864],
                [470, 846],
                [471, 828],
                [472, 810],
                [473, 792],
                [474, 774],
                [475, 756],
                [476, 738],
                [477, 720],
                [478, 702],
                [479, 684],
                [480, 666],
                [481, 648],
                [482, 630],
                [483, 612],
                [484, 594],
                [485, 576],
                [486, 558],
                [487, 540],
                [488, 522],
                [489, 504],
                [490, 486],
                [491, 468],
                [492, 450],
                [493, 432],
                [494, 414],
                [513, 108],
                [514, 90],
                [514, 72],
                [515, 54],
                [516, 36],
                [517, 18],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [315, 899],
                [333, 899],
                [351, 899],
                [369, 899],
                [387, 899],
                [405, 899],
                [423, 899],
                [441, 899],
                [459, 899],
                [468, 899],
            ]
        ),
        line_type=LineType.STOP,
    ),
]
