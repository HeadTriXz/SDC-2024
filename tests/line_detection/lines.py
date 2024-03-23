import numpy as np

from lane_assist.line_detection.line import Line, LineType

CORNER = [
    Line(
        np.array([[324, 892], [321, 884], [317, 876], [312, 868], [305, 860], [297, 852], [286, 844]]),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [480, 892],
                [480, 884],
                [478, 876],
                [477, 868],
                [475, 860],
                [473, 852],
                [471, 844],
                [468, 836],
                [465, 828],
                [461, 820],
                [456, 812],
                [450, 804],
                [444, 796],
                [438, 788],
                [431, 780],
                [424, 772],
                [416, 764],
                [408, 756],
                [398, 748],
                [387, 740],
                [375, 732],
                [363, 724],
                [349, 716],
                [334, 708],
                [316, 700],
                [297, 692],
                [276, 684],
                [252, 676],
            ]
        ),
        line_type=LineType.SOLID,
    ),
]

STRAIGHT = [
    Line(
        np.array(
            [
                [316, 892],
                [315, 884],
                [315, 876],
                [315, 868],
                [315, 860],
                [315, 852],
                [316, 844],
                [316, 836],
                [316, 828],
                [316, 820],
                [316, 812],
                [317, 804],
                [317, 796],
                [317, 788],
                [317, 780],
                [317, 772],
                [317, 764],
                [318, 756],
                [318, 748],
                [318, 740],
                [318, 732],
                [318, 724],
                [318, 716],
                [318, 708],
                [319, 700],
                [319, 692],
                [319, 684],
                [319, 676],
                [320, 668],
                [320, 660],
                [320, 652],
                [320, 644],
                [321, 636],
                [321, 628],
                [321, 620],
                [321, 612],
                [322, 604],
                [322, 596],
                [322, 588],
                [322, 580],
                [323, 572],
                [323, 564],
                [323, 556],
                [324, 548],
                [324, 540],
                [324, 532],
                [324, 524],
                [325, 516],
                [325, 508],
                [325, 500],
                [326, 492],
                [326, 484],
                [326, 476],
                [326, 468],
                [327, 460],
                [327, 452],
                [327, 444],
                [327, 436],
                [328, 428],
                [328, 420],
                [328, 412],
                [328, 404],
                [329, 396],
                [337, 100],
                [336, 92],
                [336, 84],
                [336, 76],
                [336, 68],
                [336, 60],
                [336, 52],
                [336, 44],
                [337, 36],
                [337, 28],
                [337, 20],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [466, 892],
                [467, 884],
                [468, 876],
                [468, 868],
                [469, 860],
                [469, 852],
                [470, 844],
                [470, 836],
                [470, 828],
                [471, 820],
                [471, 812],
                [472, 804],
                [472, 796],
                [473, 788],
                [473, 780],
                [474, 772],
                [474, 764],
                [474, 756],
                [475, 748],
                [475, 740],
                [476, 732],
                [476, 724],
                [477, 716],
                [477, 708],
                [478, 700],
                [478, 692],
                [478, 684],
                [479, 676],
                [479, 668],
                [480, 660],
                [480, 652],
                [480, 644],
                [481, 636],
                [481, 628],
                [482, 620],
                [482, 612],
                [482, 604],
                [483, 596],
                [483, 588],
                [484, 580],
                [484, 572],
                [485, 564],
                [485, 556],
                [485, 548],
                [486, 540],
                [486, 532],
                [487, 524],
                [487, 516],
                [488, 508],
                [488, 500],
                [489, 492],
                [489, 484],
                [490, 476],
                [490, 468],
                [490, 460],
                [491, 452],
                [491, 444],
                [492, 436],
                [492, 428],
                [492, 420],
                [493, 412],
                [494, 404],
                [494, 396],
                [512, 100],
                [512, 92],
                [513, 84],
                [513, 76],
                [513, 68],
                [513, 60],
                [514, 52],
                [514, 44],
                [515, 36],
                [515, 28],
                [516, 20],
            ]
        ),
        line_type=LineType.SOLID,
    ),
]

CROSSING = [
    Line(
        np.array(
            [
                [107, 548],
                [104, 540],
                [101, 532],
                [100, 524],
                [99, 516],
                [98, 508],
                [96, 500],
                [95, 492],
                [93, 484],
                [92, 476],
                [91, 468],
                [89, 460],
                [88, 452],
                [87, 444],
                [86, 436],
                [84, 428],
                [83, 420],
                [82, 412],
                [80, 404],
                [79, 396],
                [76, 388],
                [67, 380],
                [74, 372],
                [74, 364],
                [74, 356],
                [72, 348],
                [71, 340],
                [69, 332],
                [68, 324],
                [67, 316],
                [66, 308],
                [65, 300],
                [64, 292],
                [63, 284],
                [62, 276],
                [60, 268],
                [59, 260],
                [58, 252],
                [57, 244],
                [56, 236],
                [55, 228],
                [53, 220],
                [52, 212],
                [51, 204],
                [49, 196],
                [48, 188],
                [47, 180],
                [45, 172],
                [44, 164],
                [43, 156],
                [41, 148],
                [40, 140],
                [39, 132],
                [38, 124],
                [36, 116],
                [34, 108],
                [32, 100],
                [32, 92],
                [32, 84],
                [31, 76],
                [30, 68],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [328, 860],
                [328, 852],
                [328, 844],
                [327, 836],
                [327, 828],
                [326, 820],
                [297, 476],
                [295, 468],
                [294, 460],
                [293, 452],
                [292, 444],
                [291, 436],
                [290, 428],
                [287, 372],
                [286, 364],
                [285, 356],
                [284, 348],
                [283, 340],
                [283, 332],
                [282, 324],
                [281, 316],
                [278, 252],
                [277, 244],
                [276, 236],
                [275, 228],
                [274, 220],
                [273, 212],
                [273, 204],
                [272, 196],
                [269, 132],
                [268, 124],
                [267, 116],
                [266, 108],
                [265, 100],
                [264, 92],
                [263, 84],
                [262, 76],
                [261, 68],
                [260, 60],
            ]
        ),
        line_type=LineType.DASHED,
    ),
    Line(
        np.array(
            [
                [482, 892],
                [483, 884],
                [482, 876],
                [483, 868],
                [482, 860],
                [483, 852],
                [483, 844],
                [485, 836],
                [487, 828],
                [486, 820],
                [484, 812],
                [481, 804],
                [480, 796],
                [480, 788],
                [474, 564],
                [474, 556],
                [474, 548],
                [473, 540],
                [473, 532],
                [473, 524],
                [473, 516],
                [473, 508],
                [472, 500],
                [472, 492],
                [472, 484],
                [471, 476],
                [471, 468],
                [472, 460],
                [470, 452],
                [470, 444],
                [470, 436],
                [470, 428],
                [470, 420],
                [470, 412],
                [470, 404],
                [469, 396],
                [469, 388],
                [469, 380],
                [469, 372],
                [469, 364],
                [469, 356],
                [468, 348],
                [468, 340],
                [468, 332],
                [468, 324],
                [467, 316],
                [467, 308],
                [467, 300],
                [466, 292],
                [466, 284],
                [465, 276],
                [465, 268],
                [465, 260],
                [464, 252],
                [464, 244],
                [464, 236],
                [464, 228],
                [464, 220],
                [463, 212],
                [463, 204],
                [463, 196],
                [463, 188],
                [463, 180],
                [463, 172],
                [462, 164],
                [462, 156],
                [462, 148],
                [461, 140],
                [461, 132],
                [461, 124],
                [461, 116],
                [460, 108],
                [460, 100],
                [460, 92],
                [460, 84],
                [459, 76],
                [459, 68],
                [458, 60],
                [458, 52],
                [458, 44],
                [458, 36],
                [457, 28],
                [457, 20],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [718, 564],
                [719, 556],
                [719, 548],
                [719, 540],
                [720, 532],
                [721, 524],
                [722, 516],
                [723, 508],
                [724, 500],
                [714, 492],
            ]
        ),
        line_type=LineType.SOLID,
    ),
]

STOP_LINE = [
    Line(
        np.array(
            [
                [316, 892],
                [315, 884],
                [315, 876],
                [315, 868],
                [321, 860],
                [329, 852],
                [315, 844],
                [315, 836],
                [316, 828],
                [316, 820],
                [316, 812],
                [317, 804],
                [317, 796],
                [317, 788],
                [317, 780],
                [317, 772],
                [317, 764],
                [317, 756],
                [317, 748],
                [318, 740],
                [318, 732],
                [318, 724],
                [318, 716],
                [318, 708],
                [319, 700],
                [319, 692],
                [319, 684],
                [319, 676],
                [319, 668],
                [320, 660],
                [320, 652],
                [320, 644],
                [320, 636],
                [321, 628],
                [321, 620],
                [321, 612],
                [322, 604],
                [322, 596],
                [322, 588],
                [322, 580],
                [323, 572],
                [323, 564],
                [323, 556],
                [323, 548],
                [324, 540],
                [324, 532],
                [324, 524],
                [324, 516],
                [325, 508],
                [325, 500],
                [325, 492],
                [326, 484],
                [326, 476],
                [326, 468],
                [326, 460],
                [327, 452],
                [327, 444],
                [327, 436],
                [327, 428],
                [328, 420],
                [328, 412],
                [328, 404],
                [328, 396],
                [328, 388],
                [336, 100],
                [336, 92],
                [336, 84],
                [336, 76],
                [336, 68],
                [336, 60],
                [336, 52],
                [336, 44],
                [336, 36],
                [336, 28],
                [336, 20],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [467, 892],
                [467, 884],
                [468, 876],
                [468, 868],
                [469, 860],
                [456, 852],
                [470, 844],
                [470, 836],
                [470, 828],
                [471, 820],
                [472, 812],
                [472, 804],
                [472, 796],
                [473, 788],
                [473, 780],
                [474, 772],
                [474, 764],
                [475, 756],
                [475, 748],
                [475, 740],
                [476, 732],
                [477, 724],
                [477, 716],
                [477, 708],
                [478, 700],
                [478, 692],
                [479, 684],
                [479, 676],
                [479, 668],
                [480, 660],
                [480, 652],
                [481, 644],
                [481, 636],
                [481, 628],
                [482, 620],
                [482, 612],
                [483, 604],
                [483, 596],
                [483, 588],
                [484, 580],
                [484, 572],
                [485, 564],
                [485, 556],
                [486, 548],
                [486, 540],
                [487, 532],
                [487, 524],
                [488, 516],
                [488, 508],
                [488, 500],
                [489, 492],
                [489, 484],
                [490, 476],
                [490, 468],
                [491, 460],
                [491, 452],
                [492, 444],
                [492, 436],
                [493, 428],
                [493, 420],
                [493, 412],
                [494, 404],
                [494, 396],
                [495, 388],
                [513, 100],
                [513, 92],
                [513, 84],
                [514, 76],
                [513, 68],
                [514, 60],
                [514, 52],
                [515, 44],
                [515, 36],
                [515, 28],
                [516, 20],
            ]
        ),
        line_type=LineType.SOLID,
    ),
    Line(
        np.array(
            [
                [321, 857],
                [329, 857],
                [337, 857],
                [345, 857],
                [353, 857],
                [361, 857],
                [369, 857],
                [377, 857],
                [385, 857],
                [393, 857],
                [401, 857],
                [409, 857],
                [417, 857],
                [425, 857],
                [433, 857],
                [441, 857],
                [449, 857],
                [457, 857],
                [465, 857],
                [469, 857],
            ]
        ),
        line_type=LineType.STOP,
    ),
]
