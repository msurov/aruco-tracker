#!/usr/bin/python

import argparse
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import numpy as np


colors = {
    'black': (0,0,0),
    'white': (255,255,255),
    'green': (0, 255, 0),
    'blue': (0, 0, 255),
    'red': (255, 0, 0),
}

def get_color(c):
    if c in colors:
        r,g,b = colors[c]
        return r * 256 * 256 + g * 256 + b
    elif type(c) == tuple:
        r,g,b = c
        return r * 256 * 256 + g * 256 + b
    elif type(c) == int:
        return c
    raise Exception('Can\'t convert color')

def svg_rect(x,y,w,h,c):
    return '''<rect
       style="fill:#%06x;fill-opacity:1;stroke:none;stroke-width:2;stroke-miterlimit:4;stroke-dasharray:none;stroke-dashoffset:0;stroke-opacity:1"
       width="%f"
       height="%f"
       x="%f"
       y="%f" />
    ''' % (get_color(c), w, h, x, y)

def svg_mat(m, sz):
    Ny, Nx = np.shape(m)

    rects = ''

    for y in range(Ny):
        for x in range(Nx):
            c = 'black' if m[y,x] == 0 else 'white'
            rects += svg_rect(x * sz, y * sz, sz, sz, c)

    return '''<?xml version="1.0" encoding="UTF-8" standalone="no"?> 
    <svg>
        <g>
            %s
        </g>
    </svg>''' % rects

def get_dict_cfg(s):
    values = {
        '4x4x100': aruco.DICT_4X4_100,
        '4x4x1000': aruco.DICT_4X4_1000,
        '4x4x250': aruco.DICT_4X4_250,
        '4x4x50': aruco.DICT_4X4_50,
        '5x5x100': aruco.DICT_5X5_100,
        '5x5x1000': aruco.DICT_5X5_1000,
        '5x5x250': aruco.DICT_5X5_250,
        '5x5x50': aruco.DICT_5X5_50,
        '6x6x100': aruco.DICT_6X6_100,
        '6x6x1000': aruco.DICT_6X6_1000,
        '6x6x250': aruco.DICT_6X6_250,
        '6x6x50': aruco.DICT_6X6_50,
        '7x7x100': aruco.DICT_7X7_100,
        '7x7x1000': aruco.DICT_7X7_1000,
        '7x7x250': aruco.DICT_7X7_250,
        '7x7x50': aruco.DICT_7X7_50
    }
    if s in values:
        return values[s]
    
    raise Exception('Unknown dictionary type: ' + s)


def getMarkerBitmask(dictionary, marker_id):
    return dictionary.drawMarker(marker_id, dictionary.markerSize + 2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--id', required=True, type=int, help='marker id')
    parser.add_argument('--dictcfg', required=True, type=str, help='aruco dictionary configuration HxWxN (for example 4x4x50, 5x5x100, etc)')
    parser.add_argument('--out', type=str, help='path to destination svg file')

    args = parser.parse_args()
    dict_type = get_dict_cfg(args.dictcfg)
    dictionary = aruco.getPredefinedDictionary(dict_type)
    m = getMarkerBitmask(dictionary, args.id)
    svg = svg_mat(m, 100)

    if args.out is not None:
        name = args.out
    else:
        name = 'aruco_%d.svg' % args.id

    f = open(name, 'w')
    f.write(svg)
    f.close()
