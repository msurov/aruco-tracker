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
       width="%fmm"
       height="%fmm"
       x="%fmm"
       y="%fmm"/>
    ''' % (get_color(c), w, h, x, y)


def svg_text(x, y, text):
    templ = '''<text
       xml:space="preserve"
       style="font-style:normal;font-weight:normal;font-size:40px;line-height:125%%;font-family:sans-serif;letter-spacing:0px;word-spacing:0px;fill:#000000;fill-opacity:1;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"
       x="%fmm"
       y="%fmm">
       <tspan x="%fmm" y="%fmm" style="font-size:5mm">\n%s\n</tspan></text>\n'''
    return templ % (x, y, x, y, text)


def svg_mat(m, sz, p0=[0,0]):
    Ny, Nx = np.shape(m)
    pxsz = sz * 1. / Ny

    rects = ''

    for y in range(Ny):
        for x in range(Nx):
            c = 'black' if m[y,x] == 0 else 'white'
            rects += svg_rect(x * pxsz + p0[0], y * pxsz + p0[1], pxsz, pxsz, c)

    return '''<g>\n%s\n</g>''' % rects


def svg_dashed_rect(x,y,w,h):
    return '''<rect
       style="fill:none;stroke:#000000;stroke-width:0.99299997;stroke-miterlimit:4;stroke-dasharray:2,2;stroke-dashoffset:0"
       x="%fmm"
       y="%fmm"
       width="%fmm"
       height="%fmm"
       />''' % (x, y, w, h)



def svg_doc(content):
    template = '''<?xml version="1.0" encoding="UTF-8" standalone="no"?> 
        <svg
            width="110mm"
            height="140mm"
        >
            %s
        </svg>
    '''
    return template % (''.join(content))


def get_dict_cfg(s):
    values = {
        '4x4x100':  aruco.DICT_4X4_100,
        '4x4x1000': aruco.DICT_4X4_1000,
        '4x4x250':  aruco.DICT_4X4_250,
        '4x4x50':   aruco.DICT_4X4_50,
        '5x5x100':  aruco.DICT_5X5_100,
        '5x5x1000': aruco.DICT_5X5_1000,
        '5x5x250':  aruco.DICT_5X5_250,
        '5x5x50':   aruco.DICT_5X5_50,
        '6x6x100':  aruco.DICT_6X6_100,
        '6x6x1000': aruco.DICT_6X6_1000,
        '6x6x250':  aruco.DICT_6X6_250,
        '6x6x50':   aruco.DICT_6X6_50,
        '7x7x100':  aruco.DICT_7X7_100,
        '7x7x1000': aruco.DICT_7X7_1000,
        '7x7x250':  aruco.DICT_7X7_250,
        '7x7x50':   aruco.DICT_7X7_50
    }
    if s in values:
        return values[s]

    raise Exception('Unknown dictionary type: ' + s)


def getMarkerBitmask(dictionary, marker_id):
    return dictionary.drawMarker(marker_id, dictionary.markerSize + 2)


def gen_sample(dictionary, marker_id):
    sz = 80
    border = 15
    m = getMarkerBitmask(dictionary, marker_id)
    _mat = svg_mat(m, sz, p0=[border, border])
    _text = svg_text(border + sz / 2., border + sz * 1.2, marker_id)
    _doc = svg_doc([_mat, _text])
    return _doc


def gen_set(dictcfg, N):
    dict_type = get_dict_cfg(dictcfg)
    dictionary = aruco.getPredefinedDictionary(dict_type)

    for i in range(1,N+1):
        doc = gen_sample(dictionary, i)
        name = 'out/aruco_%d.svg' % i
        f = open(name, 'w')
        f.write(doc)
        f.close()


'''def test_():
    gen_set('5x5x250', 100)

test_()'''

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--id', required=True, type=int, help='marker id')
    parser.add_argument('--dictcfg', required=True, type=str, help='aruco dictionary configuration HxWxN (for example 4x4x50, 5x5x100, etc)')
    parser.add_argument('--out', type=str, help='path to destination svg file')
    parser.add_argument('--sz', type=float, help='size of pattern in mm')

    args = parser.parse_args()
    sz = 80 if args.sz is None else args.sz
    dict_type = get_dict_cfg(args.dictcfg)
    dictionary = aruco.getPredefinedDictionary(dict_type)
    m = getMarkerBitmask(dictionary, args.id)
    marker = svg_mat(m, sz, p0=[15,15])
    marker_id = svg_text(15 + sz / 2., 15 + sz * 1.2, args.id)
    doc = svg_doc([marker, marker_id])

    if args.out is not None:
        name = args.out
    else:
        name = 'aruco_%d.svg' % args.id

    f = open(name, 'w')
    f.write(doc)
    f.close()
