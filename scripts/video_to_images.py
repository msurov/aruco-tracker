from black import out
import cv2
import argparse
from os.path import exists, join, isdir, splitext
from os import mkdir


def main():
    parser = argparse.ArgumentParser(description='Split video into frames')
    parser.add_argument('--input', required=True, type=str, help='path to the input video file')
    parser.add_argument('--outfolder', required=False, type=str, help='path to the output folder to put frames to')
    args = parser.parse_args()

    input = args.input
    assert exists(input), f'Can\'t optn input file {input}'
    if args.outfolder is None:
        output = splitext(input)[0] + '-out'

    if not isdir(output):
        mkdir(output)

    cap = cv2.VideoCapture(input)
    assert cap.isOpened()

    i = -1

    while True:
        ok, img = cap.read()
        if not ok:
            break
        i += 1
        imname = '%06d.png' % i
        impath = join(output, imname)
        cv2.imwrite(impath, img)
        print('saved ', impath)

if __name__ == '__main__':
    main()
