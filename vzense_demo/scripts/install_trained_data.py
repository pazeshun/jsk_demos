#!/usr/bin/env python

from __future__ import print_function

import argparse
import multiprocessing

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
        target=jsk_data.download_data,
        args=args,
        kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'vzense_demo'

    download_data(
        pkg_name=PKG,
        path='trained_data/yolo8/2024-10-02-grape.pt',
        url='https://drive.google.com/uc?id=1ie5H1A8ffTDm1mUycFcsYBs8wV-vA2yr',  # NOQA
        md5='4fe9135727ee321a99fd71bc2008abe2',
    )

    download_data(
        pkg_name=PKG,
        path='trained_data/yolo8/2024-10-02-grape_ncnn_model.tar.gz',
        url='https://drive.google.com/uc?id=1LSbBJGGAQ1cyATdHh8rcDNvwGYluSo-e',  # NOQA
        md5='ed42640dc0c7e8d5599a8110e552650e',
        extract=True,
    )


if __name__ == '__main__':
    main()
