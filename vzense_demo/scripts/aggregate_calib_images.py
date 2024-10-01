#!/usr/bin/env python3

import argparse

from pathlib import Path
import shutil
from pybsc import run_command
from pybsc.makedirs import makedirs
import rospkg


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-path', default='calib_results')
    parser.add_argument('--data-path', default='data_path')
    args = parser.parse_args()

    rospack = rospkg.RosPack()
    package_path = Path(rospack.get_path('vzense_demo'))
    target_path = package_path / args.results_path / 'Images'
    run_command(f'rm -rf {target_path}', shell=True)
    makedirs(target_path)
    makedirs(target_path / 'Cam_001')
    makedirs(target_path / 'Cam_002')
    img_count = 0
    for i, dir_path in enumerate((package_path / args.data_path).glob('*')):
        if not dir_path.is_dir():
            continue
        left_image_path = dir_path / 'left.jpg'
        right_image_path = dir_path / 'right.jpg'
        if left_image_path.exists() and right_image_path.exists():
            shutil.copy(left_image_path, target_path / 'Cam_001' / '{0:03}.jpg'.format(i + 1))
            shutil.copy(right_image_path, target_path / 'Cam_002' / '{0:03}.jpg'.format(i + 1))
            img_count += 1
    if img_count > 0:
        print(f"Successfully copied {img_count} images! You're all set!")
    else:
        print("No images were found. Please double-check the directories.")
