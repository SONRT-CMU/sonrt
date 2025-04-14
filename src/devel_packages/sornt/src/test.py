import argparse
from frankapy import FrankaArm

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_pose', '-u', action='store_true')
    parser.add_argument('--close_grippers', '-c', action='store_true')
    args = parser.parse_args()

    print('Starting robot')
    fa = FrankaArm()

    
    print('Closing Grippers')
    fa.close_gripper()
