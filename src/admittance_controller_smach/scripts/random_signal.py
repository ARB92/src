#! /usr/bin/env python

from admittance_controller_smach.signal_generator import random_signal_generator

if __name__ == '__main__':
    try:
        random_signal_generator()
    except KeyboardInterrupt :
        exit(1)
