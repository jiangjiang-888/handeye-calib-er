#!/usr/bin/env python2
# coding: utf-8
import csv
import numpy as np


def read_handeye_data(path):
    hand = []
    eye = []
    reader = csv.reader(open(path, "r"))
    for line in reader:
        if line[0] == "hand":
            for d in line:
                if not str(d).isalpha():
                    hand.append(float(d))
        elif line[0] == "eye":
            for d in line:
                if not str(d).isalpha():
                    eye.append(float(d))
    return np.asarray(hand, dtype=float, order=None), np.asarray(eye, dtype=float, order=None)


