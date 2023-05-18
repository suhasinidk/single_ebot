#!/usr/bin/env python

import math
import os
import re
import sys

_DUR = 1
_SUB_SKILL = 4
_STATE = 6
_CODE = 7
_MSG = 8
_SUB_SKILLS = [
    'ApplyPaintPrimitive',
    'ArmToZeroPrimitive',
    'LoadPaintPrimitive',
    'NavigateToLocationPrimitive',
    'task_plan']
_LOG_RE = re.compile('(.*;){8,8}.*')
_NAV_RE = re.compile('Navigating \(\((.*), (.*)\), (.*)\) to \(\((.*), (.*)\), (.*)\) \[Pose: \(\((.*), (.*)\), (.*)\), Dist: (.*)\]')


def read_entry(line, f):
    if _LOG_RE.match(line):
        data = line.split(';')
        line = f.readline()
        if not line:
            return data, line
        while not _LOG_RE.match(line):
            data[_MSG] += line
            line = f.readline()
        return data, line
    return None


def write_header(f):
    f.write('test,angle,skill,code,duration,delta_st,tot_dist,rel_diff_dist,delta_dest,delta_head\n')


def write_stats(stats, f):
    f.write(','.join('{}' for i in range(len(stats))).format(*stats) + '\n')


def calc_dist(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def norm_angle(theta):
    while theta > math.pi:
        theta -= math.pi
    while theta < -math.pi:
        theta += math.pi
    return theta


if len(sys.argv) < 3:
    print('Input directory and output directory required')
    sys.exit(1)

indir = sys.argv[1]
outdir = sys.argv[2]
_, _, filenames = next(os.walk(indir))

for filename in filenames:
    print(f'Parsing {indir}/{filename}...')

    test_name = filename.split('.')[-2]
    with open(f'{indir}/{filename}') as fin, open(f'{outdir}/{test_name}.csv', 'w') as fout:
        write_header(fout)

        track_data = None
        line = fin.readline()
        while line:
            data, line = read_entry(line, fin)
            stats = [test_name, filename.split('_')[1], track_data[_SUB_SKILL], int(track_data[_CODE]), float(track_data[_DUR])] if track_data else []

            if data[_STATE] == 'Failure':
                write_stats(stats, fout)
            elif track_data and data[_SUB_SKILL] != track_data[_SUB_SKILL]:
                if track_data[_SUB_SKILL] == 'NavigateToLocationPrimitive':
                    m = _NAV_RE.match(track_data[_MSG])
                    start, dest = (float(m.group(1)), float(m.group(2))), (float(m.group(4)), float(m.group(5)))
                    delta_start, total_dist = calc_dist(start, dest), float(m.group(10))
                    stats.append(delta_start)  # Euclidean distance between start and dest
                    stats.append(total_dist)  # Total distance travelled
                    stats.append((total_dist - delta_start) / delta_start)  # Rel. diff. of total_dist and delta_start
                    stats.append(calc_dist((float(m.group(7)), float(m.group(8))), dest))  # Euclidean distance between dest and final pose
                    stats.append(norm_angle(abs(abs(float(m.group(6))) - abs(float(m.group(9))))))  # Diff. between dest heading and final heading
                write_stats(stats, fout)

            if data[_SUB_SKILL] in _SUB_SKILLS and data[_STATE] in ('Running', 'Success'):
                track_data = data
            else:
                track_data = None
