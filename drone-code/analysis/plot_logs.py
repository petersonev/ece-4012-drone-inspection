#!/usr/bin/env python3

import utm
import pandas
import sys
import os
import argparse
import glob
import matplotlib.pyplot as plt


def print_usage():
    print("Usage: plot_utm.py [path_to_log_dir] or plot_utm.py [path_to_file]")


def add_utm_cols(df):
    eastings = []
    northings = []
    for (lat, lon) in zip(df['lat'], df['lon']):
        # assume we aren't crossing zones, so don't use zone
        (easting, northing, zone, _) = utm.from_latlon(lat, lon)
        eastings.append(easting)
        northings.append(northing)

    df['easting'] = eastings
    df['northing'] = northings


def plot_file(filename):
    values = pandas.read_csv(filename, sep=' ', index_col=False)
    add_utm_cols(values)
    plt.plot(values['easting'], values['northing'])
    plt.axis('equal')
    plt.grid()
    plt.xlabel('Easting')
    plt.ylabel('Northing')
    plt.title("Plot of lat/lon converted to UTM")

    plt.show()


def plot_log_dir(input_path):
    # account for ex_ prefix, if this fails, you're missing one the files
    try:
        ideal_file = glob.glob(os.path.join(input_path,
                                            '*ideal_poselog.csv'))[0]
        actual_file = glob.glob(os.path.join(input_path, '*poselog.csv'))[0]
    except IndexError:
        print('Failed to find both ideal_poselog.csv and poselog.csv')
        print_usage()
        exit()

    print('Using files:', ideal_file, actual_file)

    ideal_values = pandas.read_csv(ideal_file, sep=' ', index_col=False)
    values = pandas.read_csv(actual_file, sep=' ', index_col=False)

    add_utm_cols(ideal_values)
    add_utm_cols(values)

    plt.plot(ideal_values['easting'], ideal_values['northing'], 'go')
    plt.plot(values['easting'], values['northing'])
    plt.axis('equal')
    plt.grid()
    plt.xlabel('Easting')
    plt.ylabel('Northing')
    plt.title("Plot of lat/lon converted to UTM")

    plt.show()


def main():
    # if handed a path, parse out both planned path and actual data, and plot
    if len(sys.argv) == 2 and os.path.isfile(sys.argv[1]):
        print('Single log file mode')
        plot_file(sys.argv[1])
        exit()
    elif len(sys.argv) == 1 or not os.path.isdir(sys.argv[1]):
        print_usage()
        exit()
    else:
        print('Multiple log dir mode')
        plot_log_dir(sys.argv[1])


if __name__ == "__main__":
    main()
