# Takes in ground_truths path, inspection path, and damages path
# Creates directory with potential damages highlighted and EXIF data modified to show damages

import argparse
import cv2
import os
from minority_color_detection import findMinColor
from sort import sortDirByTime
from geotag import geotagging

ap = argparse.ArgumentParser()
ap.add_argument("-gt", "--ground_truth", required=True, help="Path to the ground truth images.")
ap.add_argument("-gts", "--ground_truth_sorted", required=True, help="Path to the ground truth images.")
ap.add_argument("-i", "--inspection", required=True, help="Path to the inspection images.")
ap.add_argument("-is", "--inspection_sorted", required=True, help="Path to the ground truth images.")
ap.add_argument("-d", "--damage", required=True, help="Path for the damage images.")
ap.add_argument("-csv", "--csv_name", required=True, help="Path and name of the CSV file")
ap.add_argument("-wp", "--waypoint_file", required=True, help="Path to waypoint csv file.")
args = vars(ap.parse_args())

gt_path = args["ground_truth"]
gt_sorted_path = args["ground_truth_sorted"]
insp_path = args["inspection"]
insp_sorted_path = args["inspection_sorted"]
dmg_path = args["damage"]
csv_name = args["csv_name"]
waypoint_csv = args["waypoint_file"]

# Make new dirs if not already exist
if not os.path.exists(gt_sorted_path):
    os.makedirs(gt_sorted_path)
if not os.path.exists(insp_sorted_path):
    os.makedirs(insp_sorted_path)
if not os.path.exists(dmg_path):
    os.makedirs(dmg_path)

# Sort images in dirs by time-date taken.
sortDirByTime(gt_path, gt_sorted_path)
sortDirByTime(insp_path, insp_sorted_path)

# Generate file list
gt_files = []
for dirName, subdirList, fileList in os.walk(gt_sorted_path):
    for fname in fileList:
        gt_files.append(fname)
insp_files = []
for dirName, subdirList, fileList in os.walk(insp_sorted_path):
    for fname in fileList:
        insp_files.append(fname)

# Iterate through images to check for damage
csv_to_write = ""
for i in range(len(gt_files)):

    # Load in the corisponding images
    imgt = cv2.imread(gt_sorted_path + '/' + gt_files[i])
    iminsp = cv2.imread(insp_sorted_path + '/' + insp_files[i])

    # Find min colors
    min_range_gt, _ = findMinColor(imgt)
    min_range_insp, iminsp_damage = findMinColor(iminsp)

    if min_range_gt != min_range_insp:
        csv_to_write = csv_to_write + "True\n"
        cv2.imwrite(dmg_path + '/' + insp_files[i], iminsp_damage)
    else:
        csv_to_write = csv_to_write + "False\n"

# Write csv file
file = open(csv_name + '.csv', 'w')
file.write(csv_to_write)
file.close()

# Update waypoint csv for website
geotagging(insp_sorted_path, waypoint_csv, csv_name)
