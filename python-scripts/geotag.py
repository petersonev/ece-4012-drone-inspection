# This program assumes you have exiftool installed
# This should be run after the sorting script
import os
import subprocess
import csv

def geotagging(img_base_path, csv_path, inspect_path):
    # Get the latitude and longitude for every file in the directory
    gps_lat = dict()
    gps_long = dict()
    for file in os.listdir(img_base_path):
        if file.endswith(".jpg"):
            # Get lat and long
            # print file
            cmd = "exiftool -c \"%.6f\" " + img_base_path + "/" + file + " | grep \"GPS Position\""
            output = subprocess.check_output(cmd, shell=True)
            output = output.split(b":")
            output = output[1]
            output = output.split(b",")
            lat = output[0]
            long = output[1]
            # print "lat:  " + lat
            # print "long: " + long

            # Make the filename a dictionary key
            str_tmp = file.split(".")
            i = int(str_tmp[0])
            # print i

            # Add the entry to the dictionary
            gps_lat[i] = lat
            gps_long[i] = long


    # sort the dictionary based on filename and add info to csv file
    keys_lat = gps_lat.keys()
    keys_long = gps_long.keys()
    keys_lat = sorted(keys_lat)
    keys_long = sorted(keys_long)
    sorted_lat = []     # These are what you need to add to the csv file
    sorted_long = []    # These are what you need to add to the csv file

    for key in keys_lat:
        lat_str = gps_lat[key]
        lat_str = lat_str[1:]
        sorted_lat.append(lat_str)

    for key in keys_long:
        long_str = gps_long[key]
        long_str = long_str[1:-1]
        sorted_long.append(long_str)

    csv_col_1 = ["Geotag Lat"] + sorted_lat
    csv_col_2 = ["Geotag Long"] + sorted_long

    # Add the JPG geotag latitude points to the csv file
    f = open(csv_path)
    data = [item for item in csv.reader(f)]
    f.close()

    new_data = []

    for i, item in enumerate(data):
        try:
            item.append(csv_col_1[i])
        except IndexError as e:
            item.append("placeholder")
        new_data.append(item)

    f = open(csv_path, 'w')
    csv.writer(f).writerows(new_data)
    f.close()


    # Add the JPG geotag longitude points to the csv file
    f = open(csv_path)
    data = [item for item in csv.reader(f)]
    f.close()

    new_data = []

    for i, item in enumerate(data):
        try:
            item.append(csv_col_2[i])
        except IndexError as e:
            item.append("placeholder")
        new_data.append(item)

    f = open(csv_path, 'w')
    csv.writer(f).writerows(new_data)
    f.close()


    # Extract the inspection pass/fail column
    f = open(inspect_path + ".csv")
    inspect_data = [item for item in csv.reader(f)]
    f.close()
    j = 0
    for cell in inspect_data:
        cell = str(cell)
        cell = cell[2:-2]
        inspect_data[j] = cell
        j = j+1
    inspect_data_old = inspect_data
    inspect_data = ["Inspection"] + inspect_data

    l = 0
    modified_inspect = inspect_data
    print(modified_inspect)
    for cell in modified_inspect:
        if cell == "TRUE":
            cell = "Damage Detected!"
        elif cell == "FALSE":
            cell = "No Damage Detected"
        modified_inspect[l] = cell
        l = l+1


    # Add the inspection pass/fail column to the csv file
    f = open(csv_path)
    data = [item for item in csv.reader(f)]
    f.close()

    new_data = []

    for i, item in enumerate(data):
        try:
            item.append(modified_inspect[i])
        except IndexError as e:
            item.append("placeholder")
        new_data.append(item)

    f = open(csv_path, 'w')
    csv.writer(f).writerows(new_data)
    f.close()


    # Record an overall pass/fail of the inspection
    # TRUE = there is a defect, the picture is bad
    # FALSE = there is no defect, the picture is good
    overall_inspect = inspect_data_old
    for cell in overall_inspect:
        if cell == "TRUE":
            k = 0
            for cell in overall_inspect:
                modified_cell = "TRUE"
                overall_inspect[k] = modified_cell
                k = k+1
            break

    overall_inspect = ["Overall Pass/Fail"] + overall_inspect

    l = 0
    modified_overall_inspect = overall_inspect
    for cell in modified_overall_inspect:
        if cell == "TRUE":
            cell = "Damage Detected!"
        elif cell == "FALSE":
            cell = "No Damage Detected"
        modified_overall_inspect[l] = cell
        l = l+1

    # Add an overall pass/fail column
    f = open(csv_path)
    data = [item for item in csv.reader(f)]
    f.close()

    new_data = []

    for i, item in enumerate(data):
        try:
            item.append(modified_overall_inspect[i])
        except IndexError as e:
            item.append("placeholder")
        new_data.append(item)

    f = open(csv_path, 'w')
    csv.writer(f).writerows(new_data)
    f.close()
