# --- Sort Images ---
# Make this run on your system:
#    - Requires install of python pillow package (pip install pillow)

from PIL import Image
import piexif
import os
import shutil

# Put all of the .jpg files into a dictionary
def sortDirByTime(origPath, sortedPath):
    img_dict = dict()
    for file in os.listdir(origPath):
        if file.endswith(".jpg"):
            full_path = os.path.join(origPath, file)
            img = Image.open(full_path)
            exif_dict = piexif.load(img.info['exif'])
            date = exif_dict['Exif'][36867]
            img_dict[date] = full_path

    # Sort Dictionary by Date
    sorted_img_dict = sorted(img_dict.keys())

    # Make new path for sorted if not already exist
    if not os.path.exists(sortedPath):
        os.makedirs(sortedPath)

    # Sort the images based on the time stamp
    i = 1
    for key in sorted_img_dict:
        curr_path = img_dict[key]
        new_path = sortedPath + '/' + str(i) + ".jpg"
        shutil.copy(curr_path, new_path)
        i += 1