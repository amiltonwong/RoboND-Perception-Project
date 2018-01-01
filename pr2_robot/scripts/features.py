import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


def histogram(ch_0, ch_1, ch_2, nbins, bins_range):
    
    # Compute the histogram of each channel separately
    hist_0 = np.histogram(ch_0, bins=nbins, range=bins_range)
    hist_1 = np.histogram(ch_1, bins=nbins, range=bins_range)
    hist_2 = np.histogram(ch_1, bins=nbins, range=bins_range)
    
    # Concatenate the histograms into a single feature vector
    features = np.concatenate((hist_0[0], hist_1[0], hist_2[0])).astype(np.float64)
    
    # Normalize the result and return
    return features / np.sum(features)


def compute_color_histograms(cloud, using_hsv=True):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    ch_0 = [color[0] for color in point_colors_list]
    ch_1 = [color[1] for color in point_colors_list]
    ch_2 = [color[2] for color in point_colors_list]
    
    return histogram(ch_0, ch_1, ch_2, nbins=32, bins_range=(0, 256))


def compute_normal_histograms(normal_cloud):

    # Convert point cloud to array
    fields = ('normal_x', 'normal_y', 'normal_z')
    normals = pc2.read_points(normal_cloud, field_names=fields, skip_nans=True)

    # Create component arrays for each feature vector
    norm_x = [vector[0] for vector in normals]
    norm_y = [vector[1] for vector in normals]
    norm_z = [vector[2] for vector in normals]

    return histogram(norm_x, norm_y, norm_z, nbins=32, bins_range=(-1, 1))
