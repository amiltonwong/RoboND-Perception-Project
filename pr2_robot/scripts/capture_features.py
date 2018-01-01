#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    # Initialize number of poses per object
    num_poses = 100

    # Initialize number of attempts per capture
    num_attempts = 5

    # Initialze object list
    models = [\
       'biscuits',
       'soap',
       'soap2',
       'book',
       'glue',
       'sticky_notes',
       'snacks',
       'eraser']
    
    # Disable gravity and delete the ground plane
    initial_setup()

    # Initialize feature list
    labeled_features = []

    # Iterate over all objects to capture their features
    for model_name in models:
        spawn_model(model_name)

        for i in range(num_poses):

            # Try to create point cloud with a limited number of attempts
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < num_attempts:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected for pose {}.'.format(i))
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            chists = compute_color_histograms(sample_cloud, using_hsv=True)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])

        delete_model()

    # Save the captured features to a file
    file_name = "training_set.sav"
    pickle.dump(labeled_features, open(file_name, 'wb'))

