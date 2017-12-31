## Project: Perception Pick & Place

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify).
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points

[//]: # "Image References"

[confusion_matrix_1]: ./pictures/figure_1.png

[confusion_matrix_2]: ./pictures/figure_2.png

[pick_list_1]: ./pictures/pick_list_1.png

[pick_list_2]: ./pictures/pick_list_2.png

[pick_list_3]: ./pictures/pick_list_3.png


### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

- Downsampling voxel grid
    `LEAF_SIZE` is set as 0.005
```
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE = 0.005  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
```

- PassThrough Filter
    Create Passthrough filter in x and z axes
```
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = 0.5
    axis_max = 0.8
    passthrough.set_filter_limits (axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = 0.4
    axis_max = 0.8
    passthrough.set_filter_limits (axis_min, axis_max)
    cloud_filtered = passthrough.filter()
```

- Statistical Outlier Filtering
    Set the number of neighboring points 3 and threshold scale factor 0.1, any points with mean distance larger than (mean distance+x\*std_dev ) will be considered as outlier.
```
    cloud_filtered = cloud_filtered.make_statistical_outlier_filter()
    cloud_filtered.set_mean_k(3)
    cloud_filtered.set_std_dev_mul_thresh(.1)
    cloud_filtered = cloud_filtered.filter()
```

- RANSAC Plane Segmentation
```
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
```



#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
```
    white_cloud = XYZRGB_to_XYZ(cloud_objects)# Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.007)
    ec.set_MinClusterSize(30)
    ec.set_MaxClusterSize(3000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_table =  pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
```

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

- Features extracted

```
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
```

- SVM trained

  - In `features.py` (sensor_stick/src/sensor_stick):

    32 bins with range (0, 256) to compute color histograms,

    32 bins with range (-1, 1) to compute normal histograms.

  - In 'capture_features.py' (sensor_stick/scripts):

    100 features were captured for each object  to train SVM classifier.

  The confusion matrix without normalization is

![alt text][confusion_matrix_1]

  The normalized confusion matrix is

![alt text][confusion_matrix_2]

- Object Recognition

    Variables Initialization
```
    dict_list = []
    labels = []
    centroids = []
    object_list_param = []
    dropbox_param = []
    pick_position = []
    dropbox_position = []

    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
```

​     Read objects and dropbox params from yaml files
```
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
```

​     Loop through each object in the pick list, and ten assign the arm and 'place_pose' to be used for pick_place, create a list of dictionaries for later output to yaml file.
```
    for target in object_list:

        labels.append(target.label)
        points_arr = ros_to_pcl(target.cloud).to_array()
        pick_position = np.mean(points_arr, axis=0)[:3]
        pick_pose.position.x = np.float(pick_position[0])
        pick_pose.position.y = np.float(pick_position[1])
        pick_pose.position.z = np.float(pick_position[2])
        centroids.append(pick_position[:3])

        object_name.data = str(target.label)

		
        # Assign the arm and 'place_pose' to be used for pick_place
        object_group = object_list_param[0]['group']
        for index in range(0, len(object_list_param)):
            if object_list_param[index]['name'] == target.label:
                object_group = object_list_param[index]['group']
        for ii in range(0, len(dropbox_param)):
            if dropbox_param[ii]['group'] == object_group:
                arm_name.data = dropbox_param[ii]['name']
                dropbox_position = dropbox_param[ii]['position']

                place_pose.position.x = np.float(dropbox_position[0])
                place_pose.position.y = np.float(dropbox_position[1])
                place_pose.position.z = np.float(dropbox_position[2])

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)
```
### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.
- Output request parameters into output yaml file
```
    # Output request parameters into output yaml file
    yaml_filename = 'output_' + str(test_scene_num.data) + '.yaml'

    send_to_yaml(yaml_filename, dict_list)
```

- Object recognition results

  all objects in `pick_list_*.yaml` are correctly recognized, as shown in following. The `yaml` file can be found in (/output) folder.

  ![alt text][pick_list_1]
  ![alt text][pick_list_2]
  ![alt text][pick_list_3]

### Conclusion

To improve this project further, it will be useful to run the project in a native Linux enivronment, rather than VM -- due to much of the errors coming from lag, and the robot not fully grasping objects, or simply holding onto them for too long. There could also be more development done in detecting the glue object that appears in world2 and world3. It is a small object, so many of the filters keeping the noise out would often exclude it from our analysis.
