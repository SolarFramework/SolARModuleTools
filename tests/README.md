# SolARModuleTools

[![License](https://img.shields.io/github/license/SolARFramework/SolARModuleTools?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

**SolARModuleTools** is a module that implements some generic features used in vision processing, such as 2D or 3D transformations, matches between keypoints, homography checking, keyframe searching, 3D points filtering, etc.

SolARModuleTools is open-source, designed by [b<>com](https://b-com.com/en), under [Apache v2 licence](https://www.apache.org/licenses/LICENSE-2.0).

## Before running the tests

Some tests require external data which must be downloaded before executing them.

### Bag Of Word Vocabulary

This vocabulary is required for keyframe retrieval. Download the vocabularies required for the bag of words available on the [GitHub](https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip), and extract the `akaze.fbow` file and copy it in the `./data` folder.

### Maps

The loop closure tests will use a pre-built map which need to be download in your `./data` folder. You can use the map available on the SolAR artifactory.

Download the map [freiburg3_long_office_household.zip](https://repository.solarframework.org/generic/captures/singleRGB/TUM/freiburg3_long_office_household.zip), and extract the `./map` folder into the `./data` folder.

### Required modules

Some tests require other modules such as OpenGL, OpenCV, FBOW and G20. If they are not yet installed on your machine, please run the following command from the test folder:

<pre><code>remaken install packagedependencies.txt</code></pre>

and for debug mode:

<pre><code>remaken install packagedependencies.txt -c debug</code></pre>

For more information about how to install remaken on your machine, visit the [install page](https://solarframework.github.io/install/) on the SolAR website.

## The tests

### SolAR Test 3D Transform Estimation using RANSAC

Given two sets of 3D-3D point correspondences with noisy data, this test aims at estimating the 3D transformation (7-DoF) between them and defining inlier correspondences.

### SolAR test covisibility graph based on Boost library

This test creates a covisibility graph based on the boost library and evaluate different functions for example: create an edge, remove an edge, remove a node, find neighbors, find the shortest path between two nodes, find minimal spanning tree, save and load in the file.

### SolAR test covisibility graph

This test creates an our covisibility graph and evaluate different functions for example: create an edge, remove an edge, remove a node, find neighbors, find the shortest path between two nodes, find minimal spanning tree, save and load in the file.

### SolAR test mapper

This test creates two mappers that includes storage components in *Singleton* mode (e.g. point cloud manager, keyframe manager, covisibility graph, keyframe retriever).
Therefore, Only one instance of each component is created and two mappers share the same storage components.

### SolAR Test Point Cloud Manager

This test aims at creating a sample point cloud, then trying to save and load it from file.

### SolAR Test Loop closure detection

This test uses the prebuilt map to try to detect a loop closure for the last keyframe.
If a loop closure is detected, it will transform local point cloud of the last keyframe to the coordinate system of the loop detected keyframe.

### SolAR Test Loop Correction

This test uses the prebuilt map to try to detect a loop closure for the last keyframe.
If a loop closure is detected, it will perform the loop correction and the loop optimization based on the global bundle adjustment.
