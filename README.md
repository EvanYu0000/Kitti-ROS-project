
# Kitti-ROS-project


![Build Status](https://github.com/subsurface/subsurface/workflows/Ubuntu%2020.04%20/%20Qt%205.12--/badge.svg)  ![[build](https://img.shields.io/appveyor/ci/:user/:repo.svg)](https://img.shields.io/ros/v/noetic/moveit_msgs.svg)

## Introduction
Welcome to the Self-Driving Car Project, a comprehensive implementation based on the KITTI dataset within the ROS-Python framework.

![https://hackmd.io/_uploads/HksawMgQT.gif](https://user-images.githubusercontent.com/124876411/279774938-49fca8df-8021-42da-8924-bb2e1b1ca753.gif)

---

## Contents


[**Kitti-ROS-project**](https://github.com/EvanYu0000/Kitti-ROS-project#kitti-ros-project) [***(README on Hackmd)***](https://hackmd.io/@WMskyQDVTS-TnuM3hxjBZQ/HyuNfsyQa)
- [Contents](https://github.com/EvanYu0000/Kitti-ROS-project#contents)
- [Dependencies](https://github.com/EvanYu0000/Kitti-ROS-project#dependencies)
- [Dataset Quick Overview (KITTI)](https://github.com/EvanYu0000/Kitti-ROS-project#dataset-quick-overview-kitti)
- [Tests on Jupyter](https://github.com/EvanYu0000/Kitti-ROS-project#tests-on-jupyter)
	- [car_distance.ipynb](https://github.com/EvanYu0000/Kitti-ROS-project#car_distanceipynb)
	- [car_tracking.ipynb](https://github.com/EvanYu0000/Kitti-ROS-project#car_trackingipynb)
	- [kitti_plot2D.ipynb](https://github.com/EvanYu0000/Kitti-ROS-project#kitti_plot2dipynb)
	- [kitti_plot3D.ipynb](https://github.com/EvanYu0000/Kitti-ROS-project#kitti_plot3dipynb)
- [Source code](https://github.com/EvanYu0000/Kitti-ROS-project#source-code)
- [Demo](https://github.com/EvanYu0000/Kitti-ROS-project#demo)
- [Resources](https://github.com/EvanYu0000/Kitti-ROS-project#resources)
- [License](https://github.com/EvanYu0000/Kitti-ROS-project#license)


---




## Dependencies


```gherkin=
os
cv2
numpy
pandas
rospy
```
---

## Dataset Quick Overview (KITTI)

The dataset is structured to provide a comprehensive understanding of the KITTI data, which is an essential resource for various computer vision and autonomous driving applications. The dataset comprises several key components, each serving a unique purpose:
```
.
│── a_data
    ├── RawData
    │   └── 2011_09_26
    │       └── 2011_09_26_drive_0005_sync
    │           ├── image_00
    │           ├── image_01
    │           ├── image_02
    │           ├── image_03
    │           ├── oxts
    │           └── velodyne_points
    └── tracking
        └── training
            └── label_02
```



* Image Data: These high-resolution images serve as the foundation for 2D visual analysis, providing rich information about the scene's appearance. They are crucial for tasks like object detection, classification, and tracking.

* Tracking Label: The tracking labels associated with the image data offer essential annotations for object tracking and localization. They provide valuable information for tracking objects in the 2D space.

* Velodyne Points: The Velodyne LiDAR data is instrumental in reconstructing a comprehensive 3D representation of the environment. Leveraging point cloud data from Velodyne LiDAR, it allows for a detailed 3D reconstruction of the scene, making it indispensable for 3D object detection and scene understanding.

* OXTS Data: The OXTS data, obtained from onboard sensors, is used for precise positioning and orientation estimation. It also aids in estimating the relative distances between objects in the environment, contributing to a deeper understanding of the spatial relationships between objects.

---

## Tests on Jupyter

```
│── a_jupyter
    ├── car_distance.ipynb
    ├── car_tracking.ipynb
    ├── kitti_plot2D.ipynb
    └── kitti_plot3D.ipynb
```
---
### car_distance.ipynb
To ensure safety and efficient navigation, it is crucial to detect and quantify the distance between our own vehicle and other objects in the vicinity. In our final test, our objective is to calculate the shortest distance from our vehicle to other detected bounding boxes. This distance measurement is pivotal in making informed decisions to maintain a safe and collision-free environment during autonomous operations.
![https://hackmd.io/_uploads/ByE8a1g7T.png](https://user-images.githubusercontent.com/124876411/279774777-a7a7a4ab-1d3b-426e-92a3-bb5d6783f161.png)

In the beginning, our primary objective is to determine the distance between an arbitrary point("P") and a line segment defined by its two endpoints, referred to as (AB line). This function returns the shortest distance from the point to the line segment, along with the coordinates of the closest point on the segment. 
```python=
#Calculate the shortest distance between a point (P) and a line segment defined by two endpoints (A and B).
def distance_point_to_segment(P, A, B):
    AP = P - A
    BP = P - B
    AB = B - A
    if np.dot(AB, AP) > 0 and np.dot(-AB, BP) >= 0:  # P is in line AB
        return np.abs(np.cross(AP, AB)) / np.linalg.norm(AB), np.dot(AP, AB) / np.dot(AB, AB) * AB + A
    d_PA = np.linalg.norm(AP)  # P is outside triangle (not in line AB)
    d_PB = np.linalg.norm(BP)
    if d_PA < d_PB:
        return d_PA, A
    return d_PB, B
```
It's worth noting that the function employs a strategy to calculate the minimum distance based on the position of a point in relation to a line segment. If the point falls between the line segment's endpoints, it computes the shortest orthogonal distance. However, if the point lies outside the line segment, it determines the minimum distance by extending a line from the point to the closest endpoint of the segment.
![https://hackmd.io/_uploads/H1_sfee7p.png](https://user-images.githubusercontent.com/124876411/279774843-eaa1d08f-5aa5-43d6-a47d-dc1d70c0d7fa.png)

In the sequential step, to calculate the minimum distance between two cuboids defined by their vertices. It does so by computing the minimum distance between any pair of line segments from the two cuboids and returns the closest points along with the distance. 

```python=
#Calculate the minimum distance between two cuboids represented by their vertices.

def min_distance_cuboids(cub1,cub2):
    minD = 1e5
    for i in range(4):
        for j in range(4):
            # 計算點P到線段AB的最短距離和最短距離點Q的座標
            d, Q = distance_point_to_segment(cub1[i,:2], cub2[j,:2], cub2[j+1,:2])
            if d<minD:
                minD = d
                minP = cub1[i,:2]
                minQ = Q
    for i in range(4):
        for j in range(4):
            # 計算點P到線段AB的最短距離和最短距離點Q的座標
            d, Q = distance_point_to_segment(cub2[i,:2], cub1[j,:2], cub1[j+1,:2])
            if d<minD:
                minD = d
                minP = cub2[i,:2]
                minQ = Q
    return minP,minQ,minD
```
At this point, we have successfully ascertained the precise distance between the two objects. Remarkably, this approach allows us to obtain an exact distance even when the two objects are parallel.

![https://hackmd.io/_uploads/BJ8TBlxmT.png](https://user-images.githubusercontent.com/124876411/279774868-0be8f305-5442-4af9-8ead-bf34647d4665.png)

---
### car_tracking.ipynb
Precisely determining a location using GPS (Global Positioning System) data and IMU (Inertial Measurement Unit) data is a fundamental aspect of navigation, especially in applications like autonomous vehicles. GPS provides global positioning coordinates, but it can have limitations, such as accuracy and signal loss in urban canyons or tunnels. IMU data, on the other hand, can help mitigate these issues by providing continuous and reliable orientation and motion information.

```python=
#Calculate the great-circle distance between two points on the Earth's surface using their latitude and longitude coordinates.  

def comupte_great_circle_distance(lat1, lon1, lat2, lon2):
    delta_sigma = float(np.sin(lat1*np.pi/180)*np.sin(lat2*np.pi/180)+\
                        np.cos(lat1*np.pi/180)*np.cos(lat2*np.pi/180)*np.cos(lon1*np.pi/180-lon2*np.pi/180))
    return 6371000.0 * np.arccos(np.clip(delta_sigma,-1,1))
```
The plot displays the distances calculated from both GPS and IMU data, which the GPS data exhibits discontinuities, while the IMU data remains continuous.
![https://hackmd.io/_uploads/Sk_HXCJ7a.png](https://user-images.githubusercontent.com/124876411/279774678-3bfeedb0-5337-40f7-9988-80d583737c5d.png)

After merging both data, we can generate the following plot, which provides precise and continuous results.
![https://hackmd.io/_uploads/HkTlL0Jma.png](https://user-images.githubusercontent.com/124876411/279774714-86832d7d-4941-4285-a26c-db21fc555c90.png)


---
### kitti_plot2D.ipynb
The extraction of labels involves identifying objects of interest within an image, and subsequently, applying these labels to the corresponding objects. Once labeled, bounding boxes are drawn around these objects to visually highlight their presence and location within the image.

```
	frame	track_id	type		truncated	occluded	alpha		bbox_left	bbox_top	bbox_right	bbox_bottom	height	width	length		pos_x		pos_y		pos_z		rot_y
2	0	0		Car		0	0	-1.793451	296.744956	161.752147	455.226042	292.372804	2.000000	1.823255	4.433886	-4.552284	1.858523	13.410495	-2.115488
3	0	1		Cyclist		0	0	-1.936993	737.619499	161.531951	931.112229	374.000000	1.739063	0.824591	1.785241	1.640400	1.675660	5.776261	-1.675458
4	0	2		Pedestrian	0	0	-2.523309	1106.137292	166.576807	1204.470628	323.876144	1.714062	0.767881	0.972283	6.301919	1.652419	8.455685	-1.900245
7	1	0		Car		0	0	-1.796862	294.898777	156.024256	452.199718	284.621269	2.000000	1.823255	4.433886	-4.650955	1.766774	13.581085	-2.121565
8	1	1		Cyclist		0	0	-1.935205	745.017137	156.393157	938.839722	374.000000	1.739063	0.824591	1.785241	1.700640	1.640419	5.778596	-1.664456
...	...	...		...		...	...	...	...	...	...	...	...	...	...	...	...	...	...
1084	153	10		Car		0	2	-1.818856	680.294919	177.511028	842.313244	284.070033	1.524000	1.728591	3.894227	2.353367	1.622590	12.436503	-1.637280
1085	153	11		Car		0	2	1.864481	245.920800	194.456182	394.817829	286.444967	1.444000	1.595116	3.791789	-5.458963	1.908188	13.979427	1.497916
1086	153	12		Pedestrian	1	0	0.826456	1185.199080	151.165841	1241.000000	348.552707	1.688000	0.800000	0.884000	5.739732	1.500532	6.279632	1.543272
1087	153	13		Car		0	0	1.773993	344.361560	188.772369	430.531955	248.482384	1.422414	1.512803	3.707634	-6.033258	1.888008	19.788795	1.481180
1088	153	14		Car		0	2	-1.728662	652.362288	183.789605	737.478033	246.613864	1.365956	1.508586	3.485915	1.955738	1.651867	17.818612	-1.622048

711 rows × 17 columns
```
![https://hackmd.io/_uploads/ByvVtRkXT.png](https://user-images.githubusercontent.com/124876411/279774732-f8292f60-e011-4b9a-bd7d-b966b18dbb04.png)


---
### kitti_plot3D.ipynb
Utilizing point cloud data to [create a detailed 3D representation](https://github.com/enginBozkurt/Visualizing-lidar-data/blob/master/Kitti-Dataset.ipynb) of the scenario, allows us to build 3D detection bounding boxes based on tracking labels. To ensure accurate alignment and [calibration](https://github.com/charlesq34/frustum-pointnets/blob/master/kitti/kitti_util.py), we perform the necessary coordinate frame transformation from the camera frame to the global Velodyne frame. 
![plot1](https://user-images.githubusercontent.com/124876411/279775200-45de83db-e515-491d-9576-0744a522cb9b.png)
Next step involves the computation of the 3D coordinates for the eight corners of the bounding box within the camera coordinate system. This process requires the input of the box's height (h), width (w), length (l), its position ( x, y, z), and its yaw direction (yaw). It leverages rotation and translation matrices to perform the necessary transformations. The output is a 3x8 array, providing precise spatial coordinates for each of the box's corner points within the camera coordinate system, as specified.
```python=
# Calculate the 3D coordinates of the corners of a bounding box in the cam2 coordinate frame.
def compute_3d_box_cam2(h,w,l,x,y,z,yaw):
    """
    Return :3xn in cam2 coordinate
    """
    R = np.array([[np.cos(yaw),0,np.sin(yaw)],[0,1,0],[-np.sin(yaw),0,np.cos(yaw)]])
    x_corners = [l/2, l/2, -l/2, -l/2,  l/2,  l/2, -l/2, -l/2]
    y_corners = [0,   0,    0,    0,   -h,   -h,   -h,   -h  ]
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2  ]
    corners_3d_cam2 = np.dot(R,np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2 += np.vstack([x,y,z])
    return corners_3d_cam2
```
![https://hackmd.io/_uploads/S11rSkemT.png](https://user-images.githubusercontent.com/124876411/279775193-0779f19b-5a85-4c42-a1f1-e3f6fe702e8f.png)
However, it's important to note that the aforementioned plot is presented in relation to the camera frame. To align it with the Velodyne frame, we will perform calibration using the calibration file. This calibration process ensures that the spatial data is accurately transformed from the camera frame to the Velodyne frame, enabling a consistent and unified representation.
[calibration algorithm](https://github.com/charlesq34/frustum-pointnets/blob/master/kitti/kitti_util.py)
![https://hackmd.io/_uploads/H1yfHygQT.png](https://user-images.githubusercontent.com/124876411/279775188-af68e49d-5bf0-446b-a269-bc9486d8cbfe.png)

Eventually, we integrate the 3D bounding box and the point cloud environment. 

![https://hackmd.io/_uploads/r1e37yxXT.png](https://user-images.githubusercontent.com/124876411/279775179-9ce73efc-bd6a-43b8-8232-e496906c69e6.png)

---


## Source code
```
├── src
     ├── data_utils.py
     ├── kitti.py    (main)
     ├── misc.py
     ├── publish_utils.py
```

| File               | Description                                       |
| ------------------ | ------------------------------------------------- |
| `data_utils.py`    | Handles data processing and transformations.      |
| `kitti.py`         | Contains the main code for running the project.   |
| `misc.py`          | Stores helper function for dealing with problems. |
| `publish_utils.py` | Sets up ROS publisher and relative parameters     |

---

## Demo
In this section, we present a visual demonstration of our project's results, including videos and images that illustrate the outcomes and capabilities of our project. These visual assets offer a firsthand look at the project in action and provide insights into its performance and functionality. 

This image provides a visual representation of object detection results achieved through camera input. Each detected object is prominently highlighted with a bounding box, uniquely labeled in specific colors.
![https://hackmd.io/_uploads/rJfWSblmp.png](https://user-images.githubusercontent.com/124876411/279774999-ddcb80f0-4cb6-4fde-aa65-df20260a7f91.png)

The following synchronized image showcases several visual elements:

* Green Line: Represents the camera's field of view, indicating the sights that the camera can capture.
* Yellow Lines: Depicts the trajectory followed by objects that have recently passed through the scene.
* Pink Lines: Connect various object bounding boxes, estimating the distances between objects to avoid collisions. The numbers printed on the pink lines provide distance measurements.
* Purple Arrow: Always oriented from the vehicle's center to the ground, symbolizing IMU (Inertial Measurement Unit) data.

These visual cues offer valuable insights into the project's real-time monitoring, object tracking, and safety measures, making it a comprehensive visual reference for understanding the project's capabilities.
![https://hackmd.io/_uploads/rkivHWg7a.png](https://user-images.githubusercontent.com/124876411/279775003-5c3bcb27-357b-4990-87b0-e06aef355f4d.png)

**Video Link :**  https://youtu.be/pHGKihEtw4M

---

## Resources
1.	AI葵，自動駕駛教學。 [Youtube Link](https://https://youtu.be/TBdcwwr5Wyk?si=9j6L3XyesbA5WeWN)
2.	Charles R. Qi, calibration algorithm. [Github Link](https://github.com/charlesq34/frustum-pointnets/blob/master/kitti/kitti_util.py)
3.	Engin Bozkurt, visualizing algorithm.  [Github Link](https://github.com/enginBozkurt/Visualizing-lidar-data/blob/master/Kitti-Dataset.ipynb)

---

## License
Kitti-ROS-project work is © 2023 Evan YU. MIT License

See [LICENSE](https://github.com/EvanYu0000/Kitti-ROS-project/blob/main/LICENSE) for the full texts of the licenses.
