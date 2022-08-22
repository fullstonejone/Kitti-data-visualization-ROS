# ROS自动驾驶项目入门实战-KITTI数据集

![image-20220816132452557](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220816132452557.png)

![image-20220816132511980](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220816132511980.png)



## 1、Kitti资料介绍与可视化

> Kitti：http://www.cvlibs.net/datasets/kitti/eval_odometry.php
>
> BDD_Challenge：http://bdd-data.berkeley.edu

- **注：下载的数据集为2011_09_26_drive_0005_sync，其中共有image(原图像)、oxts(IMU以及GPS数据，bin文件)以及velodyne_points(激光雷达，点云，bin文件)**

安装kitti2bag：将kitti数据集转换为ROS可以读取的格式文件

> pip install kitti2bag

下载以后将calibration文件放在sync图像文件旁，cd至上一目录运行：

> kitti2bag -t 2011_09_26 -r 0005 raw_synced

这样就可以生成ros可以操作的bag了！

- 不自己写publischer等可视化数据集：

> roscore
>
> rosbag play kitti_2011_9_26_drive_0005_synced.bag -l
>
> rviz

注意：rviz上需要将当前的topic显示出来，比如camera和velo/pointcloud

## 2、发布图片

编写kitti.py，设置ros：

```python
#!/usr/bin/python
import cv2
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
DATA_PATH='/home/lee/lee/ROS/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/'

if __name__ == '__main__':
    frame = 0
    rospy.init_node('kitti_node',anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam',Image,queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        img = cv2.imread(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame))
        cam_pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))
        rospy.loginfo("camera image published")
        rate.sleep()
        frame += 1
        frame %=154  #连续播放图像数据
```

设置的步骤：

首先创建工作空间：

> cd caktin_ws/src
>
> catkin_create_pkg kitti_tutorial rospy
>
> cd ..
>
> catkin_make

进入到kitti.py上级文件夹，运行：

> chmod +x kitti.py
>
> rosrun kitti_tutorial kitti.py

在rviz上可以保存config！

## 3、发布点云资料

将kitti数据集上的360°点云资料可视化

建立Publisher——>读取资料——>发布

```python
#!/usr/bin/python
import cv2
import os
import rospy
from sensor_msgs.msg import Image，PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import Header
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
DATA_PATH='/home/lee/lee/ROS/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/'

if __name__ == '__main__':
    frame = 0
    rospy.init_node('kitti_node',anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam',Image,queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud',PointCloud2,queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        img = cv2.imread(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame))#读取图像
        cam_pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))
        ponit_cloud = np.fromfile(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame),dtype = np.float32),reshape(-1,4)#转换为n*4矩阵，n自动计算
        header = Header()
        header.stamp = rospy.Time.now() ##当前时间点
        header.frame_id = 'map'  ##坐标系名称
        pcl_pub.publish(pcl2.create_cloud_xyz32(header,point_cloud[:,:3])) ##读取前三个坐标点(前三行)
        rospy.loginfo(" published")
        rate.sleep()
        frame += 1
        frame %=154  #连续播放图像数据
```

点云资料被保存在velodyne_points/data中，数据格式为二进制文件，每行共四个数值，分别代表3D点坐标和反射率

## 4、画出车子以及相机视野

首先就是rosun kitty的分文件编写，编写data_utils.py和publish.utils.py，在kitty.py中引用：

> from data_utils import *
>
> from publish_utils import *

- rviz/Display/Types/Marker中的Line Strip(LINE_STRIP=4)中绘制车辆前置摄像头的视野方向，在publish_utils.py中进行添加：

```python
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
def publish_ego_car(ego_car_pub):
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
    
    marker.id = 0
    marker,action = Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type = Marker.LINE_STRIP
    
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0 ##透明度
    marker.scale.x = 0.2 ##线的粗细
    
    marker.points = [] ##这里的设置查看前面的velodyne_to_camera坐标设定
    marker.points.append(Point(10,-10,0))
    marker.points.append(Point(0,0,0))   ##起点 
    marker.points.append(Point(10,-10,0)) 
    
    ego_car_publish(marker)     
```

在kitti中书写:

```python
ego_pub = rospy.Publisher('kitti_ego_car',Marker,queue_size=10)
publish_ego_car(ego_pub)
```

- 加入车子模型

首先下载car dae( 3D models)

将其解压缩放置在kitti_tutorial/下，使用的包是Marker下的Mesh Resource

在publish_utils.py中写函数

```python
def publish_car_model(model_pub):
    mesh_marker=Marker()
    mesh_marker.header.frame_id =FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()

    mesh_marker.id = -1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = "package://kitti_tutorial/Audi_A8/Car-Model/Car.dae"

    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73

    q = tf.transformations.quaternion_from_euler(np.pi,np.pi,-np.pi/2.0)
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]

    mesh_marker.scale.x = 1
    mesh_marker.scale.y = 1
    mesh_marker.scale.z = 1

    mesh_marker.color.a = 1.0
    mesh_marker.color.r = 1.0
    mesh_marker.color.g = 1.0
    mesh_marker.color.b = 1.0

    model_pub.publish(mesh_marker)
```

然后同理在kitti.py中进行publish：

```python
model_pub = rospy.Publisher('kitti_car_model',Marker,queue_size=10)
publish_car_model(model_pub)
```

## 5、发布IMU数据和处理GPS数据

回顾：上一节讲的两个Marker，如果要保证时间一致的话，可以使用MarkerArray

在建立两个Marker后，将信息加至MarkerArray中：

```python
from visualization_msgs.msg import Marker，MarkerArray
def pulish_ego_car(ego_car_pub):
    marker_array = MarkerArray()
    marker = Marker()
    >>>>>>  ##这里放的是相机视角信息
    >>>>>>
    marker_array.markers.append(marker)
    >>>>>>  ##这里放的是汽车模型信息
    >>>>>>
    mesh_marker = Marker()
    marker_array.markers.append(mesh_marker)
    ego_car_pub.publish(marker_array)
```

这种方式精简了代码，只需要在kitti.py中运行：

```python
ego_pub = rospy.Publisher('kitti_ego_car',MarkerArray,queue_size=10)
publish_ego_car(ego_pub)
```

- **如何发布IMU资料（可视化 ）：**

首先处理IMU资料：

```python
import pandas as pd

IMU_COLUMN_NAMES = ["lat", "lon", "alt", "roll", "pitch", "yaw", "vn", "ve", "vf", "vl", "vu", "ax", "ay", "az", "af", "al", "au", "wx", "wy", "wz", "wf", "wl", "wu", "posacc", "velacc", "navstat", "numsats", "posmode", "velmode", "orimode"]
def read_camera(path):
    return cv2.imread(path)
def read_point_cloud(path):
    return np.fromfile(path,dtype=np.float32).reshape(-1,4)   
def read_imu(path):
    df = pd.read_csv(path,header=None,sep=' ')  ##读取IMU数据
    df.columns = IMU_COLUMN_NAMES
    return df
```

建立Publisher:

```python
def publish_imu(imu_pub,imu_data):
    imu =Imu()
    imu.header.frame_id = FRAME_ID
    imu.header.stamp = rospy.Time.now()
    #设置旋转量
    q=tf.transformations.quaternion_from_euler(float(imu_data.roll),float(imu_data.pitch),float(imu_data.yaw));
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]
    
    #设置线性加速度
    imu.linear_acceleration.x = imu_data.af  
    imu.linear_acceleration.y = imu_data.al
    imu.linear_acceleration.z = imu_data.au
    
    #设置角加速度
    imu.angular_velocity.x = imu_data.wf
    imu.angular_velocity.y = imu_data.wl
    imu.angular_velocity.z = imu_data.wu
    
    imu_pub.publish(imu)
    
    
```

最后在kitti.py中添加publisher：

```python
##创建Imu发布
imu_pub = rospy.Publisher('kitti_imu',Imu,queue_size=10)
imu_data = read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))
publish_imu(imu_pub,imu_data)
```

注意：箭头表示就是方向、加速度、角速度

![image-20220816170943187](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220816170943187.png)

- **6、处理GPS数据**

GPS的数据资料包含在IMU数据中：

```python
def publish_gps(gps_data, gps_pub):
    gps = NavSatFix()
    gps.header.frame_id = "map"
    gps.header.stamp = rospy.Time.now()
    # 赋值经纬度和海拔
    gps.latitude = gps_data.lat
    gps.longitude = gps_data.lon
    gps.altitude = gps_data.alt

    gps_pub.publish(gps)
```

查看GPS数据：

> rostopic list
>
> rostopic info /kitti_gps
>
> rostopic echo /kitti_gps

## 6、下载并读取tracking数据

首先在kitti上的网站上下载：

> https://s3.eu-central-1.amazonaws/avg-kitti/data_tracking_label_2.zip

 将下载的数据放在data/tracking下，在jupyter notebook上运行：

```python
#数据单位
import pandas as pd
import numpy as np

#数据单位
COLUMN_NAMES=['frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top','bbox_right', 
              'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']
df = pd.read_csv('/home/lee/lee/ROS/data/tracking/training/label_02/0000.txt',header=None,sep=' ')
df.columns = COLUMN_NAMES
df.head()#head()方法，默认读取前五行，如果想显示更多，那么在括号内赋值
df.type.isin(['Truck','Van','Tram'])
df.loc[df.type.isin(['Truck','Van','Tram']),'type']='Car'
df = df[df.type.isin(['Car','Pedestrian','Cyclist'])]
df
df.loc[2,['bbox_left' ,'bbox_top','bbox_right','bbox_bottom']]
box=np.array(df.loc[2,['bbox_left' ,'bbox_top','bbox_right','bbox_bottom']])
box   ##显示box的框信息
```

```python
import cv2
frame = 120
image = cv2.imread('/home/lee/lee/ROS/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/image_00/data/%010d.png'%frame)
boxes = np.array(df[df['frame']==frame][['bbox_left','bbox_top','bbox_right','bbox_bottom']])
types = np.array(df[df.frame==frame]['type'])
DETECTION_COLOR_DICT = {'Car':(255,255,0),'Pedestrian':(0,226,255),'Cyclist':(141,40,255)}##设置不同标签对应的颜色框

for type,box in zip(types,boxes):
    top_left = int(box[0]),int(box[1])
    bottom_right = int(box[2]),int(box[3])
    cv2.rectangle(image,top_left,bottom_right,DETECTION_COLOR_DICT[type],2)  ##蓝色框 宽度：2
    
cv2.imshow("image",image)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

## 7、在RVIZ上显示2D框

**明确：在视频上画出2D检测框，无非就是对于检测或者人工标注得到的检测结果框逐帧进行显示，使用cv2，首先定义数据读取：**

```python
#####data_util.py中定义
TRACKING_COLUMN_NAMES=['frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top','bbox_right', 
              'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']
def read_tracking(path):
    df = pd.read_csv(path,header=None,sep=' ')
    df.columns = TRACKING_COLUMN_NAMES
    df.loc[df.type.isin(['Truck','Van','Tram']),'type']='Car'
    df = df[df.type.isin(['Car','Pedestrian','Cyclist'])]
    return df
```

然后更改publish_utils.py中关于publish_camera:

```python
DETECTION_COLOR_DICT= {'Car':(255,255,0),'Pedestrian':(0,226,255),'Cyclist':(141,40,255)}

def publish_camera(cam_pub,bridge,image,boxes,types):
    for typ,box in zip(types,boxes):
        top_left =int(box[0]),int(box[1])
        bottom_right = int(box[2]),int(box[3])
        cv2.rectangle(image,top_left,bottom_right,DETECTION_COLOR_DICT[typ],2) ##在逐帧图像上画检测框
	    cam_pub.publish(bridge.cv2_to_imgmsg(image,"bgr8"))
```

最后在kittie.py中进行publish：

```python
df_tracking = read_tracking('/home/lee/lee/ROS/data/tracking/training/label_02/0000.txt')
        boxes = np.array(df_tracking[df_tracking['frame']==frame[['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        types = np.array(df_tracking[df_tracking.frame==frame]['type'])
		image = read_camera(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame))
		publish_camera(cam_pub,bridge,image,boxes,types)

```

## 8、画出3D检测框

首先需要绘制出点云信息：

```python
import numpy as np
import os
BASE_PATH = "/home/lee/lee/ROS/kitti/2011_09_26/2011_09_26_drive_0005_sync/"  
def read_point_cloud():
    point_cloud = np.fromfile(os.path.join(BASE_PATH, "velodyne_points/data/%010d.bin"%0), dtype=np.float32).reshape(-1,4)
    return point_cloud
```

绘制点云代码：

函数的主要用法参数介绍如下所示：
ax：matplotlib画板
points：read_point_cloud()读取的点云数据
title：画板标题
axes：[0,1,2]分别代表xyz三个轴
point_size：点云大小
xlim3d：X轴在3D显示下限制角度的范围
ylim3d：Y轴在3D显示下限制角度的范围
zlim3d：Z轴在3D显示下限制角度的范围

```python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def draw_point_cloud(ax, points, title, axes=[0, 1, 2], point_size=0.2, xlim3d=None, ylim3d=None, zlim3d=None):
    """
    Convenient method for drawing various point cloud projections as a part of frame statistics.
    """
    # 设置xyz三个轴的点云范围
    axes_limits = [
        [-20, 80], # X axis range
        [-20, 20], # Y axis range
        [-3, 5]    # Z axis range
    ]
    axes_str = ['X', 'Y', 'Z']
    # 禁止显示背后的网格
    ax.grid(False)
    # 创建散点图[1]:xyz数据集，[2]:点云的大小，[3]:点云的反射率数据,[4]:为灰度显示
    ax.scatter(*np.transpose(points[:, axes]), s=point_size, c=points[:, 3], cmap='gray')
    # 设置画板的标题
    ax.set_title(title)
    # 设置x轴标题
    ax.set_xlabel('{} axis'.format(axes_str[axes[0]]))
    # 设置y轴标题
    ax.set_ylabel('{} axis'.format(axes_str[axes[1]]))
    if len(axes) > 2:
        # 设置限制角度
        ax.set_xlim3d(*axes_limits[axes[0]])
        ax.set_ylim3d(*axes_limits[axes[1]])
        ax.set_zlim3d(*axes_limits[axes[2]])
        # 将背景颜色设置为RGBA格式，目前的参数以透明显示
        ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        # 设置z轴标题
        ax.set_zlabel('{} axis'.format(axes_str[axes[2]]))
    else:
        # 2D限制角度，只有xy轴
        ax.set_xlim(*axes_limits[axes[0]])
        ax.set_ylim(*axes_limits[axes[1]])
    # User specified limits
    if xlim3d!=None:
        ax.set_xlim3d(xlim3d)
    if ylim3d!=None:
        ax.set_ylim3d(ylim3d)
    if zlim3d!=None:
        ax.set_zlim3d(zlim3d)

```

显示点云可视化：

```python
# 获取数据集中的点云数据
point_cloud = read_point_cloud()
# 绘制3D点云数据，创建一个大小为20*10的图形画板
fig = plt.figure(figsize=(20, 10))
# 在画板中添加1*1的网格的第一个子图，为3D图像
ax = fig.add_subplot(111, projection='3d')
# 改变绘制图像的视角，即相机的位置，elev为Z轴角度，azim为(x,y)角度
ax.view_init(60,130)
# 在画板中画出点云显示数据，point_cloud[::x]x值越大，显示的点越稀疏
draw_point_cloud(ax, point_cloud[::5], "velo_points")
```

同理获得2D点云只需要将axes设置为x、y轴：

```python
# 同样是创建画布，只不过这个函数一步到位了
fig, ax = plt.subplots(figsize=(20,10))
# 绘制2D【x,y】轴方向的点云
draw_point_cloud(ax, point_cloud[::5], "velo_points", axes=[0, 1])
```

3D框需要的是7个信息分别如下：

`dimensions_height，dimensions_width，dimensions_length，location_x，location_y，location_z，rotation_y`

从这些信息中生成3D检测框的8个顶点坐标

![image-20220817163357889](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220817163357889.png)

最理想的模型初步计算顶点坐标，也就是yaw=0，如下所示。

![image-20220817163424616](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220817163424616.png)

如下公式所示：
$$
\left(\mathrm{x} \pm \frac{\mathrm{w}}{2}, \mathrm{y}, \mathrm{z} \pm \frac{1}{2}\right)
$$
可以将其替换为相对于 Location 坐标系的差值, 如下所示：
$$
\text { xcorners }=\left(\frac{1}{2}, \frac{1}{2},-\frac{1}{2},-\frac{1}{2}, \frac{1}{2}, \frac{1}{2},-\frac{1}{2},-\frac{1}{2}\right)
$$
$$
\text { ycorners }=(0,0,0,0,-\mathrm{h},-\mathrm{h},-\mathrm{h},-\mathrm{h})
$$
$$
\text { zcorners }=\left(\frac{\mathrm{w}}{2},-\frac{\mathrm{w}}{2},-\frac{\mathrm{w}}{2}, \frac{\mathrm{w}}{2}, \frac{\mathrm{w}}{2},-\frac{\mathrm{w}}{2},-\frac{\mathrm{w}}{2}, \frac{\mathrm{w}}{2}\right)
$$

但是这只是在`yaw=0`的情况下得到的坐标，一般正常情况下物体不会正常摆放，都会存在一个旋转角，在kitti数据集中也提供了`rotation_y`作为物体绕y轴的旋转角，类似下图所示：

![image-20220817163719692](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220817163719692.png)

旋转矩阵将yaw=0时的坐标通过点乘转换为yaw!=0时的坐标，由于是绕着Y轴转动，所以旋转矩阵为：
$$
\left[\begin{array}{ccc}
\cos (\text { yaw }) & 0 & \sin (\text { yaw }) \\
0 & 1 & 0 \\
\sin (\text { yaw }) & 0 & \cos (\text { yaw })
\end{array}\right]
$$
代码如下所示：

```python
def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    # 计算旋转矩阵
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    # 8个顶点的xyz
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h] 
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    # 旋转矩阵点乘(3，8)顶点矩阵
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    # 加上location中心点，得出8个顶点旋转后的坐标
    corners_3d_cam2 += np.vstack([x,y,z])
    return corners_3d_cam2
# 计算cam2眼中的3DBox
corners_3d_cam2 = compute_3d_box_cam2(*df.loc[2,['dimensions_height','dimensions_width','dimensions_length','location_x','location_y','location_z','rotation_y']])
```

绘制物体最初的样子:

```python
def draw_box(ax, vertices, axes=[0, 1, 2], color='black'):
    """
    Draws a bounding 3D box in a pyplot axis.
    
    Parameters
    ----------
    pyplot_axis : Pyplot axis to draw in.
    vertices    : Array 8 box vertices containing x, y, z coordinates.
    axes        : Axes to use. Defaults to `[0, 1, 2]`, e.g. x, y and z axes.
    color       : Drawing color. Defaults to `black`.
    """
    vertices = vertices[axes, :]
    connections = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Lower plane parallel to Z=0 plane
        [4, 5], [5, 6], [6, 7], [7, 4],  # Upper plane parallel to Z=0 plane
        [0, 4], [1, 5], [2, 6], [3, 7]  # Connections between upper and lower planes
    ]
    for connection in connections:
        ax.plot(*vertices[:, connection], c=color, lw=0.5)

fig = plt.figure(figsize=(20,10))
ax = fig.add_subplot(111, projection='3d')    
ax.view_init(40,150)
draw_box(ax,(corners_3d_cam2))
```

由于在实际运行中采用的是velodyne坐标，需要进行调整：

```python
from kitti_util import *
# 读取calibration转换数据
calib = Calibration("/home/lee/lee/ROS/data/kitti/2011_09_26/", from_video=True)
# cam2转velo坐标系
corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T).T  ##注意这里和rviz显示3D不同，是显示代码的问题，这里是3*8，后者为8*3
```

绘制转换后的物体检测框：

```python
fig = plt.figure(figsize=(20,10))
ax = fig.add_subplot(111, projection = '3d')
ax.view_init(40,150)
draw_box(ax,corners_3d_velo)
```

点云结合在一起绘制检测框:

```python
# 和点云做结合
fig = plt.figure(figsize=(20,10))
ax = fig.add_subplot(111, projection = '3d')
ax.view_init(70,230)
draw_point_cloud(ax,point_cloud[::5],"velo pointcloud")
draw_box(ax,corners_3d_velo,color='red')
```

换成XOY视角：

```python
fig, ax = plt.subplots(figsize=(20,10))
draw_point_cloud(ax,point_cloud[::5],"velo pointcloud",axes=[0,1])
draw_box(ax,corners_3d_velo,color='red',axes=[0,1])
```

## 9、在RVIZ绘制点云中所有的框

使用的是Marker中的LINE_LIST

publish_utilis.py中添加publish_3dbox函数：

```python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import tf
import cv2

FRAME_ID = "map"
DETECTION_COLOR_DICT = {'Car':(255, 255, 0), 'Pedestrian':(0, 226, 255), 'Cyclist':(141, 40, 255)}
#为三种车型,分别添加颜色值对应的框
LIFETIME = 0.1 
#生存周期

LINES = [[0,1], [1,2], [2,3], [3, 0]]   #底面
LINES += [[4,5], [5,6], [6,7], [7,4]]   #顶面
LINES += [[4,0], [5,1], [6,2], [7,3]]   #中间的四条线
LINES += [[4,1], [5,0]]                 #前边用"X"标志

def publish_3dbox(box3d_pub, corners_3d_velos, types):
    marker_array = MarkerArray()
    #对每一组数据/交通工具/矩形框进行迭代
    for i, corners_3d_velo in enumerate(corners_3d_velos):
        #header部分
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        # marker的id 
        marker.id = i
        marker.action = Marker.ADD      # 加入一个marker
        marker.lifetime = rospy.Duration(0)  # 生存时间,()中无参数永远出现
        marker.type = Marker.LINE_STRIP     #marker 的type,有很多种,这里选择线条

        b, g, r = DETECTION_COLOR_DICT[types[i]]

        marker.color.r = r / 255.0
        marker.color.g = g / 255.0
        marker.color.b = b / 255.0           #这条线的颜色

        marker.color.a = 1.0            #透明度 1不透明

        marker.scale.x = 0.1            #大小,粗细

        #设定marker中的资料
        marker.points = []
        #对每两个点之间进行迭代,存在则连
        for l in LINES :
            p1 = corners_3d_velo[l[0]] ###注意这里8*3的array维度
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            p2 = corners_3d_velo[l[1]]
            marker.points.append(Point(p2[0], p2[1], p2[2]))

        marker_array.markers.append(marker)
    box3d_pub.publish(marker_array)
```

在kitti.py中去进行publish：

```python
##首先定义计算3dbox的函数：
def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    '''
    return : 3xn in cam2 coordinate
    '''
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
    
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d_cam2 += np.vstack([x, y, z]) 
    
    return corners_3d_cam2
```

对每一帧对应的点云都进行运算，得到相应的3D框：

```python
df_tracking = read_tracking('/home/lee/lee/ROS/data/tracking/training/label_02/0000.txt')
calib = Calibration('/home/lee/lee/ROS/data/kitti/2011_09_26/', from_video=True)
while not rospy.is_shutdown():
	df_tracking_frame = df_tracking[df_tracking.frame == frame]

	boxes_2d = np.array(df_tracking_frame[['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        types = np.array(df_tracking_frame['type'])
	
	boxes_3d = np.array(df_tracking_frame[['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']])

        corners_3d_velos = []
	for box_3d in boxes_3d:
           corners_3d_cam2 = compute_3d_box_cam2(*box_3d)
           corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T)
           corners_3d_velos += [corners_3d_velo]
           publish_3dbox(box3d_pub,corners_3d_velos,types) #publish 3d框对应的Publisher！
```

## 10、显示track_id

使用的Type是Marker下的TEXT_VIEW_FACING

首先需要读取track_id的信息：

```python
#####kitty.py中添加track_ids
track_ids = np.array(df_tracking_frame['track_id'])
```

直接在publish_3dbox下对marker_array进行添加

```python
def publish_3dbox(box3d_pub, corners_3d_velos, types, track_ids):
    marker_array = MarkerArray()
    #对每一组数据/交通工具/矩形框进行迭代
    for i, corners_3d_velo in enumerate(corners_3d_velos):
        #header部分
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        # marker的id 
        marker.id = i
        marker.action = Marker.ADD      # 加入一个marker
        marker.lifetime = rospy.Duration(0)  # 生存时间,()中无参数永远出现
        marker.type = Marker.LINE_STRIP     #marker 的type,有很多种,这里选择线条

        b, g, r = DETECTION_COLOR_DICT[types[i]]
        marker.color.r = r / 255.0
        marker.color.g = g / 255.0
        marker.color.b = b / 255.0           #这条线的颜色
        marker.color.a = 1.0            #透明度 1不透明
        marker.scale.x = 0.1            #大小,粗细

        #设定marker中的资料
        marker.points = []
        #对每两个点之间进行迭代,存在则连
        for l in LINES :
            p1 = corners_3d_velo[l[0]] ###注意这里8*3的array维度
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            p2 = corners_3d_velo[l[1]]
            marker.points.append(Point(p2[0], p2[1], p2[2]))
        marker_array.markers.append(marker)
        
        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = rospy.Time.now()
        
        text_marker.id = i+1000
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(LIFETIME)
        text_marker.type = Marker.TEXT_VIEW_FACING  ##3d上查看，永远朝向自己
         
        #p4 = corners_3d_velo[4]   #取得左前上方的点作为基准点
        #text_marker.pose.posotion.x = p4[0]
        #text_marker.pose.posotion.y = p4[1]
        #text_marker.pose.posotion.z = p4[2]+0.5
        
        p4 = np.mean(corners_3d_velo[4],axis=0)   #8*3->1*3 所有的坐标平均
        text_marker.pose.posotion.x = p4[0]
        text_marker.pose.posotion.y = p4[1]
        text_marker.pose.posotion.z = p4[2]+1
        
        text_marker.text = str(track_ids[i])
        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1
        
        b,g,r = DETECTION_COLOR_DICT[types[i]]
        text_marker.color.r = r/255.0
        text_marker.color.g = b/255.0
        text_marker.color.b = r/255.0
        text_marker.color.a = 1.0
        
        marker_array.markers.append(text_marker)       
    box3d_pub.publish(marker_array)
```

## 11、如何计算移动目标的历史坐标

- 如果仅存在移动而没有转动的情况，则历史帧的坐标，相比较当前的坐标轴定义为下：

![image-20220819110256159](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220819110256159.png)

**举例：若当前帧相较于上一帧在X方向上移动d，则上一帧的坐标在当前帧的坐标系下为：(-d,0)**

- 若两帧间的坐标系发生旋转

**注：逆时针角度为正、顺时针角度为负**

仅当第一帧时，以下情况成立：**第一帧相比较于第二帧的坐标为(-d,0)**

![image-20220819111532448](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220819111532448.png)

 一般情况下，同时存在了旋转以及平移，仍以上图为例：

2D旋转矩阵定义为下：

![image-20220819114636819](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220819114636819.png)

第一帧某点的位置相比较第二帧也就是当前帧的坐标，需要结合两帧之间的旋转矩阵R，R代表的含义就是将第一帧的坐标系转化为第二帧的坐标系所做的变换，同理，从点的角度出发，我们可以认为$R(-\theta)$指坐标轴不动，点进行的变换。此时坐标点$(x_0,y_0)$在第二帧的参考坐标为：这里的T指的是带有平移的坐标变换矩阵。
$$
\begin{pmatrix}
 x_1  \\
 y_1  \\
 1
\end{pmatrix}=T(-\theta)\begin{pmatrix}
 x_0  \\
 y_0  \\
 1
\end{pmatrix}=\begin{pmatrix}
 x_0cos\theta+y_0sin\theta-d \\
 -x_0sin\theta+y_0cos\theta
\end{pmatrix}
$$
当车不停的移动，轨迹的绘制所需的历史位置的参考坐标系就是当前的坐标系！

##  12、利用IMU数据计算移动距离和旋转角度

- 获取旋转的角度：

![img](https://img-blog.csdnimg.cn/e16d98e0a8c24f948a18cafa10e3cff0.JPG?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAa3lkMTIzNDU=,size_20,color_FFFFFF,t_70,g_se,x_16)

在IMU/GPS信息保存在的oxts文件中，直接存储了yaw、roll以及pitch角度信息，由于我们绘制的是2D上的信息，则可以仅采集Yaw的角度信息。

因此获取两帧之间的角度变化，只需要提取其OXTS文件中的yaw角变化即可！

- 获取移动距离

汽车在地球上运行，距离的计算我们可以参考Great-circle distance 

设计一个大圆同时经过两点，此时最短距离就是圆上的对应圆弧

公式介绍：

$\phi_1$、$\lambda_1$、$\phi_2$、$\lambda_2$分别为A、B两点的经度和纬度，此时$\Delta\phi$和$\Delta\lambda$即为他们之间的绝对差值，$\Delta\sigma$为他们之间的中间角，并由下式给出：
$$
\Delta \sigma=\arccos \left(\sin \phi_{1} \cdot \sin \phi_{2}+\cos \phi_{1} \cdot \cos \phi_{2} \cdot \cos (\Delta \lambda)\right)
$$
![image-20220819135643944](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220819135643944.png)

半径为r的球上弧长d根据下式计算得到：
$$
d = r \Delta\sigma
$$

```python
import pandas as pd 
import numpy as np 
IMU_COLUMN_NAMES = ['lat','lon','alt','roll','pith','yaw','vn','ve','vf','vl','vu','ax','ay',
                    'az','af','al','au','wx','wy','wz','wf','wl','wu','posacc','velacc','navstat',
                   'numsats','posmode','velmode','orimode']
def read_imu(path):
    df = pd.read_csv(path,header=None,sep=' ')
    df.columns = IMU_COLUMN_NAMES
    return df

def compute_great_cricle_distance(lat1,lon1,lat2,lon2):
    """"
    Input:latitudes and longitudes in degree
    Output:distance in meter
    """
    delta_sigma = float(np.sin(lat1*np.pi/180)*np.sin(lat2*np.pi/180))+ np.cos(lat1*np.pi/180)*np.cos(lst2*np.pi/180)*np.cos(lon1*np.pi/180-lon*np.pi/180)           
    return 6371000.0*np.arccos(np.clip(delta_sigma,-1,1))  ##地球半径, 设定范围

#######注：还可以使用vf和vl用于计算移动距离，每帧间隔0.1s，使用0.1*vf 和 0.1*vl 便可以得到前向和左向的位移
```

计算距离并可视化：

```python
prev_imu_data = None
gps_distances = []   ##使用GPS计算移动距离
imu_distances = []   ##使用IMU计算移动距离
for frame in range(150):
    imu_data = read_imu('/home/lee/lee/ROS/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/oxts/data/%010d.txt'%frame)
    if prev_imu_data is not None:
        gps_distances += [compute_great_circle_distance(imu_data.lat,imu_data.lon,prev_imu_data.lat,prev_imu_data.lon)]  
        imu_distances += [0.1 * np.linalg.norm(imu_data[['vf','vl']])]
    prev_imu_data = imu_data
    
import matplotlib.pyplot as plt
plt.figure(figsize=(20,10))
plt.plot(gps_distances,label='gps_distance')
plt.plot(imu_distances,label='imu_distance')
plt.legend()
plt.show()
```

![image-20220819145305232](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220819145305232.png)

蓝色的是GPS计算距离，黄色则是IMU计算得到，由于IMU频率比较高的原因，虽然短时间内IMU测距较准，但IMU漂移的存在，GPS则在长期测距上更好一点。

## 13、绘制轨迹

使用IMU计算的距离绘制出两轴图像：

```python
prev_imu_data = None
locations = []  ###保存带有旋转平移的位置变化
for frame in range(150):
    imu_data = read_imu('/home/lee/lee/ROS/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/oxts/data/%010d.txt'%frame)
    if prev_imu_data is not None:
        displacement = 0.1 * np.linalg.norm(imu_data[['vf','vl']])
        yaw_change = float(imu_data.yaw-prev_imu_data.yaw)
        for i in range(len(locations)):
            x0,y0 = locations[i]
            x1 = x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
            y1 = -x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
            locations[i] = np.array([x1,y1])
               
    locations += [np.array([0,0])]   
    prev_imu_data = imu_data
#####绘制  locations的长度是150
plt.figure(figsize=(20,10))
plt.plot(np.array(locations)[:,0],np.array(locations)[:,1])
```

![image-20220819152150348](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220819152150348.png)

- 在点云上绘制轨迹：

首先我们需要定义一个类：

```python
class Object():
    def __init__(self):  ##初始化
        self.locations = []
        ##如果仅保存最近20帧的数据，则可以使用deque容器进行限制容量为20
        #self.locations = deque(maxlen=20)
    def update(self,displacement,yaw):   ##更新策略  用11讲的公式进行迭代更新
        for i in range(len(self.locations)):
            x0,y0 = self.locations[i]
            x1 = x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
            y1 = -x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
            self.locations[i] = np.array([x1,y1])
        self.locations += [np.array([0,0])]   ###初始第一帧是(0,0)
        #self.locations.appendleft(np.array([0,0]))  这里也要修改

    def reset(self):
        self.locations = []
        #self.locations = deque(maxlen = 20)
```

在publish_utilis.py中定义loc_pub函数：

```python
def publish_loc(loc_pub,locations):
    marker_array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()
        
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()   ##点连接的生命周期一直持续
    marker.type = Marker.LINE_STRIP
   
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.2

    marker.points = []
    for p in locations:
        marker.points.append(Point(p[0],p[1],0)) ##这里就是把locations中的所有的点全放在points中，z轴上设为零
        marker_array.markers.append(marker) 
    loc_pub.publish(marker_array)
```

在kitti.py中定义和设置：

```python
ego_car = Object()
loc_pub = rospy.Publisher('kitti_loc',MarkerArray,queue_size=10)
prev_imu_data = None
while not rospy.is_shutdown():
    if prev_imu_data is not None:
           displacement = 0.1 * np.linalg.norm(imu_data[['vf','vl']])
           yaw_change = float(imu_data.yaw-prev_imu_data.yaw)
	   ego_car.update(displacement,yaw_change)
	prev_imu_data = imu_data
    ####publish
    publich_loc(loc_pub,ego_car.locations)
    
```

## 14、绘制所有检测的物体的运动轨迹

**建立字典结构**

`tracker={}`：记录着所有出现的objects

`centers={}`：记录每一帧中出现的objects



首先需要更改类中update函数：

```python
class Object():
    def __init__(self,center):  ##初始化
        self.locations = deque(maxlen=20)
        self.locations.appendleft(center)  
        
    def update(self,center,displacement,yaw):   ##更新策略  用11讲的公式进行迭代更新
        for i in range(len(self.locations)):
            x0,y0 = self.locations[i]
            x1 = x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
            y1 = -x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
            self.locations[i] = np.array([x1,y1])        
        if center is not None:
            self.locations.appendleft(center)  

    def reset(self):
        self.locations = deque(maxlen = 20)
```

kitti.py中的代码如下所示：

```python
''''
绘制所有检测出的物体的轨迹
''''
###首先需要存储当前帧所有的检测出的物体
tracker = {}  ##不止一台车子，因此用单个的Object()不合适  track_id :Object
loc_pub = rospy.Publisher('kitti_loc',MarkerArray,queue_size=10)
prev_imu_data = None
while not rospy.is_shutdown():
    track_ids = np.array(df_tracking_frame['track_id'])
	corners_3d_velos = []
    centers = {}  ##track_id:center
	for track_id, box_3d in zip(track_ids,boxes_3d):
           corners_3d_cam2 = compute_3d_box_cam2(*box_3d)
           corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T)
           corners_3d_velos += [corners_3d_velo]
           centers[track_id] = np.mean(corner_3d_velo,axis=0)[:2]##取鸟瞰图，不需要z轴
    centers[-1] = np.array([0,0])  ##车子本身信息被存至-1索引
    
	if prev_imu_data is None:
        for track_id in centers:
            tracker[track_id] = Object(centers[track_id])
    else:
        displacement = 0.1 * np.linalg.norm(imu_data[['vf','vl']])
        yaw_change = float(imu_data.yaw-prev_imu_data.yaw)
        ####若当前帧且第一帧没有出现该object，则对其位置进行更新，若当前帧存在，但历史track中没有，则加入全局字典中
        for track_id in centers:
            if track_id in trackers:
                tracker[track_id].update(centers[track_id],displacement,yaw_change)
            else:
                tracker[track_id] = Object(centers[track_id])
        for track_id in tracker:
            if track_id not in centers:
                tracker[track_id].update(None,displacement,yaw_change)
	prev_imu_data = imu_data
    ####publish
    publich_loc(loc_pub,tracker,centers)
    if frame == 154:
        frame = 0
        for track_id in tracker:
            tracker[track_id].reset()
    

```

同时还需要修改publish_utils.py

```python
def publish_loc(loc_pub,tracker,center):
    marker_array = MarkerArray()
	for track_id in centers:
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration()   ##点连接的生命周期一直持续
        marker.type = Marker.LINE_STRIP

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 0.2

        marker.points = []
    for p in tracker[track_id].locations:
        marker.points.append(Point(p[0],p[1],0)) ##这里就是把locations中的所有的点全放在points中，z轴上设为零
        marker_array.markers.append(marker) 
    loc_pub.publish(marker_array)
```

- **多目标跟踪(MOT)**

不同帧检测出的物体，划定相同的追踪id

## 15、计算车辆和其它Objects间的距离

两个取到最小距离的两个点，其中一个必然是顶点

最小距离是通过：取其中一个检测2D框的四个点，同另一个检测框的四条边计算垂直距离，同理反向选择点和边，共计有32个距离结果，此时可以去其中的最小值 

![image-20220820160337386](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220820160337386.png)

这个求最值的问题也就转换为已知一个点的坐标，求其到线段的距离，有如下的情形需要考虑：

- 点P到直线AB的垂心在AB上：

![image-20220820160810310](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220820160810310.png)

- 若垂心出于AB的延长线上，此时的Q就不用了

![image-20220820160947400](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220820160947400.png)



注：如何判断上面情况的条件？

根据角度，且$cos\theta \gt0 $

![image-20220820161316142](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220820161316142.png)

- 如何求长度？

第一种情况，使用三角函数：

![image-20220820162000127](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220820162000127.png)
$$
Q = 
\frac{\overrightarrow{A P} \cdot \overrightarrow{A B}}{\|\overrightarrow{A B}\|^{2}} \cdot(B-A) + A
\\ d = \frac{\|\overrightarrow{A P} \times \overrightarrow{A B}\|}{\overrightarrow{AB}}
$$
第二种情况：

![image-20220820162647419](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220820162647419.png)
$$
d = min(d(P,B),d(P,A))
$$
在juyter notebook上绘制包裹自身小车和另一个的框：

```python
ego_car = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73],
                   [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])


plt.axes().set_aspect('equal', 'datalim')
plt.plot(ego_car[:5, 0], ego_car[:5, 1], '-o')
plt.plot(corners_3d_velo.T[:5, 0], corners_3d_velo.T[:5, 1], '-o')
```

![image-20220820163248021](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220820163248021.png)

```python
fig, ax = plt.subplots(figsize=(20, 10))
draw_point_cloud(ax, points[::5], axes=[0,1])
draw_box(ax, corners_3d_velo, axes=[0, 1], color='r')
draw_box(ax, ego_car.T, axes=[0, 1], color='b')
```

下面写的就是怎么计算两个框间的最小距离：

首先将计算距离的公式编写code：

```python
# 该函数返回类型 距离, 坐标
def distance_point_to_segment(P, A, B):
    AP = P - A
    BP = P - B
    AB = B - A
    if np.dot(AB, AP)>=0 and np.dot(-AB, BP)>=0:     #向量点积,投影在线上
        return np.abs(np.cross(AP, AB))/np.linalg.norm(AB), np.dot(AP, AB)/np.dot(AB, AB) * AB + A
    
    d_PA = np.linalg.norm(AP)
    d_PB = np.linalg.norm(BP)
    # 投影在线外
    if d_PA < d_PB:
        return d_PA, A
    return d_PB, B
```

然后定义函数用于计算框中的顶点距离：

```python
def min_distance_cupoints(cub1, cub2):
    minD = 1e5   #先给一个非常大的值
    # 最短距离在本车的端点
    for i in range(4):    #四个顶点遍历
        for j in range(4): #另一个长方形的四条边遍历
            d, Q = distance_point_to_segment(cub1[i, :2], cub2[j, :2], cub2[j+1, :2])
            if d < minD:  #迭代最小的几个点的数值
                minD = d
                minP = cub1[i, :2]
                minQ = Q
                
    # 最短距离在其他物体的端点
    for i in range(4):    #四个顶点遍历
        for j in range(4): #另一个长方形的四条边遍历
            d, Q = distance_point_to_segment(cub2[i, :2], cub1[j, :2], cub1[j+1, :2])
            if d < minD:  #迭代最小的几个点的数值
                minD = d
                minP = cub2[i, :2]
                minQ = Q
   
    # 遍历完毕
    return minP, minQ, minD
```

对于已知的两个框，可以调用如下的代码：

```python
plt.axes().set_aspect('equal', 'datalim')
plt.plot(ego_car[:5, 0], ego_car[:5, 1], '-o')
plt.plot(corners_3d_velo.T[:5, 0], corners_3d_velo.T[:5, 1], '-o')
minP, minQ, minD = min_distance_cupoints(ego_car, corners_3d_velo.T)
plt.plot((minP[0], minQ[0]), (minP[1], minQ[1]), 'y-')
print(minD)
```

![image-20220820195924041](C:\Users\13049\AppData\Roaming\Typora\typora-user-images\image-20220820195924041.png)

## 16、将距离信息计算并发布

kitti.py中需要

```python
# 车子8个点的坐标,框,根据车子的数据图示给出
EGOCAR = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73],
                   [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])

dist_pub = rospy.Publisher("kitti_dist", MarkerArray, queue_size=10) #发布者,发布每两个物体之间的距离
...
publish_dist(dist_pub, minPQDs)
...
```

自定义个misc.py文件，并将计算距离的公式复现在其中：

```python
!usr/bin/python
import numpy as np
def distance_point_to_segment(P, A, B):
    AP = P - A
    BP = P - B
    AB = B - A
    if np.dot(AB, AP)>=0 and np.dot(-AB, BP)>=0:     #向量点积,投影在线上
        return np.abs(np.cross(AP, AB))/np.linalg.norm(AB), np.dot(AP, AB)/np.dot(AB, AB) * AB + A
    
    d_PA = np.linalg.norm(AP)
    d_PB = np.linalg.norm(BP)
    # 投影在线外
    if d_PA < d_PB:
        return d_PA, A
    return d_PB, B
    
def min_distance_cupoints(cub1, cub2):
    minD = 1e5   #先给一个非常大的值
    # 最短距离在本车的端点
    for i in range(4):    #四个顶点遍历
        for j in range(4): #另一个长方形的四条边遍历
            d, Q = distance_point_to_segment(cub1[i, :2], cub2[j, :2], cub2[j+1, :2])
            if d < minD:  #迭代最小的几个点的数值
                minD = d
                minP = EGOCAR[i, :2]
                minQ = Q
                
    # 最短距离在其他物体的端点
    for i in range(4):    #四个顶点遍历
        for j in range(4): #另一个长方形的四条边遍历
            d, Q = distance_point_to_segment(cub2[i, :2], cub1[j, :2], cub1[j+1, :2])
            if d < minD:  #迭代最小的几个点的数值
                minD = d
                minP = corners_3d_velo[i, :2]
                minQ = Q
   
    # 遍历完毕
    return minP, minQ, minD
```

在publish_utils.py中写publish_dist函数：

```python
def publish_dist(dist_pub, minPQDs):
    marker_array = MarkerArray()

    for i, (minP, minQ, minD) in enumerate(minPQDs):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFETIME)
        marker.type = Marker.LINE_STRIP

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.scale.x = 0.1

        marker.points = []
        marker.points.append(Point(minP[0], minP[1], 0))
        marker.points.append(Point(minQ[0], minQ[1], 0))

        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = rospy.Time.now()

        text_marker.id = i + 1000  ##为了和上面的marker不同
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(LIFETIME)
        text_marker.type = Marker.TEXT_VIEW_FACING
        #显示文字的位置
        p = (minP + minQ) / 2.0
        text_marker.pose.position.x = p[0]
        text_marker.pose.position.y = p[1]
        text_marker.pose.position.z = 0.0
        #显示文字的内容.取两个百分数
        text_marker.text = '%.2f'%minD

        text_marker.scale.x = 1 
        text_marker.scale.y = 1
        text_marker.scale.z = 1

        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 0.8
        marker_array.markers.append(text_marker)

    dist_pub.publish(marker_array)
```

