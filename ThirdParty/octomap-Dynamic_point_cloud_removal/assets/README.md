Here is just Kin's note when she was working on this repo.

Pub PointCloud
- https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp#L173

```
m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, m_latchedTopics);
``` 

Sub PointCloud cloud in:
```
  m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
  m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
  m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, boost::placeholders::_1));
```

The function we are interested in is `insertCloudCallback`, [Link](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp#L263)
```
void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
```

Then go to the insertScan
```
void OctomapServer::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& ground, const PCLPointCloud& nonground){
```

octomap cannot save intensity, only choice on XYZ or XYZRGB, I didn't implement the RGB.

