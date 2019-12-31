/*
 * Paper与代码的差别:
 * paper中使用LM方法进行优化,而代码中使用的是高斯牛顿法.
 * paper中使用的是angle-axis进行优化,而代码中使用的是旋转矩阵直接对欧拉角求导,最终优化的是欧拉角
 */
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
using namespace std;
using namespace Eigen;

//一个点云周期
const float scanPeriod = 0.1;
//typedef Twist<float> Transform;
Twist <float> transform_estimated_;
Twist <float> transform_sum_;

//跳帧数，控制发给laserMapping的频率
const int skipFrameNum = 1;
bool systemInited = false;

//时间戳信息
double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

//消息接收标志
bool newCornerPointsSharp = false;
bool newCornerPointsLessSharp = false;
bool newSurfPointsFlat = false;
bool newSurfPointsLessFlat = false;
bool newLaserCloudFullRes = false;

PointCloudPtr cornerPointsSharp(new pcl::PointCloud<PointType>());
PointCloudPtr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
PointCloudPtr surfPointsFlat(new pcl::PointCloud<PointType>());
PointCloudPtr surfPointsLessFlat(new pcl::PointCloud<PointType>());
PointCloudPtr laserCloudCornerLast(new pcl::PointCloud<PointType>());
PointCloudPtr laserCloudSurfLast(new pcl::PointCloud<PointType>());
PointCloudPtr laserCloudOri(new pcl::PointCloud<PointType>());//匹配到的点的个数(即存在多少个约束)
PointCloudPtr coeffSel(new pcl::PointCloud<PointType>());
PointCloudPtr laserCloudFullRes(new pcl::PointCloud<PointType>());


//kd-tree built by less sharp points of last frame
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<PointType>());
//kd-tree built by less flat points of last frame
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<PointType>());

int laserCloudCornerLastNum;
int laserCloudSurfLastNum;

//save 2 corner points index searched
float pointSearchCornerInd1[40000]; //j
float pointSearchCornerInd2[40000];  //l

//save 3 surf points index searched
float pointSearchSurfInd1[40000]; //j
float pointSearchSurfInd2[40000];//l
float pointSearchSurfInd3[40000];//m

void Publish_Message(Twist <float> transform_ES, Twist <float> transform_sum, nav_msgs::Odometry laserOdometry, ros::Publisher pubLaserOdometry,
		tf::StampedTransform laserOdometryTrans, tf::TransformBroadcaster tfBroadcaster, ros::Publisher pubLaser2Last, nav_msgs::Odometry laserOdometry_last) {
	geometry_msgs::Quaternion geo_quat;
	geo_quat.x = transform_sum.rot.x();
	geo_quat.y = transform_sum.rot.y();
	geo_quat.z = transform_sum.rot.z();
	geo_quat.w = transform_sum.rot.w();
	
	laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
	laserOdometry.pose.pose.orientation = geo_quat;
	laserOdometry.pose.pose.position.x = transform_sum_.pos.x();
	laserOdometry.pose.pose.position.y = transform_sum_.pos.y();
	laserOdometry.pose.pose.position.z = transform_sum_.pos.z();
	pubLaserOdometry.publish(laserOdometry);
	
	// 广播新的平移旋转之后的坐标系(rviz)
	laserOdometryTrans.stamp_ = ros::Time().fromSec(timeSurfPointsLessFlat);
	laserOdometryTrans.setRotation(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w));
	laserOdometryTrans.setOrigin(tf::Vector3(transform_sum.pos.x(), transform_sum.pos.y(), transform_sum.pos.z()));
	tfBroadcaster.sendTransform(laserOdometryTrans);
	
	// for test
	geo_quat.x = transform_ES.rot.x();
	geo_quat.y = transform_ES.rot.y();
	geo_quat.z = transform_ES.rot.z();
	geo_quat.w = transform_ES.rot.w();
	laserOdometry_last.pose.pose.orientation = geo_quat;
	laserOdometry_last.pose.pose.position.x = transform_ES.pos.x();
	laserOdometry_last.pose.pose.position.y = transform_ES.pos.y();
	laserOdometry_last.pose.pose.position.z = transform_ES.pos.z();
	pubLaser2Last.publish(laserOdometry_last);

}
// 当前点云中的点相对第一个点去除因匀速运动产生的畸变，效果相当于得到在点云扫描开始位置静止扫描得到的点云

void TransformToStart(const PointType &pi, PointType &po) {
	float s = 10 * (pi.intensity - floor(pi.intensity));
	
	po.x = pi.x - s * transform_estimated_.pos.x();
	po.y = pi.y - s * transform_estimated_.pos.y();
	po.z = pi.z - s * transform_estimated_.pos.z();
	po.intensity = pi.intensity;
	
	Eigen::Quaternionf q_id, q_s, q_e;
	q_e = transform_estimated_.rot;
	q_id.setIdentity();
	// 插值
	q_s = q_id.slerp(s, q_e);
	
	geometryutils::RotatePoint(q_s.conjugate(), po);
}

// 将上一帧点云中的点相对结束位置去除因匀速运动产生的畸变，效果相当于得到在点云扫描结束位置静止扫描得到的点云
void TransformToEnd(PointCloudPtr &PointCloud) {
	std::size_t cloud_size = PointCloud->points.size();
	for (std::size_t i = 0; i < cloud_size; i++){
		PointType &point = PointCloud->points[i];
		// floor 向下取整
		float s = 10 * (point.intensity - floor(point.intensity));
		
		point.x -= s * transform_estimated_.pos.x();
		point.y -= s * transform_estimated_.pos.y();
		point.z -= s * transform_estimated_.pos.z();
		point.intensity = point.intensity;
		
		Eigen::Quaternionf q_id, q_s, q_e;
		q_e = transform_estimated_.rot;
		q_id.setIdentity();
		// 插值
		q_s = q_id.slerp(s, q_e);
		// conjugate共轭
		geometryutils::RotatePoint(q_s.conjugate(), point);
		geometryutils::RotatePoint(q_e, point);
		
		point.x += transform_estimated_.pos.x();
		point.y += transform_estimated_.pos.y();
		point.z += transform_estimated_.pos.z();
		
	}
}
 
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharp2) {
	timeCornerPointsSharp = cornerPointsSharp2->header.stamp.toSec();
	cornerPointsSharp->clear();
	pcl::fromROSMsg(*cornerPointsSharp2, *cornerPointsSharp);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
	newCornerPointsSharp = true;
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharp2){
  timeCornerPointsLessSharp = cornerPointsLessSharp2->header.stamp.toSec();

  cornerPointsLessSharp->clear();
  pcl::fromROSMsg(*cornerPointsLessSharp2, *cornerPointsLessSharp);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cornerPointsLessSharp,*cornerPointsLessSharp, indices);
  newCornerPointsLessSharp = true;
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlat2){
  timeSurfPointsFlat = surfPointsFlat2->header.stamp.toSec();

  surfPointsFlat->clear();
  pcl::fromROSMsg(*surfPointsFlat2, *surfPointsFlat);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*surfPointsFlat,*surfPointsFlat, indices);
  newSurfPointsFlat = true;
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlat2) {
	timeSurfPointsLessFlat = surfPointsLessFlat2->header.stamp.toSec();
	
	surfPointsLessFlat->clear();
	pcl::fromROSMsg(*surfPointsLessFlat2, *surfPointsLessFlat);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, indices);
	newSurfPointsLessFlat = true;
}

// 接收全部点
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2){
//subscribe velodyne_cloud_2  removeNaNFromPointCloud
  timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

  laserCloudFullRes->clear();
  pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloudFullRes,*laserCloudFullRes, indices);
  newLaserCloudFullRes = true;
}


bool HasNewData(){
	return newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat && newSurfPointsLessFlat &&
	newLaserCloudFullRes && fabs(timeCornerPointsSharp - timeSurfPointsLessFlat) < 0.005 &&
	fabs(timeCornerPointsLessSharp - timeSurfPointsLessFlat) < 0.005 &&
	fabs(timeSurfPointsFlat - timeSurfPointsLessFlat) < 0.005 &&
	fabs(timeLaserCloudFullRes - timeSurfPointsLessFlat) < 0.005;
}

void Reset_Status(){
	newCornerPointsSharp = false;
	newCornerPointsLessSharp = false;
	newSurfPointsFlat = false;
	newSurfPointsLessFlat = false;
	newLaserCloudFullRes = false;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "laserOdometry");
	ros::NodeHandle nh;
	// 订阅的5个topic都是从scanRegistration.cpp来的
	ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2,
	                                                                              laserCloudSharpHandler);
	ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2,
	                                                                                  laserCloudLessSharpHandler);
	ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2,
	                                                                           laserCloudFlatHandler);
	ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2,
	                                                                               laserCloudLessFlatHandler);
	// 消除非匀速运动畸变后的所有的点
	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2,
	                                                                              laserCloudFullResHandler);
	ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
	ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2);
	ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
	ros::Publisher pubLaser2Last = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_last", 5);
	
	nav_msgs::Odometry laserOdometry;
	laserOdometry.header.frame_id = "/camera_init";
	laserOdometry.child_frame_id = "/laser_odom";
	
	nav_msgs::Odometry laserOdometry_last;
	laserOdometry_last.header.frame_id = "/camera_init";
	laserOdometry_last.child_frame_id = "/laser_odom";
	
	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform laserOdometryTrans;
	laserOdometryTrans.frame_id_ = "/camera_init";
	laserOdometryTrans.child_frame_id_ = "/laser_odom";
	
	std::vector<int> pointSearchInd;// 搜索到的点序
	std::vector<float> pointSearchSqDis;// 搜索到的点平方距离
	
	PointType pointOri, pointSel, tripod1, tripod2, tripod3, coeff;
	
	// 退化标志
	bool isDegenerate = false;
	// P矩阵，预测矩阵
	Eigen::Matrix<double, 6, 6> matP;
	// skipFrameNum初始化为 1
	int frameCount = skipFrameNum;
	ros::Rate rate(100);
	bool status = ros::ok();
	while (status) {
		ros::spinOnce();
		// 在laserOdometry中配准的source是新一帧的少数特征点的点云 ,target是前一帧的多数特征点的点云.
		if (HasNewData()) {
			Reset_Status();
			// 将第一个点云数据集发送给laserMapping，从下一个点云数据开始处理
			// =====================留意 laserCloudCornerLast 和 laserCloudSurfLast 在后面有什么作用=========================
			// 答：laserCloudCornerLast和laserCloudSurfLast储存上一个sweep获取的特征点，用于寻找当前sweep的特征点的对应关系，然后根据距离进行非线性的优化。然后再最后，last被发送给建图的节点，建立点云地图
			// =========================================================================================================
			// 系统初始化，将获得的第一帧数据存到两个last容器里面，将两个第一帧特征点云发送给建图节点，先建立第一帧地图
			if (!systemInited) {
				// 将cornerPointsLessSharp与laserCloudCornerLast交换,目的保存cornerPointsLessSharp的值下轮使用
				cornerPointsLessSharp.swap(laserCloudCornerLast);
				// 将surfPointLessFlat与laserCloudSurfLast交换，目的保存surfPointsLessFlat的值下轮使用
				surfPointsLessFlat.swap(laserCloudSurfLast);
				// 使用上一帧的特征点构建kd-tree
				kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
				kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
				
				// 将cornerPointsLessSharp和surfPointLessFlat点也即边沿点和平面点分别发送给laserMapping
				sensor_msgs::PointCloud2 laserCloudCornerLast2;
				pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
				laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
				laserCloudCornerLast2.header.frame_id = "/camera";
				pubLaserCloudCornerLast.publish(laserCloudCornerLast2);
				
				sensor_msgs::PointCloud2 laserCloudSurfLast2;
				pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
				laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
				laserCloudSurfLast2.header.frame_id = "/camera";
				pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
				
				systemInited = true;
				continue;
			}
			// 特征点足够多再向下做优化
			if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
				std::vector<int> indices;
				// 防止边缘点点云中出现了NaN
				pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
				// 新一帧点云边缘点数量
				int cornerPointsSharpNum = cornerPointsSharp->points.size();
				// 新一帧点云平面点数量
				int surfPointsFlatNum = surfPointsFlat->points.size();
				
				// Levenberg-Marquardt算法(L-M method)，非线性最小二乘算法，最优化算法的一种
				// 最多迭代25次 (基本不会迭代超过5次就会收敛)
				for (int iterCount = 0; iterCount < 25; iterCount++) {
					laserCloudOri->clear(); //保存匹配距离点对 成功的当前帧点云 的 点 //匹配到的点的个数(即存在多少个约束)
					coeffSel->clear();
					// 处理当前点云中的曲率最大的特征点,从上个点云中曲率比较大的特征点中找两个最近距离点，一个点使用kd-tree查找，另一个根据找到的点在其相邻线找另外一个最近距离的点
					// ===============================开始处理边缘点==============================
					for (int i = 0; i < cornerPointsSharpNum; i++) {
						// 去除运动畸变
						// TODO 改
						TransformToStart(cornerPointsSharp->points[i], pointSel);
						// 每迭代五次，重新查找最近点，在第一次迭代的时候，对于每个边缘点都会寻找一遍最近点
						if (iterCount % 5 == 0) {
							// target点云 上一帧多数特征点的点云
							pcl::removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);
							// 构建KD树(返回两个数组，按照距离近到远排序)
							kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
							// 距离目标点距离最小的点index 和 相邻线距离目标点距离最小的点index
							int closestPointInd = -1, minPointInd2 = -1;
							
							// 寻找相邻线距离目标点距离最小的点
							// TODO 测试输出kd树查找的距离数组
							if (pointSearchSqDis[0] < 25) {
								// 找到的最近点距离的确很近的话  //搜索到的点平方距离
								closestPointInd = pointSearchInd[0];  //最近点的索引
								// 提取最近点线号
								int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);
								// 初始门槛值5米，可大致过滤掉scanID相邻，但实际线不相邻的值
								float pointSqDis, minPointSqDis2 = 25;
								// 寻找距离目标点最近距离的平方和最小的点
								// 向scanID增大的方向查找，就查找下一条线(目前是相邻的上下各两条线
								for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
									if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
										// 非相邻线
										break;
									}
									if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
										// 确保两个点不在同一条scan上（相邻线查找应该可以用scanID == closestPointScan +/- 1 来做）
										pointSqDis = mathutils::CalcSquaredDiff(laserCloudCornerLast->points[j],
										                                        pointSel);
										if (pointSqDis < minPointSqDis2) {
											// 距离更近，要小于初始值5米
											// 更新最小距离与点序
											minPointSqDis2 = pointSqDis;
											minPointInd2 = j;
										}
									}
								}
								// 同理向scanID减小的方向查找
								for (int j = closestPointInd - 1; j > 0; j--) {
									if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
										break;
									}
									if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
										// 计算距离
										pointSqDis = mathutils::CalcSquaredDiff(laserCloudCornerLast->points[j],
										                                        pointSel);
										if (pointSqDis < minPointSqDis2) {
											minPointSqDis2 = pointSqDis;
											minPointInd2 = j;
										}
									}
								}
							}
							// 寻找到的距离最近的两个点
							pointSearchCornerInd1[i] = float(closestPointInd);
							pointSearchCornerInd2[i] = float(minPointInd2);
						}
						// 对于边缘点来说，如果两个对应点都找到了的话，就计算一下边缘点距离两个对应点构成的直线的距离，同时计算一下权重，距离越小，权重越大。如果距离过大就直接舍弃。同时舍弃距离为0的点
						// ===============================思考：为什么要舍弃距离为0的点，同时，为什么从第五次迭代开始，开始减小权重？=====================================
						// ============================猜想：=================================
						// 前五次所有的边缘点都进入非线性优化， 为了获得更准确的数值，如果迭代五次都没有结束，可能不收敛或者收敛速度太慢，因此去除迭代五次距离仍然很远的点，以加快收敛速度
						
						// 只有第一个点存在，第二个点才会开始找，所以只需要一个判断条件就可以了
						if (pointSearchCornerInd2[i] >= 0) {
							// 拿出来找到的上一帧率的距离最近的两个点
							tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
							tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];
							
							// 选择的特征点记为O，kd-tree最近距离点记为1，另一个最近距离点记为2
							float x0 = pointSel.x;
							float y0 = pointSel.y;
							float z0 = pointSel.z;
							float x1 = tripod1.x;
							float y1 = tripod1.y;
							float z1 = tripod1.z;
							float x2 = tripod2.x;
							float y2 = tripod2.y;
							float z2 = tripod2.z;
							// 模为：
							float a012 = std::sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
							                       * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
							                       + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
							                         * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
							                       + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))
							                         * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));
							// 两个最近距离点之间的距离，即向量AB的模
							float l12 = std::sqrt(
									(x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
							// 点到线的距离，d = |向量OA 叉乘 向量OB|/|AB|
							float ld2 = a012 / l12;
							// 向量[la；lb；lc] 为距离ld2分别对[x0 y0 z0](选中的特征点)的偏导，具体推导见IPad：goodnotes：loam笔记
							// ∂(ld2) / ∂(x0)
							float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
							            + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;
							// ∂(ld2) / ∂(y0)
							float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
							             - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
							// ∂(ld2) / ∂(z0)
							float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
							             + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
							// 权重计算，距离越大权重越小，距离越小权重越大，得到的权重范围<=1
							// TODO 这个地方是什么意思？？？
							float s = 1;
							if (iterCount >= 5) {
								// 5次迭代之后开始  减小 权重因素  ???
								s = 1.0f - 1.8f * std::fabs(ld2);
							}
							// 考虑权重
							coeff.x = s * la;
							coeff.y = s * lb;
							coeff.z = s * lc;
							coeff.intensity = s * ld2;
							// NOTE: ld2 <= 0.5
							if (s > 0.1 && ld2 != 0) {
								// 只保留权重大的，也即距离比较小的点，同时也舍弃距离为零的
								laserCloudOri->push_back(cornerPointsSharp->points[i]);
								coeffSel->push_back(coeff);
							}
						}
					}
					// 当前状态：具有了每个点到其对应直线的距离，以及权重，接下来开始对于每个平面点，进行同样的操作
					// 对本次接收到的曲率最小的点,从上次接收到的点云曲率比较小的点中找三点组成平面，一个使用kd-tree查找，另外一个在同一线上查找满足要求的，第三个在不同线上查找满足要求的
					// ===============================开始处理平面点==============================
					for (int i = 0; i < surfPointsFlatNum; i++) {
						// TODO
						TransformToStart(surfPointsFlat->points[i], pointSel);
						// 每隔五次重新寻找最邻近点
						if (iterCount % 5 == 0) {
							// kd-tree最近点查找
							kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
							// 初始化三个要寻找的点的index
							int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
							if (pointSearchSqDis[0] < 25) {
								closestPointInd = pointSearchInd[0];
								// 获取最邻近的点在上一帧所在的scanID
								int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);
								
								float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
								// ===========向scanID增大的方向寻找===========
								for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
									// 找到最邻近的下一条直线去了，就退出
									if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
										break;
									}
									pointSqDis = mathutils::CalcSquaredDiff(laserCloudSurfLast->points[j], pointSel);
									// 如果第二个点和上一个最邻近点在同一条scan中：
									// 即论文中的点j和点l
									if (int(laserCloudSurfLast->points[j].intensity) == closestPointScan) {
										// 如果点的线号小于等于最近点的线号(应该最多取等，也即同一线上的点!!!)
										if (pointSqDis < minPointSqDis2) {
											minPointSqDis2 = pointSqDis;
											minPointInd2 = j;
										}
									}
										// 如果和i不在同一条scan中，即paper中的点m
									else {
										//如果点处在大于该线上
										if (pointSqDis < minPointSqDis3) {
											minPointSqDis3 = pointSqDis;
											minPointInd3 = j;
										}
									}
								}
								// ===========向scanID减小的方向寻找===========
								for (int j = closestPointInd - 1; j >= 0; j--) {
									if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
										break;
									}
									pointSqDis = mathutils::CalcSquaredDiff(laserCloudSurfLast->points[j], pointSel);
									if (int(laserCloudSurfLast->points[j].intensity) == closestPointScan) {
										if (pointSqDis < minPointSqDis2) {
											minPointSqDis2 = pointSqDis;
											minPointInd2 = j;
										}
									} else {
										if (pointSqDis < minPointSqDis3) {
											minPointSqDis3 = pointSqDis;
											minPointInd3 = j;
										}
									}
								}
							}
							pointSearchSurfInd1[i] = closestPointInd;//kd-tree最近距离点,-1表示未找到满足要求的点
							pointSearchSurfInd2[i] = minPointInd2;//同一线号上的距离最近的点，-1表示未找到满足要求的点
							pointSearchSurfInd3[i] = minPointInd3;//不同线号上的距离最近的点，-1表示未找到满足要求的点
						}
						// 这个地方就是必须2 3同时找到了
						if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
							// 找到了三个点
							tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];//A点
							tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];//B点
							tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];//C点
							// 具体推导公式见Ipad goodnotes LOAM笔记
							// 求∂(pd2) / ∂(x0)(少一个模长，下面pa /= ps添加了)
							float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
							           - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
							// 求∂(pd2) / ∂(y0)(少一个模长，下面pb /= ps添加了)
							float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
							           - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
							// 求∂(pd2) / ∂(z0)(少一个模长，下面pc /= ps添加了)
							float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
							           - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
							float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);
							// 法向量的模
							float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
							// pa pb pc为法向量各方向上的单位向量
							pa /= ps;
							pb /= ps;
							pc /= ps;
							pd /= ps;
							// 点到面的距离：向量OA与与法向量的点积除以法向量的模
							float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
							// 同理计算权重
							float s = 1;
							// TODO 和上面边缘点一样，这里没明白为什么
							if (iterCount >= 5) {
								// ==================又一次迷惑，这个地方为什么要这样更新这个s参数？？？？？？？=====================
								s = 1.0f - 1.8f * std::fabs(pd2) / std::sqrt(mathutils::CalcPointDistance(pointSel));
							}
							coeff.x = s * pa;
							coeff.y = s * pb;
							coeff.z = s * pc;
							coeff.intensity = s * pd2;
							if (s > 0.1 && pd2 != 0) {
								laserCloudOri->push_back(surfPointsFlat->points[i]);
								coeffSel->push_back(coeff);
							}
						}
					}
					// 匹配到的点的个数(即存在多少个约束) （包括边缘点和平面点，不包括less的点）
					int pointSelNum = laserCloudOri->points.size();
					// =======================如果获取的特征点太少，不足10个，continue没用？？？？？？应该是break吧？？？=========================
					// 答：不是，后面的迭代中，从五次迭代之后s会变化============================
					if (pointSelNum < 10) {
						continue;
					}
					Eigen::Matrix<double, Eigen::Dynamic, 6> matA(pointSelNum, 6);
					Eigen::Matrix<double, 6, Eigen::Dynamic> matAt(6, pointSelNum);
					Eigen::Matrix<double, 6, 6> matAtA;
					Eigen::Matrix<double, Eigen::Dynamic, 1> matB(pointSelNum);
					Eigen::Matrix<double, 6, 1> matAtB;
					Eigen::Matrix<double, 6, 1> matX;
					
					SO3 R_SO3(transform_estimated_.rot);
					// 计算matA,matB矩阵 Jacobian矩阵  pointSelNum行 6列
					for (int i = 0; i < pointSelNum; i++) {
						/* 采用Levenberg-Marquardt计算
						* 首先建立当前时刻Lidar坐标系下提取到的特征点与点到直线/平面
						* 的约束方程。而后对约束方程求对坐标变换(3旋转+3平移)的偏导
						* 公式参见论文(2)-(8)*/
						pointOri = laserCloudOri->points[i];
						coeff = coeffSel->points[i];
						
						Eigen::Vector3f p(pointOri.x, pointOri.y, pointOri.z);
						Eigen::Vector3f w(coeff.x, coeff.y, coeff.z);
						Eigen::Vector3f J_r = w.transpose() * mathutils::SkewSymmetric(
								transform_estimated_.rot.conjugate() * (p - transform_estimated_.pos));
						Eigen::Vector3f J_t = -w.transpose() * transform_estimated_.rot.toRotationMatrix().transpose();
//						// ？？？？？？？？迷惑行为？？？s = 1
//						// 猜测：之前在transToStart部分已经去除了运动畸变了，所以这里就把s当做1了，
//						// 因为当前时刻，所有点的坐标都是一个sweep的起始坐标，transform是上一个sweep到这个sweep的转移矩阵，
//						// 然后依据允许运动模型，当做和上次的transform是相同的，作为一个优化的起始位姿
//						// J = ∂f / ∂T_k+1
//						// intensity是 s * 距离
						float d2 = coeff.intensity;
						
						matA(i, 0) = J_r.x();
						matA(i, 1) = J_r.y();
						matA(i, 2) = J_r.z();
						matA(i, 3) = J_t.x();
						matA(i, 4) = J_t.y();
						matA(i, 5) = J_t.z();
						// TODO 为什么要乘-0.05
						matB(i, 0) = -0.05 * d2;
					}
					// 最小二乘计算(QR分解法)
					matAt = matA.transpose();
					matAtA = matAt * matA;
					matAtB = matAt * matB;
					// 求解matAtA * matX = matAtB   得到的结果就是△x
					matX = matAtA.colPivHouseholderQr().solve(matAtB);
					// 第一次开始迭代
					if (iterCount == 0) {
						//特征值1*6矩阵F
						Eigen::Matrix<double, 1, 6> matE;
						Eigen::Matrix<double, 6, 6> matV;
						Eigen::Matrix<double, 6, 6> matV2;
						//求解特征值/特征向量
						// 此时的matATA就是 海塞矩阵 H
						Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(matAtA);
						matE = eigen_solver.eigenvalues().real();
						matV = eigen_solver.eigenvectors().real();
						matV2 = matV;
						
						isDegenerate = false;
						// TODO 发生退化要怎么处理
						// 特征值取值门槛
						float eignThre[6] = {10, 10, 10, 10, 10, 10};
						for (int i = 5; i >= 0; i--) {
							//从小到大查找
							if (matE(0, i) < eignThre[i]) {
								// 特征值太小，则认为处在兼并环境中，发生了退化
								for (int j = 0; j < 6; j++) {
									// 对应的特征向量置为0
									matV2(i, j) = 0;
								}
								isDegenerate = true;
							} else {
								break;
							}
						}
						// 计算P矩阵
						// TODO p矩阵是干啥的
//						matP = matV.inverse() * matV2;
						matP = matV2 * matV.inverse();
					}
					
					if (isDegenerate) {
						// 如果发生退化，只使用预测矩阵P计算
						Eigen::Matrix<double, 6, 1> matX2 = matX;
						matX = matP * matX2;
					}

					// 对数映射获取李代数
					Eigen::Vector3f r_so3 = R_SO3.log();
					// 旋转更新量
					r_so3.x() += matX(0, 0);
					r_so3.y() += matX(1, 0);
					r_so3.z() += matX(2, 0);
					
					transform_estimated_.pos.x() += matX(3, 0);
					transform_estimated_.pos.y() += matX(4, 0);
					transform_estimated_.pos.z() += matX(5, 0);
					
					if (!isfinite(r_so3.x())) r_so3.x() = 0;
					if (!isfinite(r_so3.y())) r_so3.y() = 0;
					if (!isfinite(r_so3.z())) r_so3.z() = 0;
					// 利用计算出的△x，更新transform，计算下次的误差累加每次迭代的旋转平移量
					// 搞明白这个公式，或者做好自己的想法
					// 解释 一个旋转量的更新，可以利用q * 【1， 1/2w】 （w代表旋转向量） 来更新旋转量
					transform_estimated_.rot = transform_estimated_.rot *
					                           mathutils::DeltaQ(Eigen::Vector3f(matX(0, 0), matX(1, 0), matX(2, 0)));
					if (!isfinite(transform_estimated_.pos.x())) transform_estimated_.pos.x() = 0.0;
					if (!isfinite(transform_estimated_.pos.y())) transform_estimated_.pos.y() = 0.0;
					if (!isfinite(transform_estimated_.pos.z())) transform_estimated_.pos.z() = 0.0;

					// 计算旋转平移量，如果很小就停止迭代
					float delta_r = mathutils::RadToDeg(
							R_SO3.unit_quaternion().angularDistance(transform_estimated_.rot));
					float delta_t = std::sqrt(
							std::pow(matX(3, 0) * 100.0f, 2.0f) + std::pow(matX(4, 0) * 100.0f, 2.0f) +
							std::pow(matX(5, 0) * 100.0f, 2.0f));
					if (delta_r < .05f && delta_t < .05f) {
						break;
					}
				}
			}
			Twist<float> transform_Start_2_End = transform_estimated_.inverse();
			Twist<float> transform_sum_temp = transform_sum_ * transform_Start_2_End;
			transform_sum_ = transform_sum_temp;
			
			TransformToEnd(cornerPointsLessSharp);
			TransformToEnd(surfPointsLessFlat);
			
			// 归一化
			transform_estimated_.rot.normalize();
			cornerPointsLessSharp.swap(laserCloudCornerLast);
			// 将surfPointLessFlat与laserCloudSurfLast交换，目的保存surfPointsLessFlat的值下轮使用
			surfPointsLessFlat.swap(laserCloudSurfLast);
			laserCloudCornerLastNum = laserCloudCornerLast->points.size();
			laserCloudSurfLastNum = laserCloudSurfLast->points.size();
			if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
				kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
				kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
			}
			
			Publish_Message(transform_estimated_, transform_sum_, laserOdometry, pubLaserOdometry, laserOdometryTrans,
					tfBroadcaster, pubLaser2Last, laserOdometry_last);
			
			frameCount++;
			// 点云全部点，每间隔 skipFrameNum帧点云数据相对点云最后一个点进行畸变校正  投影到 一帧点云 扫描结束的时间点
			// 按照跳帧数publish边沿点，平面点以及全部点给laserMapping(每隔skipFrameNum帧发一次)
			if (frameCount >= skipFrameNum + 1) {
				frameCount = 0;
				TransformToEnd(laserCloudFullRes);
				
				std_msgs::Header temp_Header;
				temp_Header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
				temp_Header.frame_id = "/camera";
				
				ros_comman::PublishCloudMsg(pubLaserCloudCornerLast, *laserCloudCornerLast, temp_Header);
				ros_comman::PublishCloudMsg(pubLaserCloudSurfLast, *laserCloudSurfLast, temp_Header);
				ros_comman::PublishCloudMsg(pubLaserCloudFullRes, *laserCloudFullRes, temp_Header);
			}
		}
		status = ros::ok();
		rate.sleep();
	}
	return 0;
}