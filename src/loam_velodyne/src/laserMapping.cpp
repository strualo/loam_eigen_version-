// 有问题版本
#include <math.h>
#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
using namespace std;
// 扫描周期
const float scanPeriod = 0.1;

// 控制接收到的点云数据，每隔几帧处理一次
const int num_stack_frames_ = 1;
// 控制处理得到的点云map，每隔几次publich给rviz显示
const int mapFrameNum = 5;

// 时间戳
double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

// 接收标志
bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;
bool newLaserOdometry = false;

int laser_cloud_cen_width_ = 10;
int laser_cloud_cen_height_ = 5;
int laser_cloud_cen_length_ = 10;

const int laserCloudWidth = 21;
const int laserCloudHeight = 11;
const int laserCloudLength = 21;
// 点云方块集合最大数量
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudLength;//4851

// lidar视域范围内(FOV)的点云集索引
int laserCloudValidInd[125];
// lidar周围的点云集索引
int laserCloudSurroundInd[125];

PointCloudPtr laserCloudCornerLast(new PointCloud);
PointCloudPtr laserCloudSurfLast(new PointCloud);
PointCloudPtr laser_cloud_corner_stack_(new PointCloud);
PointCloudPtr laser_cloud_surf_stack_(new PointCloud);
PointCloudPtr laser_cloud_corner_stack_downsampled_(new PointCloud);
PointCloudPtr laser_cloud_surf_stack_downsampled_(new PointCloud);
PointCloudPtr laserCloudOri(new PointCloud);
PointCloudPtr coeffSel(new PointCloud);
PointCloudPtr laserCloudSurround(new PointCloud);
PointCloudPtr laserCloudSurround2(new PointCloud);
PointCloudPtr laserCloudCornerFromMap(new PointCloud);
PointCloudPtr laserCloudSurfFromMap(new PointCloud);
PointCloudPtr laserCloudFullRes(new PointCloud);

// array都是以50米为单位的立方体地图，运行过程中会一直保存(有需要的话可考虑优化，只保存近邻的，或者直接数组开小一点)
PointCloudPtr laserCloudCornerArray[laserCloudNum];
PointCloudPtr laserCloudSurfArray[laserCloudNum];
PointCloudPtr laserCloudCornerArray_downSampled[laserCloudNum];
PointCloudPtr laserCloudSurfArray_downSampled[laserCloudNum];

// kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

Transform transform_sum_;
Transform transform_tobe_mapped_;
Transform transform_bef_mapped_;
Transform transform_aft_mapped_;

// 猜测一个新的Lidar坐标系在世界坐标系下的位置
// ======每次进来，只是要了一个上一次和这一次的之间的变换，而不是累积变换，利用这个增加的变换来更新上次的transform_tobe_mapped_
// ======就像里程计一样，里程计有累积误差，所以我每次就要上次到这次之间的这个变换就够了
void transformAssociateToMap() {
	Transform transform_incre(transform_bef_mapped_.inverse() * transform_sum_.transform());
	transform_tobe_mapped_ = transform_tobe_mapped_ * transform_incre;
}

// 记录odometry发送的转换矩阵与mapping之后的转换矩阵，下一帧点云会使用(有IMU的话会使用IMU进行补偿)
void transformUpdate() {
	transform_bef_mapped_ = transform_sum_;
	transform_aft_mapped_ = transform_tobe_mapped_;
}

// 根据调整计算后的转移矩阵，将点注册到全局世界坐标系下
void pointAssociateToMap(const PointType &pi, PointType &po) {
	po.x = pi.x;
	po.y = pi.y;
	po.z = pi.z;
	po.intensity = pi.intensity;
	
	geometryutils::RotatePoint(transform_tobe_mapped_.rot, po);
	
	po.x += transform_tobe_mapped_.pos.x();
	po.y += transform_tobe_mapped_.pos.y();
	po.z += transform_tobe_mapped_.pos.z();
}

// 点转移到局部坐标系下
void pointAssociateTobeMapped(const PointType &pi, PointType &po) {
	po.x = pi.x - transform_tobe_mapped_.pos.x();
	po.y = pi.y - transform_tobe_mapped_.pos.y();
	po.z = pi.z - transform_tobe_mapped_.pos.z();
	po.intensity = pi.intensity;
	
	geometryutils::RotatePoint(transform_tobe_mapped_.rot.conjugate(), po);
}

// 接收边沿点
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2) {
	timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();
	laserCloudCornerLast->clear();
	pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);
	newLaserCloudCornerLast = true;
}

// 接收平面点
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2) {
	timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();
	laserCloudSurfLast->clear();
	pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);
	newLaserCloudSurfLast = true;
}

// 接收点云全部点
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) {
	timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
	laserCloudFullRes->clear();
	pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
	newLaserCloudFullRes = true;
}

// 接收旋转平移信息
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) {
	timeLaserOdometry = laserOdometry->header.stamp.toSec();
	//四元数转换为欧拉角
	geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
	transform_sum_.rot.w() = geoQuat.w;
	transform_sum_.rot.x() = geoQuat.x;
	transform_sum_.rot.y() = geoQuat.y;
	transform_sum_.rot.z() = geoQuat.z;
	
	transform_sum_.pos.x() = float(laserOdometry->pose.pose.position.x);
	transform_sum_.pos.y() = float(laserOdometry->pose.pose.position.y);
	transform_sum_.pos.z() = float(laserOdometry->pose.pose.position.z);
	
	newLaserOdometry = true;
}

void Reset_status(){
	newLaserCloudCornerLast = false;
	newLaserCloudSurfLast = false;
	newLaserCloudFullRes = false;
	newLaserOdometry = false;
}

size_t ToIndex(int i, int j, int k) {
	return i + 21 * j + 21 * 21 * k;
}

size_t FromIndex(const size_t &index, int &i, int &j, int &k) {
	int residual = index % (21 * 21);
	k = index / (21 * 21);
	j = residual / 21;
	i = residual % 21;
}


bool HasNewdata(){
	return newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
	       fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
	       fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
	       fabs(timeLaserCloudFullRes - timeLaserOdometry) < 0.005;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "laserMapping");
	ros::NodeHandle nh;
	
	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2,
	                                                                                 laserCloudCornerLastHandler);
	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2,
	                                                                               laserCloudSurfLastHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, laserOdometryHandler);
	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2,
	                                                                              laserCloudFullResHandler);
	// 最终的地图
	ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
	ros::Publisher pubLaserCloud_Corner_1 = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_Corner_1", 1);
	ros::Publisher pubLaserCloud_Surf_1 = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_Surf_1", 1);
	ros::Publisher pubLaserCloud_Corner_2 = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_Corner_2", 1);
	ros::Publisher pubLaserCloud_Surf_2 = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_Surf_2", 1);
	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2);
	ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);
	
	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "/camera_init";
	odomAftMapped.child_frame_id = "/aft_mapped";
	
	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform aftMappedTrans;
	aftMappedTrans.frame_id_ = "/camera_init";
	aftMappedTrans.child_frame_id_ = "/aft_mapped";
	
	PointType pointSel;
	
	bool isDegenerate = false;
	// 创建VoxelGrid滤波器（体素栅格滤波器）
	VoxelGrid downSizeFilterCorner;
	downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);	// lio-mapping
	
	VoxelGrid downSizeFilterSurf;
	downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);	// lio-mapping
	
	VoxelGrid downSizeFilterMap;
	downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);
	
	// 指针初始化
	for (int i = 0; i < laserCloudNum; i++) {
		laserCloudCornerArray[i].reset(new PointCloud);
		laserCloudSurfArray[i].reset(new PointCloud);
		laserCloudCornerArray_downSampled[i].reset(new PointCloud);
		laserCloudSurfArray_downSampled[i].reset(new PointCloud);
	}
	
	int frame_count_ = num_stack_frames_ - 1;   // initialize as 0
	int mapFrameCount = mapFrameNum - 1;  // initialize as 4
	ros::Rate rate(100);
	bool status = ros::ok();
	while (status) {
		ros::spinOnce();
		// 在laserMapping中配准的source是新一帧的点云,target是前面的点云拼接成的地图.
		if (HasNewdata()) {
			Reset_status();
			
			frame_count_++;
			// 控制跳帧数，>=这里实际并没有跳帧，只取>或者增大stackFrameNum才能实现相应的跳帧处理
			// frameCount初始化为 num_stack_frames_ - 1 ，所以每次都会进入if判断
			if (frame_count_ >= num_stack_frames_) {
				
				frame_count_ = 0;
				
				// 获取一个先验位姿 transform_tobe_mapped
				transformAssociateToMap();
				// 将最新接收到的激光点云变换到地图坐标系下（Paper中的 P_k+1 -> Q_k+1）
				// transform是依据上面一行预变换之后的transform_tobe_mapped
				int laser_cloud_corner_last_size = laserCloudCornerLast->points.size();
				for (int i = 0; i < laser_cloud_corner_last_size; i++) {
					pointAssociateToMap(laserCloudCornerLast->points[i], pointSel);
					laser_cloud_corner_stack_->push_back(pointSel);
				}
				int laser_cloud_surf_last_size = laserCloudSurfLast->points.size();
				for (int i = 0; i < laser_cloud_surf_last_size; i++) {
					pointAssociateToMap(laserCloudSurfLast->points[i], pointSel);
					laser_cloud_surf_stack_->push_back(pointSel);
				}
				
				// 这个点就是用来判断是否在视野范围内的
				PointType pointOnZAxis;
				pointOnZAxis.x = 0.0;
				pointOnZAxis.y = 0.0;
				pointOnZAxis.z = 10.0;
				
				// TODO 获取Z方向上10米高位置的点在世界坐标系下的坐标
				// ================有什么用？？？？？？？？===============
				pointAssociateToMap(pointOnZAxis, pointOnZAxis);
				
				// 立方体中点在世界坐标系下的（原点）位置
				// 过半取一（以50米进行四舍五入的效果），由于数组下标只能为正数，而地图可能建立在原点前后，因此
				// 每一维偏移一个laserCloudCenWidth（该值会动态调整，以使得数组利用最大化，初始值为该维数组长度1/2）的量
				int centerCubeI = int((transform_tobe_mapped_.pos.x() + 25.0) / 50.0) + laser_cloud_cen_length_; // 10
				int centerCubeJ = int((transform_tobe_mapped_.pos.y() + 25.0) / 50.0) + laser_cloud_cen_width_; // 10
				int centerCubeK = int((transform_tobe_mapped_.pos.z() + 25.0) / 50.0) + laser_cloud_cen_height_; // 5
//				cout << transform_tobe_mapped_.pos.x() << "\t" << transform_tobe_mapped_.pos.y() << "\t" << transform_tobe_mapped_.pos.z() << "\n";
//				std::cout << "centerCubeI" << centerCubeI << "centerCubeJ" << centerCubeJ << "centerCubeK" << centerCubeK << std::endl;
				// TODO 由于计算机求余是向零取整，为了不使（-50.0,50.0）求余后都向零偏移，当被求余数为负数时求余结果统一向左偏移一个单位，也即减一
				if (transform_tobe_mapped_.pos.x() + 25.0 < 0) --centerCubeI;
				if (transform_tobe_mapped_.pos.y() + 25.0 < 0) --centerCubeJ;
				if (transform_tobe_mapped_.pos.z() + 25.0 < 0) --centerCubeK;
//				std::cout <<"++++++++++++++++++++++++++++++++++++++++++---------------------------------------------\n";
//				std::cout << "centerCubeI" << centerCubeI << "centerCubeJ" << centerCubeJ << "centerCubeK" << centerCubeK << std::endl;
				// 调整之后取值范围:3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18
				// 如果处于下边界，表明地图向负方向延伸的可能性比较大，则循环移位，将数组中心点向上边界调整一个单位
				// 先不看
				/*{
					while (centerCubeI < 3) {
						std::cout << "inininiin\n";
						for (int j = 0; j < laserCloudWidth; j++) {
							for (int k = 0; k < laserCloudHeight; k++) {
								// 实现一次循环移位效果
								// 循环移位，I维度上依次后移
								for (int i = laserCloudLength - 1; i >= 1; i--) {
									const size_t index_a = ToIndex(i, j, k);
									const size_t index_b = ToIndex(i - 1, j, k);
									std::swap(laserCloudCornerArray[index_a], laserCloudCornerArray[index_b]);
									std::swap(laserCloudSurfArray[index_a], laserCloudSurfArray[index_b]);
								}
								laserCloudCornerArray[ToIndex(0, j, k)]->clear();
								laserCloudSurfArray[ToIndex(0, j, k)]->clear();
							}
						}
						// TODO 查看这里和 centerCubeI++的区别
						++centerCubeI;
						++laser_cloud_cen_length_;
					}
					// 如果处于上边界，表明地图向正方向延伸的可能性比较大，则循环移位，将数组中心点向下边界调整一个单位
					while (centerCubeI >= laserCloudLength - 3) {
						std::cout << "inininiin\n";
						for (int j = 0; j < laserCloudWidth; j++) {
							for (int k = 0; k < laserCloudHeight; k++) {
								//I维度上依次前移
								for (int i = 0; i < laserCloudLength - 1; i++) {
									const size_t index_a = ToIndex(i, j, k);
									const size_t index_b = ToIndex(i + 1, j, k);
									std::swap(laserCloudCornerArray[index_a], laserCloudCornerArray[index_b]);
									std::swap(laserCloudSurfArray[index_a], laserCloudSurfArray[index_b]);
								}
								laserCloudCornerArray[ToIndex(laserCloudLength - 1, j, k)]->clear();
								laserCloudSurfArray[ToIndex(laserCloudLength - 1, j, k)]->clear();
							}
						}
						--centerCubeI;
						--laser_cloud_cen_length_;
					}
					while (centerCubeJ < 3) {
						std::cout << "inininiin\n";
						for (int i = 0; i < laserCloudLength; i++) {
							for (int k = 0; k < laserCloudHeight; k++) {
								//J维度上，依次后移
								for (int j = laserCloudWidth - 1; j >= 1; j--) {
									const size_t index_a = ToIndex(i, j, k);
									const size_t index_b = ToIndex(i, j - 1, k);
									std::swap(laserCloudCornerArray[index_a], laserCloudCornerArray[index_b]);
									std::swap(laserCloudSurfArray[index_a], laserCloudSurfArray[index_b]);
								}
								laserCloudCornerArray[ToIndex(i, 0, k)]->clear();
								laserCloudSurfArray[ToIndex(i, 0, k)]->clear();
							}
						}
						++centerCubeJ;
						++laser_cloud_cen_width_;
					}
					while (centerCubeJ >= laserCloudWidth - 3) {
						std::cout << "inininiin\n";
						for (int i = 0; i < laserCloudLength; i++) {
							for (int k = 0; k < laserCloudHeight; k++) {
								//J维度上一次前移
								for (int j = 0; j < laserCloudWidth - 1; j++) {
									const size_t index_a = ToIndex(i, j, k);
									const size_t index_b = ToIndex(i, j + 1, k);
									std::swap(laserCloudCornerArray[index_a], laserCloudCornerArray[index_b]);
									std::swap(laserCloudSurfArray[index_a], laserCloudSurfArray[index_b]);
								}
								laserCloudCornerArray[ToIndex(i, laserCloudWidth - 1, k)]->clear();
								laserCloudSurfArray[ToIndex(i, laserCloudWidth - 1, k)]->clear();
							}
						}
						--centerCubeJ;
						--laser_cloud_cen_width_;
					}
					while (centerCubeK < 3) {
						std::cout << "inininiin\n";
						for (int i = 0; i < laserCloudLength; i++) {
							for (int j = 0; j < laserCloudWidth; j++) {
								//K维度上依次后移
								for (int k = laserCloudHeight - 1; k >= 1; k--) {
									const size_t index_a = ToIndex(i, j, k);
									const size_t index_b = ToIndex(i, j, k - 1);
									std::swap(laserCloudCornerArray[index_a], laserCloudCornerArray[index_b]);
									std::swap(laserCloudSurfArray[index_a], laserCloudSurfArray[index_b]);
								}
								laserCloudCornerArray[ToIndex(i, j, 0)]->clear();
								laserCloudSurfArray[ToIndex(i, j, 0)]->clear();
							}
						}
						++centerCubeK;
						++laser_cloud_cen_height_;
					}
					while (centerCubeK >= laserCloudHeight - 3) {
						std::cout << "inininiin\n";
						for (int i = 0; i < laserCloudLength; i++) {
							for (int j = 0; j < laserCloudWidth; j++) {
								for (int k = 0; k < laserCloudHeight - 1; k++) {
									const size_t index_a = ToIndex(i, j, k);
									const size_t index_b = ToIndex(i, j, k + 1);
									std::swap(laserCloudCornerArray[index_a], laserCloudCornerArray[index_b]);
									std::swap(laserCloudSurfArray[index_a], laserCloudSurfArray[index_b]);
								}
								laserCloudCornerArray[ToIndex(i, j, laserCloudHeight - 1)]->clear();
								laserCloudSurfArray[ToIndex(i, j, laserCloudHeight - 1)]->clear();
							}
						}
						--centerCubeK;
						--laser_cloud_cen_height_;
					}
				}*/
				
				int laserCloudValidNum = 0;
				int laserCloudSurroundNum = 0;
				
				// 在每一维附近5个cube(前2个，后2个，中间1个)里进行查找（前后250米范围内，总共500米范围），三个维度总共125个cube
				// 在这125个cube里面进一步筛选在视域范围内的cube
				for (int i = centerCubeI - 2; i <= centerCubeI + 2; ++i) {
					for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; ++j) {
						for (int k = centerCubeK - 2; k <= centerCubeK + 2; ++k) {
							if (i >= 0 && i < laserCloudLength &&
							    j >= 0 && j < laserCloudWidth &&
							    k >= 0 && k < laserCloudHeight) {
								
								// TODO 什么用？ 如果索引合法
								// 换算成实际比例，在世界坐标系下的坐标
								float centerX = 50.0f * float(i - laser_cloud_cen_length_);
								float centerY = 50.0f * float(j - laser_cloud_cen_width_);
								float centerZ = 50.0f * float(k - laser_cloud_cen_height_);
								
								PointType transform_pos;
								transform_pos.x = transform_tobe_mapped_.pos.x();
								transform_pos.y = transform_tobe_mapped_.pos.y();
								transform_pos.z = transform_tobe_mapped_.pos.z();
								
								// 判断是否在lidar视线范围的标志（Field of View）
								bool isInLaserFOV = false;
								for (int ii = -1; ii <= 1; ii += 2) {
									for (int jj = -1; jj <= 1; jj += 2) {
										for (int kk = -1; kk <= 1; kk += 2) {
											// 上下左右八个顶点坐标
											PointType corner;
											corner.x = centerX + 25.0f * float(ii);
											corner.y = centerY + 25.0f * float(jj);
											corner.z = centerZ + 25.0f * float(kk);
											// 原点到顶点距离的平方
											float squaredSide1 = mathutils::CalcSquaredDiff(transform_pos, corner);
											// pointOnZAxis到顶点距离的平方
											float squaredSide2 = mathutils::CalcSquaredDiff(pointOnZAxis, corner);
											
											float check1 = 100.0f + squaredSide1 - squaredSide2
											               - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);
											
											float check2 = 100.0f + squaredSide1 - squaredSide2
											               + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);
											// TODO 什么意思？
											if (check1 < 0 && check2 > 0) {
												// within +-60 degree
												//if |100 + squaredSide1 - squaredSide2| < 10.0 * sqrt(3.0) * sqrt(squaredSide1)
												isInLaserFOV = true;
											}
										}
									}
								}
								
								size_t cube_index = ToIndex(i, j, k);
								
								// 记住视域范围内的cube索引，匹配用
								if (isInLaserFOV) {
									laserCloudValidInd[laserCloudValidNum] = cube_index;
									laserCloudValidNum++;
								}
								//记住附近所有cube的索引，显示用
								laserCloudSurroundInd[laserCloudSurroundNum] = cube_index;
								laserCloudSurroundNum++;
							}
						}
					}
				}
				
				laserCloudCornerFromMap->clear();
				laserCloudSurfFromMap->clear();
				
				// 构建特征点地图，查找匹配使用
				for (int i = 0; i < laserCloudValidNum; i++) {
					// TODO laserCloudCornerArray存放的是什么 什么时候存放的(看上面)
					// ====获取地图中的和选中的点靠近的边缘点和平面点
					*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
//					cout << "===========" << laserCloudCornerArray[laserCloudValidInd[i]]->size() << endl;
					*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
				}
				
				// TODO 上面将点变换到laserCloudCornerStack2之后就没有用过，测试是不是没有必要，这两个步骤
				int laserCloudCornerStackNum2 = laser_cloud_corner_stack_->points.size();
				for (int i = 0; i < laserCloudCornerStackNum2; ++i) {
					// ====pointAssociateTobeMapped是用transform_tobe_mapped反变换回去
					pointAssociateTobeMapped(laser_cloud_corner_stack_->points[i], laser_cloud_corner_stack_->points[i]);
				}
				
				int laserCloudSurfStackNum2 = laser_cloud_surf_stack_->points.size();
				for (int i = 0; i < laserCloudSurfStackNum2; ++i) {
					pointAssociateTobeMapped(laser_cloud_surf_stack_->points[i], laser_cloud_surf_stack_->points[i]);
				}
				
				std_msgs::Header temp_Header_2;
				temp_Header_2.stamp = ros::Time().fromSec(timeLaserOdometry);
				temp_Header_2.frame_id = "/aft_mapped";
				ros_comman::PublishCloudMsg(pubLaserCloud_Corner_2, *laser_cloud_corner_stack_, temp_Header_2);
				ros_comman::PublishCloudMsg(pubLaserCloud_Surf_2, *laser_cloud_surf_stack_, temp_Header_2);
				
				// 降采样
				laser_cloud_corner_stack_downsampled_->clear();
				downSizeFilterCorner.setInputCloud(laserCloudCornerLast);// 设置滤波对象
				downSizeFilterCorner.filter(*laser_cloud_corner_stack_downsampled_);// 执行滤波处理
				size_t laserCloudCornerStackNum = laser_cloud_corner_stack_downsampled_->points.size();// 获取滤波后体素点尺寸
				
				laser_cloud_surf_stack_downsampled_->clear();
				downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
				downSizeFilterSurf.filter(*laser_cloud_surf_stack_downsampled_);
				size_t laserCloudSurfStackNum = laser_cloud_surf_stack_downsampled_->points.size();
				
				laser_cloud_corner_stack_->clear();
				laser_cloud_surf_stack_->clear();
				
				// 优化开始
				int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
				int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();
				if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {
					std::vector<int> pointSearchInd(5, 0);
					std::vector<float> pointSearchSquareDis(5, 0);
					
					PointType pointOri, coeff;
					
					kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
					kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
					
					// TODO 这些矩阵是干啥的
					Eigen::Matrix <float, 5, 3> matA0;
					Eigen::Matrix <float, 5, 1> matB0;
					Eigen::Vector3f matX0;
					Eigen::Matrix3f matA1;
					Eigen::Matrix <float, 1, 3> matD1;
					Eigen::Matrix <float, 3, 3> matV1;
					Eigen::Matrix <float, 6, 6> matP;
					
					matA0.setZero();
					matB0.setConstant(-1);
					matX0.setZero();
					matA1.setZero();
					matD1.setZero();
					matV1.setZero();
					matP.setIdentity();
					
					for (size_t iterCount = 0; iterCount < 10; iterCount++) {
						laserCloudOri->clear();
						coeffSel->clear();
						// ========================开始处理边缘点========================
						for (int i = 0; i < laserCloudCornerStackNum; i++) {
							pointOri = laser_cloud_corner_stack_downsampled_->points[i];
							// 转换回世界坐标系
							// =====这里又变换到地图坐标系了，上面的反变换是不是没什么用
							pointAssociateToMap(pointOri, pointSel);
							// 寻找最近距离五个点
							kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSquareDis);
							// 5个点中最大距离不超过1才处理
							if (pointSearchSquareDis[4] < 1.0) {
								// 将五个最近点的坐标加和求平均
								Eigen::Vector3f vc(0.0f, 0.0f, 0.0f);
								for (int j = 0; j < 5; j++) {
									vc.x() += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
									vc.y() += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
									vc.z() += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
								}
								vc /= 5.0f;
								
								Eigen::Matrix3f mat_a;
								mat_a.setZero();
								
								// 求均方差
								for (int j = 0; j < 5; j++) {
									const PointType point_sel_tmp = laserCloudCornerFromMap->points[pointSearchInd[j]];
									Eigen::Vector3f a;
									a.x() = point_sel_tmp.x - vc.x();
									a.y() = point_sel_tmp.y - vc.y();
									a.z() = point_sel_tmp.z - vc.z();
									
									mat_a(0, 0) += a.x() * a.x();
									mat_a(1, 0) += a.x() * a.y();
									mat_a(2, 0) += a.x() * a.z();
									mat_a(1, 1) += a.y() * a.y();
									mat_a(2, 1) += a.y() * a.z();
									mat_a(2, 2) += a.z() * a.z();
								}
								matA1 = mat_a / 5.0f;
								Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
								matD1 = esolver.eigenvalues().real();
								matV1 = esolver.eigenvectors().real();
								// 如果最大的特征值大于第二大的特征值三倍以上
								if (matD1(0, 2) > 3 * matD1(0, 1)) {
									
									float x0 = pointSel.x;
									float y0 = pointSel.y;
									float z0 = pointSel.z;
									// 边缘点，直线上选两个点，进行优化
									float x1 = float(vc.x()) + 0.1f * matV1(0, 2);
									float y1 = float(vc.y()) + 0.1f * matV1(1, 2);
									float z1 = float(vc.z()) + 0.1f * matV1(2, 2);
									
									float x2 = float(vc.x()) - 0.1f * matV1(0, 2);
									float y2 = float(vc.y()) - 0.1f * matV1(1, 2);
									float z2 = float(vc.z()) - 0.1f * matV1(2, 2);
									
									Eigen::Vector3f X0(x0, y0, z0);
									Eigen::Vector3f X1(x1, y1, z1);
									Eigen::Vector3f X2(x2, y2, z2);


									//  向量叉乘
									Eigen::Vector3f a012_vector = (X0 - X1).cross(X0 - X2);
									// normalized 向量每个值除以摸长（归一化）
									Eigen::Vector3f normal_to_point = ((X1 - X2).cross(a012_vector)).normalized();
									// 求向量的摸长
									float a012 = a012_vector.norm();
									float l12 = (X1 - X2).norm();

									float la = normal_to_point.x();
									float lb = normal_to_point.y();
									float lc = normal_to_point.z();

									float ld2 = a012 / l12;
									// 根据距离确定权重
									float s = 1.0f - 0.9f * std::fabs(ld2);
									
									coeff.x = s * la;
									coeff.y = s * lb;
									coeff.z = s * lc;
									coeff.intensity = s * ld2;

//									bool is_in_laser_fov = false;
//
//									// TODO 看明白是怎么判断的 相同的判断是否在视野范围内
//									PointType transform_pos;
//									transform_pos.x = transform_tobe_mapped_.pos.x();
//									transform_pos.y = transform_tobe_mapped_.pos.y();
//									transform_pos.z = transform_tobe_mapped_.pos.z();
//
//									float squared_side1 = mathutils::CalcSquaredDiff(transform_pos, pointSel);
//									float squared_side2 = mathutils::CalcSquaredDiff(pointOnZAxis, pointSel);
//
//									float check1 = 100.0f + squared_side1 - squared_side2
//									               - 10.0f * std::sqrt(3.0f) * std::sqrt(squared_side1);
//
//									float check2 = 100.0f + squared_side1 - squared_side2
//									               + 10.0f * std::sqrt(3.0f) * std::sqrt(squared_side1);
//
//									if (check1 < 0 && check2 > 0) { /// within +-60 degree
//										is_in_laser_fov = true;
//									}
									
									if (s > 0.1 ) {
										// 距离足够小才使用
										laserCloudOri->push_back(pointOri);
										coeffSel->push_back(coeff);
									}
								}
							}
						}
						// ========================开始处理平面点========================
						for (int i = 0; i < laserCloudSurfStackNum; i++) {
							pointOri = laser_cloud_surf_stack_downsampled_->points[i];
							pointAssociateToMap(pointOri, pointSel);
							kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSquareDis);
							
							if (pointSearchSquareDis[4] < 1.0) {
								// 构建五个最近点的坐标矩阵
								for (int j = 0; j < 5; j++) {
									matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
									matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
									matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
								}
								// 求解matA0*matX0=matB0
								matX0 = matA0.colPivHouseholderQr().solve(matB0);
								float pa = matX0(0, 0);
								float pb = matX0(1, 0);
								float pc = matX0(2, 0);
								float pd = 1;
								
								float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
								pa /= ps;
								pb /= ps;
								pc /= ps;
								pd /= ps;
								
								bool planeValid = true;
								for (int j = 0; j < 5; j++) {
									if (std::fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
									              pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
									              pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
										planeValid = false;
										break;
									}
								}
								
								if (planeValid) {
									float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
									
									float s = 1.0f - 0.9f * std::fabs(pd2) / std::sqrt(mathutils::CalcPointDistance(pointSel));
									
									
									coeff.x = s * pa;
									coeff.y = s * pb;
									coeff.z = s * pc;
									coeff.intensity = s * pd2;
									
//									bool is_in_laser_fov = false;
//									PointType transform_pos;
//									transform_pos.x = transform_tobe_mapped_.pos.x();
//									transform_pos.y = transform_tobe_mapped_.pos.y();
//									transform_pos.z = transform_tobe_mapped_.pos.z();
//									float squared_side1 = mathutils::CalcSquaredDiff(transform_pos, pointSel);
//									float squared_side2 = mathutils::CalcSquaredDiff(pointOnZAxis, pointSel);
//
//									float check1 = 100.0f + squared_side1 - squared_side2
//									               - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
//
//									float check2 = 100.0f + squared_side1 - squared_side2
//									               + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
//
//									if (check1 < 0 && check2 > 0) { /// within +-60 degree
//										is_in_laser_fov = true;
//									}
									
									if (s > 0.1) {
										laserCloudOri->push_back(pointOri);
										coeffSel->push_back(coeff);
									}
								}
							}
						}
						
						int laserCloudSelNum = laserCloudOri->points.size();
						if (laserCloudSelNum < 50) {
							//如果特征点太少
							continue;
						}
						Eigen::Matrix<float, Eigen::Dynamic, 6> mat_A(laserCloudSelNum, 6);
						Eigen::Matrix<float, 6, Eigen::Dynamic> mat_At(6, laserCloudSelNum);
						Eigen::Matrix<float, 6, 6> matAtA;
						Eigen::VectorXf mat_B(laserCloudSelNum);
						Eigen::VectorXf mat_AtB;
						Eigen::VectorXf mat_X;
						SO3 R_SO3(transform_tobe_mapped_.rot); /// SO3
						
						for (int i = 0; i < laserCloudSelNum; i++) {
							pointOri = laserCloudOri->points[i];
							coeff = coeffSel->points[i];
							
							Eigen::Vector3f p(pointOri.x, pointOri.y, pointOri.z);
							Eigen::Vector3f w(coeff.x, coeff.y, coeff.z);
							
							Eigen::Vector3f J_r = -w.transpose() * (transform_tobe_mapped_.rot * mathutils::SkewSymmetric(p));
							Eigen::Vector3f J_t = w.transpose();
							
							float d2 = coeff.intensity;
							
							mat_A(i, 0) = J_r.x();
							mat_A(i, 1) = J_r.y();
							mat_A(i, 2) = J_r.z();
							mat_A(i, 3) = J_t.x();
							mat_A(i, 4) = J_t.y();
							mat_A(i, 5) = J_t.z();
							mat_B(i, 0) = -d2;
						}
						mat_At = mat_A.transpose();
						matAtA = mat_At * mat_A;
						mat_AtB = mat_At * mat_B;
						mat_X = matAtA.colPivHouseholderQr().solve(mat_AtB);
						
						//退化场景判断与处理
						if (iterCount == 0) {
							Eigen::Matrix<float, 1, 6> mat_E;
							Eigen::Matrix<float, 6, 6> mat_V;
							Eigen::Matrix<float, 6, 6> mat_V2;
							
							Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6> > esolver(matAtA);
							mat_E = esolver.eigenvalues().real();
							mat_V = esolver.eigenvectors().real();
							
							mat_V2 = mat_V;
							
							isDegenerate = false;
							float eignThre[6] = {100, 100, 100, 100, 100, 100};
							for (int i = 0; i < 6; ++i) {
								if (mat_E(0, i) < eignThre[i]) {
									for (int j = 0; j < 6; j++) {
										mat_V2(i, j) = 0;
									}
									isDegenerate = true;
								} else {
									break;
								}
							}
							matP =  mat_V.inverse() * mat_V2;
						}
						
						if (isDegenerate) {
							Eigen::Matrix<float, 6, 1> matX2(mat_X);
							mat_X = matP * matX2;
						}
						Eigen::Vector3f r_so3 = R_SO3.log();

						r_so3.x() += mat_X(0, 0);
						r_so3.y() += mat_X(1, 0);
						r_so3.z() += mat_X(2, 0);
						transform_tobe_mapped_.pos.x() += mat_X(3, 0);
						transform_tobe_mapped_.pos.y() += mat_X(4, 0);
						transform_tobe_mapped_.pos.z() += mat_X(5, 0);
//						std::cout << mat_X(0, 0) << "   " << mat_X(1, 0) << "   " << mat_X(2, 0) << "   "<< mat_X(3, 0) << "   "<< mat_X(4, 0) << "   "<< mat_X(5, 0) << "   \n";
//						if (!std::isfinite(r_so3.x())) r_so3.x() = 0;
//						if (!std::isfinite(r_so3.y())) r_so3.y() = 0;
//						if (!std::isfinite(r_so3.z())) r_so3.z() = 0;

//						SO3 tobe_mapped_SO3 = SO3::exp(r_so3);
//						    transform_tobe_mapped_.rot = tobe_mapped_SO3.unit_quaternion().normalized();
						
						transform_tobe_mapped_.rot =
								transform_tobe_mapped_.rot * mathutils::DeltaQ(Eigen::Vector3f(mat_X(0, 0), mat_X(1, 0), mat_X(2, 0)));
						
						if (!std::isfinite(transform_tobe_mapped_.pos.x())) transform_tobe_mapped_.pos.x() = 0.0;
						if (!std::isfinite(transform_tobe_mapped_.pos.y())) transform_tobe_mapped_.pos.y() = 0.0;
						if (!std::isfinite(transform_tobe_mapped_.pos.z())) transform_tobe_mapped_.pos.z() = 0.0;
						
						float delta_r = rad2deg(R_SO3.unit_quaternion().angularDistance(transform_tobe_mapped_.rot));
						float delta_t = std::sqrt(float(std::pow(mat_X(3, 0) * 100.0f, 2)) +
						                          float(std::pow(mat_X(4, 0) * 100.0f, 2)) +
						                          float(std::pow(mat_X(5, 0) * 100.0f, 2)));
						// 旋转平移量足够小就停止迭代
						if (delta_r < 0.05 && delta_t < 0.05) {
//							cout << iterCount << endl;
							break;
						}
					}
					//迭代结束更新相关的转移矩阵
					transformUpdate();
//					std::cout << transform_tobe_mapped_.rot.x() << "     " << transform_tobe_mapped_.rot.y() << "     " << transform_tobe_mapped_.rot.z() << "     " << transform_tobe_mapped_.rot.w() << "     " << transform_tobe_mapped_.pos << std::endl;
				}
				
				// TODO
				// 将corner points按距离（比例尺缩小）归入相应的立方体
				for (int i = 0; i < laserCloudCornerStackNum; i++) {
					// 转移到世界坐标系
					pointAssociateToMap(laser_cloud_corner_stack_downsampled_->points[i], pointSel);
					//按50的比例尺缩小，四舍五入，偏移laserCloudCen*的量，计算索引
					int cubeI = int((pointSel.x + 25.0) / 50.0) + laser_cloud_cen_length_;
					int cubeJ = int((pointSel.y + 25.0) / 50.0) + laser_cloud_cen_width_;
					int cubeK = int((pointSel.z + 25.0) / 50.0) + laser_cloud_cen_height_;
					if (pointSel.x + 25.0 < 0) --cubeI;
					if (pointSel.y + 25.0 < 0) --cubeJ;
					if (pointSel.z + 25.0 < 0) --cubeK;
					if (cubeI >= 0 && cubeI < laserCloudLength &&
					    cubeJ >= 0 && cubeJ < laserCloudWidth &&
					    cubeK >= 0 && cubeK < laserCloudHeight) {
						
						//只挑选-laserCloudCenWidth * 50.0 < point.x < laserCloudCenWidth * 50.0范围内的点，y和z同理
						//按照尺度放进不同的组，每个组的点数量各异
						int cubeInd = ToIndex(cubeI, cubeJ, cubeK);
						laserCloudCornerArray[cubeInd]->push_back(pointSel);
					}
				}
				// 将surf points按距离（比例尺缩小）归入相应的立方体
				for (int i = 0; i < laserCloudSurfStackNum; i++) {
					pointAssociateToMap(laser_cloud_surf_stack_downsampled_->points[i], pointSel);
					
					int cubeI = int((pointSel.x + 25.0) / 50.0) + laser_cloud_cen_length_;
					int cubeJ = int((pointSel.y + 25.0) / 50.0) + laser_cloud_cen_width_;
					int cubeK = int((pointSel.z + 25.0) / 50.0) + laser_cloud_cen_height_;
					
					if (pointSel.x + 25.0 < 0) --cubeI;
					if (pointSel.y + 25.0 < 0) --cubeJ;
					if (pointSel.z + 25.0 < 0) --cubeK;
					
					if (cubeI >= 0 && cubeI < laserCloudLength &&
					    cubeJ >= 0 && cubeJ < laserCloudWidth &&
					    cubeK >= 0 && cubeK < laserCloudHeight) {
						int cubeInd = ToIndex(cubeI, cubeJ, cubeK);
						laserCloudSurfArray[cubeInd]->push_back(pointSel);
					}
				}
				
				// 特征点下采样
				// laserCloudCornerArray_downSampled就是当做一个过渡容器
				for (int i = 0; i < laserCloudValidNum; i++) {
					int ind = laserCloudValidInd[i];
					
					int last_i, last_j, last_k;
					FromIndex(ind, last_i, last_j, last_k);
					
					float center_x = 50.0f * float(last_i - laser_cloud_cen_length_);
					float center_y = 50.0f * float(last_j - laser_cloud_cen_width_);
					float center_z = 50.0f * float(last_k - laser_cloud_cen_height_); // NOTE: center of the margin cube
					
					int cube_i = int((center_x + 25.0) / 50.0) + laser_cloud_cen_length_;
					int cube_j = int((center_y + 25.0) / 50.0) + laser_cloud_cen_width_;
					int cube_k = int((center_z + 25.0) / 50.0) + laser_cloud_cen_height_;
					
					if (center_x + 25.0 < 0) --cube_i;
					if (center_y + 25.0 < 0) --cube_j;
					if (center_z + 25.0 < 0) --cube_k;
					
					if (cube_i >= 0 && cube_i < laserCloudLength &&
					    cube_j >= 0 && cube_j < laserCloudWidth &&
					    cube_k >= 0 && cube_k < laserCloudHeight) {
						
						ind = ToIndex(cube_i, cube_j, cube_k); // NOTE: update to current index
						
						//      DLOG(INFO) << "index after: " << index;
						
						laserCloudCornerArray_downSampled[ind]->clear();
						downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
						downSizeFilterCorner.filter(*laserCloudCornerArray_downSampled[ind]);
						
						laserCloudSurfArray_downSampled[ind]->clear();
						downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
						downSizeFilterSurf.filter(*laserCloudSurfArray_downSampled[ind]);
						
						// swap cube clouds for next processing
						laserCloudCornerArray[ind].swap(laserCloudCornerArray_downSampled[ind]);
						laserCloudSurfArray[ind].swap(laserCloudSurfArray_downSampled[ind]);
					}
//					laserCloudCornerArray_downSampled[ind]->clear();
//					downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
//					downSizeFilterCorner.filter(*laserCloudCornerArray_downSampled[ind]);//滤波输出到Array2
//
//					laserCloudSurfArray_downSampled[ind]->clear();
//					downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
//					downSizeFilterSurf.filter(*laserCloudSurfArray_downSampled[ind]);
//
//					// Array与Array2交换，即滤波后自我更新
//					laserCloudCornerArray[ind].swap(laserCloudCornerArray_downSampled[ind]);
//					laserCloudSurfArray[ind].swap(laserCloudSurfArray_downSampled[ind]);
				}
				
				mapFrameCount++;
				
				std_msgs::Header temp_Header;
				temp_Header.stamp = ros::Time().fromSec(timeLaserOdometry);
				temp_Header.frame_id = "/camera_init";
				
				// 特征点汇总下采样，每隔五帧publish一次，从第一次开始
				if (mapFrameCount >= mapFrameNum) {
					mapFrameCount = 0;
					
					laserCloudSurround2->clear();
					for (int i = 0; i < laserCloudSurroundNum; i++) {
						int ind = laserCloudSurroundInd[i];
						*laserCloudSurround2 += *laserCloudCornerArray[ind];
						*laserCloudSurround2 += *laserCloudSurfArray[ind];
					}
					laserCloudSurround->clear();
					downSizeFilterCorner.setInputCloud(laserCloudSurround2);
					downSizeFilterCorner.filter(*laserCloudSurround);
					
					ros_comman::PublishCloudMsg(pubLaserCloudSurround, *laserCloudSurround, temp_Header);
				}
				
				// 将点云中全部点转移到世界坐标系下
				int laserCloudFullResNum = laserCloudFullRes->points.size();
				for (int i = 0; i < laserCloudFullResNum; i++) {
					pointAssociateToMap(laserCloudFullRes->points[i], laserCloudFullRes->points[i]);
				}
				
				ros_comman::PublishCloudMsg(pubLaserCloudFullRes, *laserCloudFullRes, temp_Header);
				
				geometry_msgs::Quaternion geoQuat;
				geoQuat.w = transform_aft_mapped_.rot.w();
				geoQuat.x = transform_aft_mapped_.rot.x();
				geoQuat.y = transform_aft_mapped_.rot.y();
				geoQuat.z = transform_aft_mapped_.rot.z();
				
				odomAftMapped.header.stamp = temp_Header.stamp;
				odomAftMapped.pose.pose.orientation.x = geoQuat.x;
				odomAftMapped.pose.pose.orientation.y = geoQuat.y;
				odomAftMapped.pose.pose.orientation.z = geoQuat.z;
				odomAftMapped.pose.pose.orientation.w = geoQuat.w;
				odomAftMapped.pose.pose.position.x = transform_aft_mapped_.pos.x();
				odomAftMapped.pose.pose.position.y = transform_aft_mapped_.pos.y();
				odomAftMapped.pose.pose.position.z = transform_aft_mapped_.pos.z();
				pubOdomAftMapped.publish(odomAftMapped);
				
				// 广播坐标系旋转平移参量
				aftMappedTrans.stamp_ = temp_Header.stamp;
				aftMappedTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
				aftMappedTrans.setOrigin(tf::Vector3(transform_aft_mapped_.pos.x(),
				                                     transform_aft_mapped_.pos.y(),
				                                     transform_aft_mapped_.pos.z()));
				tfBroadcaster.sendTransform(aftMappedTrans);
			}
		}
		status = ros::ok();
		rate.sleep();
	}
	return 0;
}
