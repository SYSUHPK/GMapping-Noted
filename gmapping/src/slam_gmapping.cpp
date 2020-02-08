/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */


/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [int] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [int] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)
- @b "~/minimumScore" @b [double] minimum score for considering the outcome of the scanmatching good. Can avoid 'jumping' pose estimates in large open spaces when using laser scanners with limited range (e.g. 5m). (0 = default. Scores go up to 600+, try 50 for example when experiencing 'jumping' estimate issues)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular step size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/



#include "slam_gmapping.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

/*默认构造函数*/
//初始化map 到 odom的transform
//雷达计数器
//调用系统时间随机初始化种子，是一种常用的方法
//然后调用init完成初始化
SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}
/*构造函数*/
SlamGMapping::SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0),node_(nh), private_nh_(pnh), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
  seed_ = time(NULL);
  init();
}
/*构造函数*/
SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
  seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
  init();
}

/*SLAMGMapping初始化*/
void SlamGMapping::init()
{
  // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The library is pretty chatty
  //gsp_ = new GMapping::GridSlamProcessor(std::cerr);
  //创建建图引擎对象，每个粒子都有一个关于地图和机器人位姿的估计
  //gsp_其实就是基本的GridFastSLAM算法的类对象， 也就是openslam里面的源码，用RBPF完成SLAM工作，
  gsp_ = new GMapping::GridSlamProcessor();
  ROS_ASSERT(gsp_);
  //创建broadcaster对象tf_B，用于发布从地图到里程计的坐标变换map_to_odom_
  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);
  //激光传感器和里程计传感器
  //这里只是设置为null，后面在startLiveSlam中会新建一个对象
  gsp_laser_ = NULL;
  gsp_odom_ = NULL;
  //第一次扫描和建图初始化
  //是否接收到了第一次的扫描数据
  got_first_scan_ = false;
  //是否生成了地图
  got_map_ = false;
  

  /*初始化参数*/
  // Parameters used by our GMapping wrapper
  //利用私有句柄private_nh_从参数服务器上请求，没有就设置默认值
  //私有句柄可以通过getParam得到参数，因为launch文件中的node_name和私有句柄是一样的
  //处理的容忍度，接受到throttle_scans个scan，就进行更新
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  //基座坐标系
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  //地图坐标系
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  //里程计坐标系
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  //发布间隔
  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  //地图更新间隔
  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  //maxRange：传感器的最大测量距离，一般是第一次scan-1cm。maxUrange=maxRange
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  //评分的最小值，衡量scan数据好坏
  if(!private_nh_.getParam("minimumScore", minimum_score_))
    minimum_score_ = 0;
  //扫描匹配中cell（计算占用概率）的标准差
  if(!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  //扫描匹配中搜索窗口的大小
  if(!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  //扫描匹配中的初始距离步长
  if(!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  //扫描匹配中的初始角度
  if(!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  //扫描匹配中的迭代次数
  if(!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  //扫描匹配中单个激光扫描束的标准差
  if(!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  //似然度平滑的增益
  if(!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  //lskip次扫描之后进行一次匹配
  if(!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  //位置的噪声
  if(!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  //方位角的噪声
  if(!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  //位置到方位角的协方差
  if(!private_nh_.getParam("str", str_))
    str_ = 0.1;
  //方位角到位置的协方差
  if(!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  //机器人运行linearUpdate之后才进行一次测量
  if(!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  //机器人转动了angularUpdate之后才进行一次测量
  if(!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  //重采样的阈值
  if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  //粒子数目
  if(!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  /*地图的最小/大x/y值*/
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  //地图中一个像素的尺度
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  //占用概率阈值
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  //距离采样范围
  if(!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  //距离采样步长
  if(!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  //角度采样范围
  if(!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  //角度采样步长
  if(!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;
  //转换的间隔
  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;
  /*虽然init得到了参数和gsp_，但是没有在gsp_上生效，所以后面调用函数initMapper完成建图引擎的配置
  这个操作会在startLiveSlam中调用lasercallback中被调用*/
}


/*订阅一些主题 发布一些主题*/
void SlamGMapping::startLiveSlam()
{
  /*私有句柄发布“~entropy” topic，全局命名空间发布/slam_gmapping/map和/slam_gmapping/map_metadata*/
  //三个都是通过句柄发布一些数据，topic/queue/store the last for next one/
  //entropy:机器人姿态分布的熵的估计值（较高的值表示较大的不确定性）
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

  /*注册服务"dynamic_map"，以及相关的回调函数mapCallback*/
  //service name/callback function/调用callback的对象，this其实就是类方法作为回调函数，也就是main中的gn
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  

  /*订阅激光数据
  三个做的其实就是tf下的消息过滤器，也就是订阅，缓存，转换坐标系，进行处理（回调函数中）
  先用对象scan_filter_sub_订阅了主题"scan"，以监听激光传感器的扫描值。然后用该订阅器构建消息过滤器， 
  让它同时监听激光扫描消息和里程计坐标变换，只有转换数据存在才会调用。最后注册回调函数SlamGMapping::laserCallback，
  每当传感器的数据可以转换到目标坐标系上时，就调用该函数完成地图的更新。
  */
  //句柄/topic/队列长度，namespace/class template
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  //这里用到subscribe函数，TransformListener，目标坐标系，queue
  //保证激光数据可以转换到odom坐标系上，也就是说在这之前应该是tf tree里面有这个变换了
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);  
  //用到类方法作为回调函数
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));
  
  /*创建一个线程，以transform_publish_period_为周期发布坐标变换,map to odom*/
  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period_));
  
}

/*播放包的slam，与startLiveSlam类似*/
void SlamGMapping::startReplay(const std::string & bag_fname, std::string scan_topic)
{
  /*publisher*/
  double transform_publish_period;
  ros::NodeHandle private_nh_("~");
  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  
  /*打开数据集*/
  rosbag::Bag bag;
  bag.open(bag_fname, rosbag::bagmode::Read);
  
  /*指定要读的topics*/
  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(scan_topic);
  /*读指定的topics*/
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  /*存储数据*/
  // Store up to 5 messages and there error message (if they cannot be processed right away)
  std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
  //按照时间顺序遍历每一个topic
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if (cur_tf != NULL) {
      for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
      {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        //将 TransformStamped msg 转换到 tf::StampedTransform
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        //添加一个新的 transform 到 Transformer graph
        tf_.setTransform(stampedTf);
      }
    }

    /*对laserscan的数据也是一样的处理*/
    sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
    if (s != NULL) {
      if (!(ros::Time(s->header.stamp)).is_zero())
      {
        s_queue.push(std::make_pair(s, ""));
      }
      //处理最近的五个scan
      // Just like in live processing, only process the latest 5 scans
      if (s_queue.size() > 5) {
        ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
        s_queue.pop();
      }
      // ignoring un-timestamped tf data 
    }

    /*队列不空，调用laserCallback进行处理*/
    // Only process a scan if it has tf data
    while (!s_queue.empty())
    {
      try
      {
        tf::StampedTransform t;
        tf_.lookupTransform(s_queue.front().first->header.frame_id, odom_frame_, s_queue.front().first->header.stamp, t);
        this->laserCallback(s_queue.front().first);
        s_queue.pop();
      }
      // If tf does not have the data yet
      catch(tf2::TransformException& e)
      {
        // Store the error to display it if we cannot process the data after some time
        s_queue.front().second = std::string(e.what());
        break;
      }
    }
  }

  bag.close();
}
/*不停地发布坐标转换，map to odom*/
//map to odom已经在laserCallback中计算出来了，所以这里不停地发布
void SlamGMapping::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}
/*析构函数*/
SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

/*t时刻激光中心位置在里程计坐标下的位姿,存储到gmap_pose，这里的里程计不是base，但是是会动的那一点，也就是机器人的起始点*/
bool
SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  
  // Get the pose of the centered laser at the right time
  //激光中心位置的时间，在initMapper里面已经初始化过了centered_laser_pose_，求出来了
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  //得到激光中心位置在里程计坐标位姿，tf::<T>继承T，并增加了2个字段stamp_ frame_id_，其实加上Transform就和input、output类型一致
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    //目标坐标系；input（geometry_msgs::PoseStamped[有stamped、fram_id,point+Quaternion(其实和tf::Stamped<tf::Transform>一样)]）: 本来坐标系下的位姿，output: 目标坐标系下的位姿
    tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  //获得OrientdPoint的x，y和yaw/theta
  double yaw = tf::getYaw(odom_pose.getRotation());

  gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

/*初始化地图*/
bool
SlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
  /*计算激光相对于基座的位姿*/
  //激光传感器的坐标系名称
  laser_frame_ = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  //tf::Stamped<tf::Pose>：需要stamped,frame_id,pose(平移+旋转)；tf::Stamped<T>
  tf::Stamped<tf::Pose> ident;
  //旋转+平移矩阵，stamped,frame_id，等价于上面的定义
  tf::Stamped<tf::Transform> laser_pose;
  //ident存储激光的初始原点，函数将全部设置为0，自己的坐标，肯定是0
  ident.setIdentity();
  //额外的信息
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;
  try
  {
    //转换前的存储在ident中，隐含自己的坐标系，转换的位姿存储在laser_pose，此时包含目标坐标系，是激光原点在目标坐标系下的位姿。tf_是listener，假设tf tree上已经有转换的信息，这个实际机器人应该会发布
    //其实就是矩阵操作
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  /*在机器人坐标系下，在激光传感器上方1m的点up，转换到激光坐标下的位置，用于判断激光的安装方式*/
  // create a point 1m above the laser position and transform it into the laser-frame
  //转来转去就是为了进行判断，所以不可以直接在激光传感器坐标下取点，这里激光传感器坐标应该是根据某种规则事先定好的，所以才有可能有负的情况
  //用了上面转化的laser_pose，所以是在基座base_frame坐标下
  //Vector3 = point(x,y,z) 
  tf::Vector3 v;
  //这个v，其实现在坐标是base_link上的（0，0，1+z），如果laser_frame和base_frame处于同一个x，y上，那就是一样的，不是也是可以判断的
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  //构造函数：Vector3,stamped,frame_id
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                      base_frame_);
  try
  {
    //将上面的点，转换为激光坐标下的位置，point=vector3
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  
  /*判断激光有没有水平安装*/
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
    return false;
  }
  //激光数据数组（数据量）的size
  gsp_laser_beam_count_ = scan.ranges.size();
  //激光的中心位置
  double angle_center = (scan.angle_min + scan.angle_max)/2;
  
  /*判断激光有没有上下颠倒，调整*/
  //激光正常安装，判断扫描方向
  if (up.z() > 0)
  {
    //激光有没有顺序相反，可能默认会错，所以还需要判断一下
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    //激光开始的位置应该是最左边，然后应该加上angle_center，才是中间的点在laser_frame_id坐标系下的坐标，RPY：欧拉角
    //getOdompose会调用
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  //激光安装上下颠倒，判断扫描方向
  else
  {
    //判断有没有激光顺序相反
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    //默认左边为起始线，翻转之后在右边，所以是负的
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  /*激光的角度按相同间隔从小到大存储在laser_angles_中，
  根据扫描的数据长度和增长步长初始化雷达的扫描角度表laser_angles_*/
  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  //保证角度的数组和激光数据数组一致
  laser_angles_.resize(scan.ranges.size());
  //对称，一开始为负，注意angle和range的区别
  // Make sure angles are started so that they are centered
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }
  //没有负的
  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  //有负的 
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));
  //orientedpoint的类型：(x,y,theta)，并不是3*3的旋转矩阵，表示激光中心在laser_frame的位姿
  //实际上就是里程计的位姿
  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  //它提供简单的启动和关闭roscpp程序内部节点
  //这个是子空间的意思，它提供了额外的命名空间解析层，可以使编写子组件更容易。
  //这里其实就是私有命名空间，格式:/node name/
  //maxRange就是maxUrange，range是指激光数值
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

   /*初始化激光传感器模型对象RangeSensor，gsp_laser_*/
  // The laser must be called "FLASER".
  // We pass in the absolute value of the computed angle increment, on the
  // assumption that GMapping requires a positive angle increment.  If the
  // actual increment is negative, we'll swap the order of ranges before
  // feeding each scan to GMapping.
  //类对象gsp_laser，描述扫描光束的各种物理性质
  //https://github.com/ros-perception/openslam_gmapping/blob/master/include/gmapping/sensor/sensor_range/rangesensor.h
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         gsp_laser_beam_count_,          //激光数据数组的size
                                         fabs(scan.angle_increment),      
                                         gmap_pose,                       
                                         0.0,
                                         maxRange_);
  ROS_ASSERT(gsp_laser_);
  /*gsp_的成员函数smap，就是传感器映射，将名字与传感器map*/
  GMapping::SensorMap smap;
  //getName就是"FLASER"
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  //gsp_是GridSlamProcessor类对象，建图引擎，调用setSensorMap配置激光传感器，后面就可以通过map容器获取激光传感器
  gsp_->setSensorMap(smap);

  /*里程计模型对象，似乎没有用到*/
  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);

  /*getOdommPose设置激光初始位姿，其实就是将激光的中心位姿转换为odom坐标
  不是里程计位姿的初始化，而是传感器在里程计坐标下的位姿，其实是机器人的初始位姿，在processScan里面会出现*/
  /// @todo Expose setting an initial pose
  //initialPose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    //不然激光的中心位置就定义为和odom坐标原点重合，是在odom坐标下
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
  }
  /*设置建图引擎的参数，其实就是参数服务器的一种设置方法*/
  //匹配模型参数
  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);
  //运动模型参数数据
  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  //更新距离数据
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  //更新周期
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  //采样范围
  gsp_->setllsamplerange(llsamplerange_);
  //采样步骤
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

  // Call the sampling function once to set the seed.
  //采样一次函数，随机种子
  GMapping::sampleGaussian(1,seed_);

  ROS_INFO("Initialization complete");

  return true;
}

/*收集距离扫描数据和里程计数据，供建图引擎更新粒子集合*/
//gmap_pose其实是输出，里程计的累积位姿，
bool
SlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
  /*输入参数进行一些检查，防止程序发生意外的状况*/
  //激光的中心坐标是否可以转换为里程计坐标，应该只是每次都测试一下原点，看看可不可以转换，gmap_pose是输出
  //之前initMapper里面是初始化，而且是原点之间，这里是判断可不可以转换，因为后面要进行转换
  if(!getOdomPose(gmap_pose, scan.header.stamp))
     return false;
  //激光的数据size是否相等
  if(scan.ranges.size() != gsp_laser_beam_count_)
    return false;

  /*GMapping需要一个double型的数组来接收扫描数据*/
  // GMapping wants an array of doubles...
  //新建一个数组并把输入参数scan中的数据拷贝进去
  double* ranges_double = new double[scan.ranges.size()];
  
  /*保证扫描角度从小到大，同时去掉数据的异常值*/
  // If the angle increment is negative, we have to invert the order of the readings.
  //在初始化过程中已经根据扫描数据判定扫描方向， 并用成员函数do_reverse_range_标记，不是就翻转
  if (do_reverse_range_)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    /*顺序转换，若出现不合理值(噪声),则置为最大值，这里指的应该是数据*/
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  }
  /*不是颠倒也需要遍历*/ 
  else 
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  /*使用GMapping中的标准扫描读数对象reading，继承vector*/
  GMapping::RangeReading reading(scan.ranges.size(),
                                //扫描的数据
                                 ranges_double,
                                 //reading需要激光传感器对象
                                 gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  //会对数组ranges_double进行深度拷贝，可以释放内存
  delete[] ranges_double;
  //添加里程计数据到reading
  //gmap_pose是激光中心位置，其在odom坐标下的位置是会变化的，根据getOdomPose可以获得
  reading.setPose(gmap_pose);

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");
  //调用建图引擎gsp_的函数processScan更新粒子集合
  return gsp_->processScan(reading);
}

/*传感器的数据转换到对应坐标系后，回调函数对数据进行处理，更新地图
1.把激光传感器数据和里程计数据提供给建图引擎gsp_，让其更新粒子集，这一过程在addScan中实现
2.根据最优粒子的地图数据更新最终的占用栅格地图，这一过程由updateMap实现
*/
void
SlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  //对激光传感器的数据进行计数,每当接收到throttle_scans个扫描数据就进行一次更新操作，但是不会缓存数据，相当于限流器
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;
  //构建一个静态的ros::Time对象，用于记录最近一次的更新时间，全局
  static ros::Time last_map_update(0,0);

  /*在满足一定条件下，对地图进行初始化*/
  // We can't initialize the mapper until we've got the first scan
  //got_first_scan一开始只是声明，然后init()函数会被调用，设置为false
  if(!got_first_scan_)
  {
    //只有满足initMapper的条件，才可以对数据进行处理，才记为得到了first_scan，之后就不会再调用
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }
  /*里程计位姿：x，y，theta*/
  GMapping::OrientedPoint odom_pose;
  
  /*如果可以添加数据，进一步的计算地图到里程计的坐标变换并更新地图*/
  if(addScan(*scan, odom_pose))
  {
    ROS_DEBUG("scan processed");
    /*以最优粒子地图坐标为基准计算从激光雷达到地图之间的坐标变换,同时计算从里程计到激光雷达的坐标变换*/
    //从粒子集合中挑选最优的粒子，mpose是激光在地图坐标下的位置，odom_pose是激光在odom下的位姿
    GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);
    //变换是已经存在了的
    //激光雷达到地图之间的坐标变换，其实就是base_link to map，其实就是当前的激光原点位姿（x，y，theta）（在地图坐标上）以transform的形式表示出来， 是坐标变换，但是是已经存在那种，不是transformpose那种
    //这是粒子滤波测出来的，是绝对的
    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    //从里程计到激光雷达的坐标变换, 其实就是odom to base_link
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));
    
    /*原子操作计算地图到里程计，坐标转换一般都这么计算*/
    //在更新的时候对map_to_odom_加锁了，以保证更新过程是原子的，后面 map_to_odom 会被publish出去
    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    map_to_odom_mutex_.unlock();

    /*更新地图*/
    //1.没有地图，直接更新
    //2.根据上面参数map_update_interval_的设定，定时更新地图
    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      updateMap(*scan);
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  } else
    ROS_DEBUG("cannot process scan");
}

/*遍历粒子集合计算熵*/
double
SlamGMapping::computePoseEntropy()
{
  //计算粒子的总权重
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  //计算熵
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}
/*根据最优粒子的地图数据更新最终栅格地图中各个单元的占用概率*/
void
SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  //加锁
  boost::mutex::scoped_lock map_lock (map_mutex_);
  /*构建扫描器，配置参数，对建议分布进行采样时，考虑激光传感器的读数，提高准确度， 减少粒子数量*/
  GMapping::ScanMatcher matcher;
  //激光参数
  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());
  //范围参数:最大范围和可用范围
  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  //产生地图true
  matcher.setgenerateMap(true);
  
  //利用建图引擎，获取最优的粒子
  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  
  /*计算熵，通过发布器发布，应该只是用来订阅*/
  std_msgs::Float64 entropy;
  //computePoseEntropy可以遍历粒子集合计算熵
  entropy.data = computePoseEntropy();
  //>0则用startLiveSlam中定义的publisher发布
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);
  /*通过变量got_map_判断地图是否构建过，没有就对参数进行初始化*/
  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 
  /*根据地图中心创造一个扫描匹配地图对象ScanMatcherMap*/
  GMapping::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);

  /*遍历最优粒子的轨迹树，计算激活区域，更新地图*/
  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  /*地图可能会随着探索区域增加而扩张，需要resize*/
  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  /*根据占用概率阈值将二维地图进行划分*/
  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      //调用cell计算占用概率值
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      //未知区域为-1
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      //占用区域为100
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      //空闲区域为0
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  //地图已经构建过
  got_map_ = true;

  /*地图消息打上时间戳*/
  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );
 
  /*发布地图的topic信息*/
  //用之前定义的publisher发布占用地图
  sst_.publish(map_.map);
  //用之前定义的publisher发布地图位置信息
  sstm_.publish(map_.map.info);
}
/*前面的服务的回调函数，可以获取实时地图*/
bool 
SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  //加锁
  boost::mutex::scoped_lock map_lock (map_mutex_);
  //地图已经存在，给予响应
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}
/*发布从map坐标系到odom坐标系的变换*/
void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}
