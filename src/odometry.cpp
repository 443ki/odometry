/**
 * @file odometry.cpp
 * @brief ANIのエンコーダからオドメトリを取得するためのプログラム
 * @author Yusuke Yoshizaki
 * @date 2019/10/10
 * @detail
 */

#include "odometry.h"

// ANIの寸法、設定値
#define CRAWLER_HEIGHT 0.195 // クローラの高さ[m]
#define NUMBER_OF_PULSE 1024 // 1回転あたりのエンコーダのパルス数
#define DPP (CRAWLER_HEIGHT/2)*(2*M_PI/NUMBER_OF_PULSE) // 1パルスで進む距離[m]
#define TREAD 0.31 // トレッド長[m]

//===== constructor & destrucor ==================================================//
/** @fn
 * @brief コンストラクタ
 * @param なし
 */
Odometry::Odometry(){

  // メンバ変数の初期化
  x_ = 0.0;
  y_ = 0.0;
  th_ = 0.0;

  vx_ = 0.0;
  vy_ = 0.0;
  vth_ = 0.0;

  l_pulse_ = 0.0;
  r_pulse_ = 0.0;

  // 制御周期[s]
  sampling_time_ = 0.1;    //10Hz

  // 時間
  current_time_ = ros::Time::now();
  //last_time_ = ros::Time::now();

  // Odometry
  // 親フレームに/mapを子フレームに/daniel_baseを指定
  odom_trans_.header.frame_id = "odom";
  odom_trans_.child_frame_id = "daniel_base";
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "daniel_base";

  // Timer
  timer_ = nh_.createTimer(ros::Duration(sampling_time_), &Odometry::TimerCallback,this);

  // Subscriber
  l_enc_sub_ = nh_.subscribe("/l_encoder", 1, &Odometry::LeftEncCallBack,this);
  r_enc_sub_ = nh_.subscribe("/r_encoder", 1, &Odometry::RightEncCallBack,this);

  // Publisher
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom" ,50);
}

/** @fn
 * @brief デストラクタ
 * @param なし
 */
Odometry::~Odometry()
{
}

//===== Callback ==================================================//
/** @fn
 * @brief タイマーコールバック
 * @param const ros::TimerEvent& event
 * @return なし
 * @detail
 *  サンプリングタイム毎にtfとオドメトリを計算して送信
 */
void Odometry::TimerCallback(const ros::TimerEvent& event) {
  // 現在時刻を求める
  current_time_ = ros::Time::now();

  //エンコーダからオドメトリの計算
  //double dt = (current_time_ - last_time_).toSec();
  double dt = sampling_time_;
  double vr = r_pulse_*DPP/dt;
  double vl = l_pulse_*DPP/dt;
  double v = (vr+vl) / 2.0;
  vth_ = (vr-vl) / (TREAD);
  vx_ = v * cos(th_);
  vy_ = v * sin(th_);
  double delta_x = vx_ * dt;
  double delta_y = vy_ * dt;
  double delta_th = vth_ * dt;

  ROS_INFO("dt:%f", dt);
  ROS_INFO("x_:%f", x_);


  // パルス値の初期化
  r_pulse_ = 0;
  l_pulse_ = 0;

  // 位置、姿勢の更新
  x_ += delta_x;
  y_ += delta_y;
  th_ += delta_th;

  //ヨー角からクォータニオンへ変換する
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

  // tfの更新
  odom_trans_.header.stamp = current_time_;

  odom_trans_.transform.translation.x = x_;
  odom_trans_.transform.translation.y = y_;
  odom_trans_.transform.translation.z = 0.0;
  odom_trans_.transform.rotation = odom_quat;

  // tfを発行
  odom_broad_.sendTransform(odom_trans_);

  //オドメトリへ代入
  odom_.header.stamp = current_time_;

  odom_.pose.pose.position.x = x_;
  odom_.pose.pose.position.y = y_;
  odom_.pose.pose.position.z = 0.0;
  odom_.pose.pose.orientation = odom_quat;

  odom_.twist.twist.linear.x = vx_;
  odom_.twist.twist.linear.y = vy_;
  odom_.twist.twist.angular.z = vth_;

  // オドメトリをpublish
  odom_pub_.publish(odom_);

  // 時刻の更新
  //last_time_ = current_time_;
}

/** @fn
 * @brief エンコーダのコールバック関数
 * @param const std_msgs::Int32 l_enc_count
 * @return なし
 * @detail
 *  左右それぞれのエンコーダの値を累積させる
 */
void Odometry::LeftEncCallBack(const std_msgs::Int32 l_enc_count){
  l_pulse_ += l_enc_count.data;
}

void Odometry::RightEncCallBack(const std_msgs::Int32 r_enc_count){
  r_pulse_ += r_enc_count.data;
}
