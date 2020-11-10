/**
 * @file odometry.h
 * @brief ANIのエンコーダからodometryを取得するためのクラス
 * @author Yusuke Yoshizaki
 * @date 2019/10/10
 * @detail
 */

#ifndef ODOMETRY_SRC_ODOMETRY_H_
#define ODOMETRY_SRC_ODOMETRY_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

//クラスの宣言
class Odometry{
  public:
    Odometry();
    ~Odometry();

  private:
    // CallBack関数
    void TimerCallback(const ros::TimerEvent& event);
    void LeftEncCallBack(const std_msgs::Int32 l_enc_count);
    void RightEncCallBack(const std_msgs::Int32 r_enc_count);

    //位置
    double x_;
    double y_;
    double th_;

    //速度
    double vx_;
    double vy_;
    double vth_;

    // Encoder
    double l_pulse_;
    double r_pulse_;

    // Odometry関係
    nav_msgs::Odometry odom_;
    ros::Time current_time_; // 現在の時間
    ros::Time last_time_;    // 一つ前の時間
    geometry_msgs::TransformStamped odom_trans_; // Odometry用のtf

    // Timer
    ros::Timer timer_;

    // 制御周期
    double sampling_time_;

    // NodeHandle
    ros::NodeHandle nh_;

    // Subscriber
    ros::Subscriber l_enc_sub_;
    ros::Subscriber r_enc_sub_;

    // Publisher
    ros::Publisher odom_pub_;

    // tf
    tf::TransformBroadcaster odom_broad_;
 };

 //条件コンパイル
 #endif /* ODOMETRY_SRC_ODOMETRY_H_ */
