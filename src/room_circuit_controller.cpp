#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

geometry_msgs::Twist cmd_vel;
ros::Publisher pub;
ros::Subscriber sub;

// variables
float velocity = 0.2;
float angular_velocity = 1.57;
float avoid_distance = 1.0;
float avoid_angle = 60;

void pubMsg(float vel, float ang_vel)
{
    cmd_vel.linear.x = vel;
    cmd_vel.angular.z = ang_vel;
    pub.publish(cmd_vel); // publish
}

void scanCallback(sensor_msgs::LaserScan msg)
{
    int center_index = msg.ranges.size() / 2;             // ロボット前方方向を表すインデックス
    int index_avoid_angle = avoid_angle * (M_PI / 180.0) / msg.angle_increment; // 回避させたい角度に相当するインデックス
    int start_index = center_index - index_avoid_angle / 2;
    int last_index = center_index + index_avoid_angle / 2;

    bool detect_obstacle = false;
    std::string detect_side;

    for (int i = start_index; i <= last_index; i++)
    {
        float range = msg.ranges[i]; // 距離

        // 値が無効な場合の排除
        if (range < msg.range_min || // エラー値の場合
            range > msg.range_max || // 測定範囲外の場合
            std::isnan(range))       // 無限遠の場合
            ;

        // 値が有効である場合
        else
        {
            if (range < avoid_distance) // 近くに障害物がある場合
            {
                detect_obstacle = true;
                if (i <= center_index)
                {
                    detect_side = "right";
                }
                else
                {
                    detect_side = "left";
                }
                break;
            }
        }
    }

    if (detect_obstacle)
    {
        if (detect_side == "right")
        {
            pubMsg(0.0, angular_velocity); // その場で反時計回り回転
        }
        else
        {
            pubMsg(0.0, -angular_velocity); // その場時計回り回転
        }
    }
    else
    {
        pubMsg(velocity, 0.0); // 直進
    }
}

int main(int argc, char **argv)
{
    //config
    ros::init(argc, argv, "room_circuit_controller");
    ros::NodeHandle n; //make nodehandler
    ros::NodeHandle pnh("~");

    pnh.getParam("velocity", velocity);
    pnh.getParam("angular_velocity", angular_velocity);
    pnh.getParam("avoid_distance", avoid_distance);
    pnh.getParam("avoid_angle", avoid_angle);

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sub = n.subscribe("/scan", 1, scanCallback);

    ros::spin(); //call CallBack function
}