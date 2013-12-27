#include "ros/ros.h"
#include "iostream"
#include "cereal_port/CerealPort.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "remote_defines.h"

using namespace std;

bool openPort(const char *str, int baud);
void streamCallback(std::string * msg);
void cmdLED(const std_msgs::Bool::ConstPtr& cmd_led);

cereal::CerealPort serial;
int linear=0, angular=0;
unsigned int buttons = 0;
bool is_connected = false;
ros::Time last_time;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jmotes_node");

    ROS_INFO("JMoteS driver");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string serial_port_path;
    pn.param<std::string>("serial_port", serial_port_path,"/dev/ttyUSB0");

    ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>("/JMoteS/data", 10);
    ros::Subscriber remote_led_sub = n.subscribe<std_msgs::Bool>("/JMoteS/LED", 2, cmdLED);

    // First we open the port...
    if(!openPort((char*)serial_port_path.c_str(),57600))
    {
        ROS_FATAL("JMoteS -- Failed to open serial port %s at 57600 baud!", serial_port_path.c_str());
        ROS_BREAK();
    }
    ROS_INFO("JMoteS -- Successfully connected to JMoteS!");

    if(!serial.startReadBetweenStream(boost::bind(&streamCallback, _1), ':', ';'))
    {
        return false;
    }
    ROS_INFO("JMoteS -- Successfully configured read stream !");

    ros::Rate r(10);
    ros::Time start_time = ros::Time::now();

    while(n.ok())
    {
        if((ros::Time::now() - last_time).toNSec() < 100000000)
        {
            sensor_msgs::Joy remote_msg;
            remote_msg.header.stamp = ros::Time::now();
            remote_msg.header.frame_id = "/JMotes_frame_id";
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(float(linear)/128.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(float(angular)/128.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back((buttons & RB_BUTTON) >> 3);
            remote_msg.buttons.push_back((buttons & LB_BUTTON) >> 1);
            remote_msg.buttons.push_back(0);            
            remote_msg.buttons.push_back((buttons & X_BUTTON) >> 0);
            remote_msg.buttons.push_back((buttons & B_BUTTON) >> 2);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            joy_pub.publish(remote_msg);
        }
        else
        {
            sensor_msgs::Joy remote_msg;
            remote_msg.header.stamp = ros::Time::now();
            remote_msg.header.frame_id = "/JMotes_frame_id";
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.axes.push_back(0.0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            remote_msg.buttons.push_back(0);
            joy_pub.publish(remote_msg);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

bool openPort(const char *str, int baud)
{
    try {
        serial.open(str,baud);
    } catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

void streamCallback(std::string * msg)
{
    int scan_res;
    char h1[50];

    scan_res = sscanf(msg->c_str(), ":%s %*s", h1);

    //ROS_INFO("received msg: |%s|.", msg->c_str());

    if(scan_res == 1)                                   // must scan 1 entry
    {
        if(!strncmp(h1,"L",1))                        // header
        {
            int d1=0, d2=0, d3 = 0, scan_res;
            char h1[3],h2[1],h3[1];

            //scan message
            scan_res=sscanf(msg->c_str(), ":%c %d %c %d %c %x;%*s",h1, &d1, h2, &d2, h3, &d3);

            if(scan_res == 6 && h1[0]=='L' && h2[0]=='A' && h3[0]=='B')
            {
                linear = d1;
                angular = d2;
                buttons = d3;

                last_time = ros::Time::now();

                ROS_INFO("received cmd: l = %d, a = %d, b = %d.", linear, angular, buttons);
            }
        }
    }
}

void cmdLED(const std_msgs::Bool::ConstPtr& cmd_led)
{
    int num_bytes;
    char msg_to_send[20];

    num_bytes = sprintf(msg_to_send, ":led %d;", cmd_led->data);
    serial.write(msg_to_send, num_bytes);
}
