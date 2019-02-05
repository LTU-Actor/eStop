#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <string>

#define ZEROS_HZ 50

enum EstopState { RUNNING, STOPPED } static state = STOPPED;

// Only one node is allowed to call resume and succeed (the first one to call).
// The eStop node starts in the stopped state, so the node that initially
// resumes must be the same node that does it in the future.
static std::string resume_node;

// Output publisher
static ros::Publisher pub;

// publish zeros when enabled
static ros::Timer zeros;

static bool
stop(ros::ServiceEvent<std_srvs::EmptyRequest, std_srvs::EmptyResponse> &event)
{
    state = STOPPED;
    zeros.start();
    return true;
}

static bool
resume(ros::ServiceEvent<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
           &event)
{
    const std::string &caller = event.getCallerName();
    if (resume_node.empty()) resume_node = caller;

    std_srvs::TriggerResponse &response = event.getResponse();
    if (resume_node != caller)
    {
        response.message = "Permission denied";
        response.success = false;
    }
    else
    {
        state = RUNNING;
        zeros.stop();
        response.message = "";
        response.success = true;
    }

    return true;
}

static void
forwarded_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (state == RUNNING) pub.publish(msg);
}

void
publish_zero(const ros::TimerEvent)
{
    geometry_msgs::Twist msg;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    pub.publish(msg);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "eStop");
    ros::NodeHandle nh("~");

    std::string input_topic, output_topic;

    if (!nh.getParam("input_topic", input_topic))
    {
        ROS_ERROR_STREAM("eStop forwarded topic not specified");
        return EXIT_FAILURE;
    }

    if (!nh.getParam("output_topic", output_topic))
        output_topic = "out";

    pub = nh.advertise<geometry_msgs::Twist>(output_topic, 1);
    ros::Subscriber forwarded = nh.subscribe(input_topic, 1, &forwarded_cb);
    ros::ServiceServer srv_stop = nh.advertiseService("stop", &stop);
    ros::ServiceServer srv_restart = nh.advertiseService("resume", &resume);
    zeros = nh.createTimer(ros::Duration(1.0 / ZEROS_HZ), &publish_zero);

    ros::spin();
    return EXIT_SUCCESS;
}
