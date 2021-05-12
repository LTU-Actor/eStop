#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <string>

// Only one node is allowed to call resume and succeed (the first one to call).
// The eStop node starts in the stopped state, so the node that initially
// resumes must be the same node that does it in the future.

#define REPUB_HZ 50
#define ZEROS_HZ 50
#define STATE_HZ 0.5

enum EstopState { RUNNING, STOPPED } static state = STOPPED;

static void stop(void);

static bool stop_cb(ros::ServiceEvent<std_srvs::EmptyRequest, std_srvs::EmptyResponse> &event);
static bool resume_cb(ros::ServiceEvent<std_srvs::TriggerRequest, std_srvs::TriggerResponse> &event);
static void forwarded_cb(const geometry_msgs::Twist::ConstPtr &msg);;
static void publish_zero_cb(const ros::TimerEvent);
static void publish_repub_cb(const ros::TimerEvent);
static void publish_state_cb(const ros::TimerEvent);
static void timeout_cb(const ros::TimerEvent);

static std::string resume_node; // node name with resume permission
static ros::Publisher pub_twist; // Output Twist publisher
static ros::Publisher pub_state; // Periodic state publisher
static ros::Timer zeros; // publish zeros when enabled
static ros::Timer repub;

static ros::Time last_recv;
static geometry_msgs::Twist last_to_repub;

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

    if (!nh.getParam("output_topic", output_topic)) output_topic = "out";

    pub_twist = nh.advertise<geometry_msgs::Twist>(output_topic, 1);
    pub_state = nh.advertise<std_msgs::Bool>("state", 1);
    ros::Subscriber forwarded = nh.subscribe(input_topic, 1, &forwarded_cb);
    ros::ServiceServer srv_stop = nh.advertiseService("stop", &stop_cb);
    ros::ServiceServer srv_restart = nh.advertiseService("resume", &resume_cb);;
    zeros = nh.createTimer(ros::Duration(1.0 / ZEROS_HZ), &publish_zero_cb);
    repub = nh.createTimer(ros::Duration(1.0 / REPUB_HZ), &publish_repub_cb);
    repub.stop();
    zeros.start();

    last_recv = ros::Time::now();
    ros::Timer state = nh.createTimer(ros::Duration(1.0 / STATE_HZ), &publish_state_cb);
    ros::Timer timeout = nh.createTimer(ros::Duration(1.0 / 50), &timeout_cb);

    ros::spin();
    return EXIT_SUCCESS;
}

static void
stop()
{
    state = STOPPED;
    repub.stop();
    zeros.start();
    std_msgs::Bool msg;
    msg.data = true;
    pub_state.publish(msg);
}

static bool
stop_cb(ros::ServiceEvent<std_srvs::EmptyRequest, std_srvs::EmptyResponse> &event)
{
    stop();
    return true;
}

static bool
resume_cb(ros::ServiceEvent<std_srvs::TriggerRequest, std_srvs::TriggerResponse> &event)
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
        last_to_repub = geometry_msgs::Twist();
        repub.start();
        response.message = "";
        response.success = true;
    }

    std_msgs::Bool msg;
    msg.data = (state == STOPPED);
    pub_state.publish(msg);
    return true;
}

static void
forwarded_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    last_recv = ros::Time::now();
    last_to_repub = *msg;
    if (state == RUNNING) pub_twist.publish(msg);
}

static void
publish_zero_cb(const ros::TimerEvent)
{
    geometry_msgs::Twist msg;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    pub_twist.publish(msg);
}

static void
publish_repub_cb(const ros::TimerEvent)
{
  pub_twist.publish(last_to_repub);
}

static void
publish_state_cb(const ros::TimerEvent)
{
    std_msgs::Bool msg;
    msg.data = (state == STOPPED);
    pub_state.publish(msg);
}

static void
timeout_cb(const ros::TimerEvent)
{
    if ((ros::Time::now() - last_recv) > ros::Duration(1.0))
        stop();
}
