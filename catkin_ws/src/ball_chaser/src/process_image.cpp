#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    client.call(srv);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // Number of white pixels in the left, middle, and right part of the image, respectively
    int left_white_pixel_cnt = 0;
    int middle_white_pixel_cnt = 0;
    int right_white_pixel_cnt = 0;

    for(int h=0; h<img.height; h++)
    {
        // Dividing the image width or to be precise its 'step' into three parts by 30% left, next 40% into middle, & remaining 30% into right
        
        // Left part/side
        for(int s=0; s<0.3*img.step; s = s + 3)
        {
            // In 'sensor_msgs/image', data is arranged in a 1D vector whereby one pixel is represented by three consecutive bytes (uint8) comprising the RED, BLUE, and GREEN color information.
            // So, in each iteration, I'm checking three consecutive pixels/bytes
            if((img.data[s + img.step*h] == white_pixel) && (img.data[s + img.step*h + 1] == white_pixel) && (img.data[s + img.step*h+ 2] == white_pixel))
            {
                left_white_pixel_cnt++;
            }
        }

        // middle part
        for(int s=0.3*img.step; s < 0.7*img.step; s = s + 3)
        {
            if((img.data[s + img.step*h] == white_pixel) && (img.data[s + img.step*h + 1] == white_pixel) &&(img.data[s + img.step*h+ 2] == white_pixel))
            {
                middle_white_pixel_cnt++;
            }
        }

        // right part/side
        for(int s=0.7*img.step; s < img.step; s = s + 3)
        {
            if((img.data[s + img.step*h] == white_pixel) && (img.data[s + img.step*h + 1] == white_pixel) && (img.data[s + img.step*h+ 2] == white_pixel))
            {
                right_white_pixel_cnt++;
            }
        }
    }

    // First, check if there is a white ball in the image or not
    // If not then stop 
    if((left_white_pixel_cnt == 0) && (middle_white_pixel_cnt == 0) && (right_white_pixel_cnt == 0))
    {
        drive_robot(0.0, 0.0);
    }
    // if it is then check it's in which part of the image
    else
    {
        // If the white ball is in the left side of the image, turn right
        if((left_white_pixel_cnt > right_white_pixel_cnt) && (left_white_pixel_cnt > middle_white_pixel_cnt))
        {
            drive_robot(0.3, 0.3);
        }

        // If the white ball is in the middle part of the image, go straight
        else if((middle_white_pixel_cnt > left_white_pixel_cnt) && (middle_white_pixel_cnt > right_white_pixel_cnt))
        {
            drive_robot(0.3, 0.0);
        }

        // If the white ball is in the right side of the image, turn right
        else
        {
            drive_robot(0.3, -0.3);
        }
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}