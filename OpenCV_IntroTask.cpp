/* This code was developed by Kenville Haynes

Electrical and Electronic Engineering department 

University of Nottingham */

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // Load the image
    Mat image = imread("RedApple.bmp", IMREAD_COLOR);

    // Check if the image is loaded correctly
    if (image.empty())
    {
        cout << "Error: Could not load the image." << endl;
        return -1;
    }

    // Convert the image to HSV colour space
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);

    // Define the range of colours for blue, green and red
    Scalar blue_lower = Scalar(100, 150, 0);
    Scalar blue_upper = Scalar(140, 255, 255);
    Scalar green_lower = Scalar(40, 70, 70);
    Scalar green_upper = Scalar(80, 255, 255);
    Scalar red_lower = Scalar(0, 50, 50);
    Scalar red_upper = Scalar(10, 255, 255);

    // Create binary images for blue, green and red
    Mat blue_mask, green_mask, red_mask;
    inRange(hsv, blue_lower, blue_upper, blue_mask);
    inRange(hsv, green_lower, green_upper, green_mask);
    inRange(hsv, red_lower, red_upper, red_mask);

    // Count the number of non-zero pixels in each binary image
    int blue_pixels = 0;
    int green_pixels = 0;
    int red_pixels = 0;

    blue_pixels = countNonZero(blue_mask);
    green_pixels = countNonZero(green_mask);
    red_pixels = countNonZero(red_mask);

    // Identify the colour of the object
    string color = "unknown";
    if (blue_pixels > green_pixels && blue_pixels > red_pixels)
    {
        color = "blue";
    }
    else if (green_pixels > blue_pixels && green_pixels > red_pixels)
    {
        color = "green";
    }
    else if (red_pixels > blue_pixels && red_pixels > green_pixels)
    {
        color = "red";
    }

    // Print the result
    cout << "The colour of the object is " << color << "." << endl;

    return 0;
}
