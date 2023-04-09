#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv_aee.hpp"
#include "pi2c.h"
#include <tuple>
#include <chrono>
#include <thread>
#include <vector>

using namespace std;
using namespace cv;

int left_baseSpeed = 160;   //EEEBot motor stable speed
int right_baseSpeed = 160;  //EEEBot motor stable speed

int midServo = 52;   //EEEBot center steering

int leftSpeed = 0;
int rightSpeed = 0;
int servoPos = 52;

const int nano_address = 0x04;

// PID tuning parameters
const double Kp = 2.6;
const double Ki = 0;// 0.01;
const double Kd = 0.1; //0.1;

// PID error terms
double pid_error = 0;
double last_error = 0;
double pid_integral = 0;
double pid_derivative = 0;
const double K = 0.1;
double pid_output = 0;

// Camera resolution
const int cam_width = 320;
const int cam_height = 240;

const int ROI_TOP = 0;
const int ROI_BOTTOM = 30;
const int ROI_LEFT = 0;
const int ROI_RIGHT = cam_width;

const int setpoint = 160; //cam_width / 2

const int max_count  = 9600; //Max count for top 30 row of pixels

const int template_width = 350;
const int template_height = 350;


Scalar lower_pink(143, 42, 38);
Scalar upper_pink(169, 255, 255);

Scalar red_low(0, 52, 49);
Scalar red_high(12, 255, 255);

Scalar green_low(40, 70, 70);
Scalar green_high(80, 255, 255);

Scalar blue_low(100, 150, 0);
Scalar blue_high(140, 255, 255);

Scalar black_low(0, 0, 0);
Scalar black_high(180, 255, 50);


Scalar yellow_low(20, 100, 100);
Scalar yellow_high(40, 255, 255);

// Transmit to Arduino Nano
void transmitArduino(int leftSpeed, int rightSpeed, int servoPos)
{
    Pi2c i2c(nano_address);

    char data[6];

    data[0] = (leftSpeed >> 8) & 0xFF;
    data[1] = leftSpeed & 0xFF;
    data[2] = (rightSpeed >> 8) & 0xFF;
    data[3] = rightSpeed & 0xFF;
    data[4] = (servoPos >> 8) & 0xFF;
    data[5] = servoPos & 0xFF;

    i2c.i2cWrite(data, 6);

    //Function: Destructor - Closes the I2C File handle.
    i2c.~Pi2c();
}

template <typename T>
T constrain(T value, T minValue, T maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    else if (value > maxValue)
    {
        return maxValue;
    }
    else
    {
        return value;
    }
}


void PID(double pid_error)
{

    last_error = pid_error;

    // Update the integral and derivative error terms
    pid_integral += pid_error;
    pid_derivative = pid_error - last_error;

    // Calculate the PID control output
    pid_output = Kp * pid_error + Ki * pid_integral + Kd * pid_derivative;

    leftSpeed = left_baseSpeed + K * pid_output;    //Left motor speed is base speed + scaling factor K multiplied by the PID output
    rightSpeed = right_baseSpeed - K * pid_output; //Right motor speed is base speed - scaling factor K multiplied by the PID output
    servoPos = midServo + pid_output; //Servo angle is the center angle + the PID output

    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    servoPos = constrain(servoPos, 0, 100);

    transmitArduino(leftSpeed, rightSpeed, servoPos);  //Transmit to Arduino NANO over I2C which controls speed and servo

    //std::cout << "Error: " << error << std::endl;

    //std::cout << "Output: " << output << std::endl;
}

Mat process_frame(Scalar lower_bound, Scalar upper_bound)
{
    // Capture image
    Mat frame = captureFrame();

    // Crop the image to the region of interest
    Rect roi(ROI_LEFT, ROI_TOP, ROI_RIGHT - ROI_LEFT, ROI_BOTTOM - ROI_TOP);
    frame = frame(roi);

    // Convert frame to HSV color space
    Mat hsv_frame, binary_frame;
    cvtColor(frame, hsv_frame, COLOR_BGR2HSV);

    // Apply the color mask
    inRange(hsv_frame, lower_bound, upper_bound, binary_frame);
    bitwise_not(binary_frame, binary_frame);

//   cv::imshow("Process frame function", frame);
    // Wait for the user to close the window
//    cv::waitKey(1);

    return binary_frame;
}

void red_line()
{

    double red_cnt = 0;
    double black_cnt = 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));


    do
    {

        Mat binary_frame = process_frame(red_low, red_high);
        red_cnt = countNonZero(binary_frame);

        Mat black_binary_frame = process_frame(black_low, black_high);
        black_cnt = countNonZero(black_binary_frame);


//       std::cout << "Red count: " << red_cnt << std::endl;
//       std::cout << "Black count: " << black_cnt << std::endl;

        pid_error = (moments(binary_frame, true).m10 / moments(binary_frame, true).m00) - setpoint;
        PID(pid_error);


    }
    while (black_cnt == max_count && red_cnt < max_count); //Do until black line is recognisied and no red is
}


void green_line()
{

//  double green_cnt = 0;
    double black_cnt = 0;

    do
    {
        Mat binary_frame = process_frame(green_low, green_high);
//        green_cnt = countNonZero(binary_frame);

        Mat black_binary_frame = process_frame(black_low, black_high);
        black_cnt = countNonZero(black_binary_frame);

        // std::cout << "Red count: " << red_cnt << std::endl;
        std::cout << "Black count: " << black_cnt << std::endl;

        pid_error = (moments(binary_frame, true).m10 / moments(binary_frame, true).m00) - setpoint;
        PID(pid_error);

    }
    while (black_cnt >= max_count);
}


void blue_line()
{

//  double blue_cnt = 0;
    double black_cnt = 0;

    do
    {
        Mat binary_frame = process_frame(blue_low, blue_high);
//        blue_cnt = countNonZero(binary_frame);

        Mat black_binary_frame = process_frame(black_low, black_high);
        black_cnt = countNonZero(black_binary_frame);

        // std::cout << "Red count: " << red_cnt << std::endl;
        std::cout << "Black count: " << black_cnt << std::endl;

        pid_error = (moments(binary_frame, true).m10 / moments(binary_frame, true).m00) - setpoint;
        PID(pid_error);

    }
    while (black_cnt >= max_count);

}

void yellow_line()
{

//  double yellow_cnt = 0;
    double black_cnt = 0;

    do
    {
        Mat binary_frame = process_frame(yellow_low, yellow_high);
//      yellow_cnt = countNonZero(binary_frame);

        Mat black_binary_frame = process_frame(black_low, black_high);
        black_cnt = countNonZero(black_binary_frame);

        // std::cout << "Red count: " << red_cnt << std::endl;
        std::cout << "Black count: " << black_cnt << std::endl;

        pid_error = (moments(binary_frame, true).m10 / moments(binary_frame, true).m00) - setpoint;
        PID(pid_error);

    }
    while (black_cnt >= max_count);

}


void black_line()
{
    double black_cnt = 0;

    Mat black_binary_frame = process_frame(black_low, black_high);
    // black_cnt = countNonZero(black_binary_frame);

    // std::cout << "Red count: " << red_cnt << std::endl;
    // std::cout << "Black count: " << black_cnt << std::endl;

    pid_error = (moments(black_binary_frame, true).m10 / moments(black_binary_frame, true).m00) - setpoint;
    PID(pid_error);

}



void load_templates(vector<Mat> &template_masks)
{
    // Load the four symbol templates
    vector<String> template_paths = {"circle.png", "star.png", "triangle.png", "umbrella.png"};

    Scalar lower_pink(143, 42, 38);
    Scalar upper_pink(169, 255, 255);

    for (const String &path : template_paths)
    {
        Mat template_img = imread(path, IMREAD_COLOR);

        if (template_img.empty())
        {
            std::cout << "Error: could not load the symbol image at " << path << std::endl;
            return;
        }

        Rect roi(30, 30, template_img.cols - 60, template_img.rows - 60);
        Mat template_roi = template_img(roi);

        Mat template_hsv_image;
        cvtColor(template_roi, template_hsv_image, COLOR_BGR2HSV);

        Mat template_mask;
        inRange(template_hsv_image, lower_pink, upper_pink, template_mask);

        resize(template_mask, template_mask, Size(template_width, template_height));

        template_masks.push_back(template_mask);
    }
}




bool symbol_rec(int &choice, const vector<Mat> &template_masks)
{



    bool stop_program = false;

    // Define structuring elements for morphology operations
    Mat erode_element = getStructuringElement(MORPH_RECT, Size(3, 3));
//   Mat dilate_element = getStructuringElement(MORPH_RECT, Size(3, 3));

    // Loop through each frame from the camera
    while (!stop_program)
    {

        // Capture image
        Mat input_image = captureFrame();

        // Crop the image to the region of interest
//        Rect input_image_roi(ROI_LEFT_SR, ROI_TOP_SR, ROI_RIGHT_SR - ROI_LEFT_SR, ROI_BOTTOM_SR - ROI_TOP_SR);
//       input_image = input_image(input_image_roi);


        resize(input_image, input_image, Size(template_width, template_height));


        // Rotate the image by 180 degrees
        cv::flip(input_image, input_image, -1);
        //Brighten the image
        //input_image.convertTo(input_image, -1, 1.5, 0);

        // Apply Gaussian blur to reduce noise
//       GaussianBlur(input_image, input_image, Size(5, 5), 0);
        // Erode the mask to remove small objects
        erode(input_image, input_image, erode_element);
        // Dilate the mask to fill gaps
//        dilate(input_image, input_image, dilate_element);

//        cv::imshow("Edit Image", input_image);


        // convert the ROI to HSV format
        Mat hsv_image;
        cvtColor(input_image, hsv_image, COLOR_BGR2HSV);

        // threshold the image to isolate the pink pixels
        Mat mask;
        inRange(hsv_image, lower_pink, upper_pink, mask);

        // Display the image in a window
        //cv::imshow("Input Mask image", mask);
        /*
                float pink_count = countNonZero(mask);
                std::cout << "Pink count: " << pink_count << std::endl;
        */

        // Find contours in the mask
        vector<vector<Point> > contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Find largest contour
        int largest_contour_index = 0;
        double largest_contour_area = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            double contour_area = contourArea(contours[i]);
            if (contour_area > largest_contour_area)
            {
                largest_contour_index = i;
                largest_contour_area = contour_area;
            }
        }


        // Draw largest contour on image
        drawContours(input_image, contours, largest_contour_index, Scalar(0, 0, 255), 2);
        // Show output image
        imshow("Contour Image", input_image);


        if (!contours.empty())
        {
            //std::cout << "Contour not empty" << std::endl;

            // Find the four corners of the largest contour
            vector<Point> boundingContour;
            approxPolyDP(Mat(contours[largest_contour_index]), boundingContour, arcLength(Mat(contours[largest_contour_index]), true) * 0.02, true);

            // Check if the pink border has been detected
            if (boundingContour.size() == 4)
            {
                // Align the image with the four corners
                Mat aligned_image = transformPerspective(boundingContour, mask, template_width, template_height);

                // Display the aligned image
                //cv::imshow("Aligned Image", aligned_image);

                if (aligned_image.cols > 60 && aligned_image.rows > 60)
                {
                    int x = 30;
                    int y = 30;
                    int width = aligned_image.cols - 60;
                    int height = aligned_image.rows - 60;

                    if (x >= 0 && y >= 0 && x + width <= aligned_image.cols && y + height <= aligned_image.rows)
                    {

                        // Define the ROI of the input image to exclude the 30-pixel border
                        Rect roi(x, y, width, height);
                        Mat roi_mask = aligned_image(roi);

                        resize(roi_mask, roi_mask, Size(template_width, template_height));

                        // Display the image in a window
//                        cv::imshow("ROI image", roi_mask);


                        // compare the transformed image with the four symbol templates


                        double match_circle = compareImages(roi_mask, template_masks[0]);
                        double match_star = compareImages(roi_mask, template_masks[1]);
                        double match_triangle = compareImages(roi_mask, template_masks[2]);
                        double match_umbrella = compareImages(roi_mask, template_masks[3]);


                        /*                    // Display the image in a window
                                            cv::imshow("Circle template image", circle_template_mask);
                                            // Display the image in a window
                                            cv::imshow("Star template image", star_template_mask);
                                            // Display the image in a window
                                            cv::imshow("Triangle template image", triangle_template_mask);
                                            // Display the image in a window
                                            cv::imshow("Umbrella template image", umbrella_template_mask);
                        */

                        std::cout << "Match circle: " << match_circle << std::endl;
                        std::cout << "Match star: " << match_star << std::endl;
                        std::cout << "Match triangle: " << match_triangle << std::endl;
                        std::cout << "Match umbrella: " << match_umbrella << std::endl;

                        // set a threshold value for the match value
                        float threshold = 75.0;

                        //int max_count = max({match_circle, match_star});

                        // output the detected symbol or no symbol
                        if (match_circle > threshold)
                        {
                            cout << "Circle detected" << endl;

                            choice = 1;

                            return true;

                        }
                        else if (match_star > threshold)
                        {
                            cout << "Star detected" << endl;

                            choice = 2;

                            return true;

                        }
                        else if (match_triangle > threshold)
                        {
                            cout << "Triangle detected" << endl;

                            choice = 3;

                            return true;

                        }
                        else if (match_umbrella > threshold)
                        {
                            cout << "Umbrella detected" << endl;

                            choice = 4;

                            return true;

                        }
                        else
                        {
                            cout << "No symbol detected" << endl;

                            return false;
                        }

                    }

                }
            }
            else if (boundingContour.size() != 4)
            {

                //    cout << "Borders not detected" << endl;

                //choice = 0;

                return false;

            }

        }

        cv::waitKey(1);

    }
    return choice;
}


void PID_stop()
{

    //EEEBot is off the line

    leftSpeed = 0; //left_baseSpeed + K * output;    //Left motor speed is base speed + scaling factor K multiplied by the PID output
    rightSpeed = 0;//right_baseSpeed - K * output; //Right motor speed is base speed - scaling factor K multiplied by the PID output
    servoPos = midServo; //Servo angle is the center angle + the PID output
    transmitArduino(leftSpeed, rightSpeed, servoPos);  //Transmit to Arduino NANO over I2C which controls speed and servo


    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void PID_reverse()
{
    //EEEBot is off the line

    leftSpeed = -160; //left_baseSpeed + K * output;    //Left motor speed is base speed + scaling factor K multiplied by the PID output
    rightSpeed = -160;//right_baseSpeed - K * output; //Right motor speed is base speed - scaling factor K multiplied by the PID output
    servoPos = midServo; //Servo angle is the center angle + the PID output
    transmitArduino(leftSpeed, rightSpeed, servoPos);  //Transmit to Arduino NANO over I2C which controls speed and servo

}

void turnLeft()
{
    //EEEBot is off the line

    leftSpeed = 160; //left_baseSpeed + K * output;    //Left motor speed is base speed + scaling factor K multiplied by the PID output
    rightSpeed = 180;//right_baseSpeed - K * output; //Right motor speed is base speed - scaling factor K multiplied by the PID output
    servoPos = 30; //Servo angle is the center angle + the PID output
    transmitArduino(leftSpeed, rightSpeed, servoPos);  //Transmit to Arduino NANO over I2C which controls speed and servo

}


void turnRight()
{
    //EEEBot is off the line

    leftSpeed = 180; //left_baseSpeed + K * output;    //Left motor speed is base speed + scaling factor K multiplied by the PID output
    rightSpeed = 160;//right_baseSpeed - K * output; //Right motor speed is base speed - scaling factor K multiplied by the PID output
    servoPos = 80; //Servo angle is the center angle + the PID output
    transmitArduino(leftSpeed, rightSpeed, servoPos);  //Transmit to Arduino NANO over I2C which controls speed and servo

}

void goForward()
{
    //EEEBot is off the line

    leftSpeed = 160; //left_baseSpeed + K * output;    //Left motor speed is base speed + scaling factor K multiplied by the PID output
    rightSpeed = 160;//right_baseSpeed - K * output; //Right motor speed is base speed - scaling factor K multiplied by the PID output
    servoPos = midServo; //Servo angle is the center angle + the PID output
    transmitArduino(leftSpeed, rightSpeed, servoPos);  //Transmit to Arduino NANO over I2C which controls speed and servo


}

Rect find_pink_border_bounding_rect(Mat &mask)
{
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int largest_contour_index = 0;
    double largest_contour_area = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        double contour_area = contourArea(contours[i]);
        if (contour_area > largest_contour_area)
        {
            largest_contour_index = i;
            largest_contour_area = contour_area;
        }
    }

    if (!contours.empty())
    {
        return boundingRect(contours[largest_contour_index]);
    }
    else
    {
        return Rect();
    }
}


/*
// Adjusts the car's position until it is within the desired range
void adjustCarPosition()
{


    Mat input_image = captureFrame();

//    resize(input_image, input_image, Size(320, 240));
//    cv::flip(input_image, input_image, -1);

//        cv::imshow("Adjust postion camera", input_image);
    // Wait for the user to close the window
//      cv::waitKey(1);

    Mat hsv_image;
    cvtColor(input_image, hsv_image, COLOR_BGR2HSV);

    Mat mask;
    inRange(hsv_image, lower_pink, upper_pink, mask);

    Rect bounding_rect = find_pink_border_bounding_rect(mask);
    Point centroid = (bounding_rect.tl() + bounding_rect.br()) * 0.5;

    int error_x = centroid.x - input_image.cols / 2;
    int error_y = centroid.y - input_image.rows / 2;

//    std::cout << "Error X: " << error_x << std::endl;
//    std::cout << "Error Y " << error_y << std::endl;

    //EEEBot is off the line


    if(error_x < 35 && error_x > -35 && error_y <62 && error_y > -62)
    {
        PID_stop();

    }


    // Check if pink pixels are detected
      int pink_cnt = countNonZero(mask);

       std::cout << "Pink count" << pink_cnt << std::endl;

       if(pink_cnt < 7500)
       {

           PID_stop();
           PID_reverse();

       }


       // Adjust car's position based on error
       if (error_x > 35)
       {

           turnLeft();

       }
       else if(error_x < -35)
       {
           turnRight();
       }
       else if (error_y > 62)
       {

           goForward();
       }
       else if(error_y < -62)
       {
           PID_reverse();
       }
       else
       {

           // Stop the car once it is within the desired range
           PID_stop();
       }




}*/


int adjust_and_recognize(const vector<Mat> &template_masks)
{
    int choice;
    bool symbol_recognized = false;

    while (!symbol_recognized)
    {

//        cout << "Waiting for symbol " << endl;

//        adjustCarPosition(); // Adjust the car position
        symbol_recognized = symbol_rec(choice,template_masks); // Try to recognize a symbol
    }

//    cout << "Symbol recognized: " << choice << endl;

    return choice;
}


int main(int argc, char** argv)
{

    // Initialize camera
    setupCamera(cam_width, cam_height);

    vector<Mat> template_masks;
    load_templates(template_masks);

    if (template_masks.size() != 4)
    {
        return -1;
    }


    bool stop_program = false;

    // Loop through each frame from the camera
    while (!stop_program)
    {
        // Capture image
        Mat input_image = captureFrame();

        // Crop the image to the region of interest
        Rect roi(ROI_LEFT, ROI_TOP, ROI_RIGHT - ROI_LEFT, ROI_BOTTOM - ROI_TOP);
        input_image = input_image(roi);

        // Search for pink pixels in the image
        Mat hsv_image;
        cvtColor(input_image, hsv_image, COLOR_BGR2HSV);

        Mat pink_mask;
        inRange(hsv_image, lower_pink, upper_pink, pink_mask);

        // Check if pink pixels are detected
        bool pink_detected = countNonZero(pink_mask) > 0;

        if (pink_detected)
        {

           // std::this_thread::sleep_for(std::chrono::milliseconds(500));

            PID_stop();
            //PID_reverse();

            int choice = adjust_and_recognize(template_masks);

            // Execute function based on the recognized symbol
            switch (choice)
            {
            case 1:
                cout << "Red line following" << endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                red_line();
                break;
            case 2:
                cout << "Green line following" << endl;
                green_line();
                break;
            case 3:
                cout << "Blue line following" << endl;
                blue_line();
                break;
            case 4:
                cout << "Yellow line following" << endl;
                yellow_line();
            default:
                cout << " Default black line following" << endl;
                black_line();
                break;
            }
        }
        else
        {
            // Follow the black line if no symbol detected
            cout << "Black line following" << endl;
            black_line();
        }

//       cv::imshow("Camera", input_image);
        // Wait for the user to close the window
//       cv::waitKey(1);
    }

    destroyAllWindows();
    closeCV();  // Disable the camera

    return 0;
}



