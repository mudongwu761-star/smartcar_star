/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2025-01-04 06:50:56
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-03-13 08:15:10
 * @FilePath: /2k300_smartcar/src/image_cv.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "image_cv.h"

cv::Mat raw_frame;
cv::Mat grayFrame;
cv::Mat binarizedFrame;
cv::Mat morphologyExFrame;
cv::Mat track;
cv::Mat resized_raw_Frame;

std::vector<int> left_line;              // 左边缘列号数组
std::vector<int> right_line;             // 右边缘列号数组
std::vector<int> mid_line;               // 中线列号数组
std::vector<double> left_line_filtered;  // 中线列号数组
std::vector<double> right_line_filtered; // 中线列号数组
std::vector<double> mid_line_filtered;   // 中线列号数组

int line_tracking_width;
int line_tracking_height;

cv::Mat image_binerize(cv::Mat &frame)
{
    cv::Mat output;
    cv::Mat binarizedFrame;
    cv::Mat hsvImage;
    cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImage, hsvChannels);

    cv::threshold(hsvChannels[0], binarizedFrame, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::threshold(hsvChannels[1], output, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    cv::bitwise_or(output, binarizedFrame, output);

    return output;
}

cv::Mat find_road(cv::Mat &frame)
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2, 2));
    cv::morphologyEx(binarizedFrame, morphologyExFrame, cv::MORPH_OPEN, kernel);

    cv::Mat mask = cv::Mat::zeros(line_tracking_height + 2, line_tracking_width + 2, CV_8UC1);

    cv::Point seedPoint(line_tracking_width / 2, line_tracking_height - 10);

    cv::circle(morphologyExFrame, seedPoint, 10, 255, -1);

    cv::Scalar newVal(128);

    cv::Scalar loDiff = cv::Scalar(20);
    cv::Scalar upDiff = cv::Scalar(20);

    cv::floodFill(morphologyExFrame, mask, seedPoint, newVal, 0, loDiff, upDiff, 8);

    cv::Mat outputImage = cv::Mat::zeros(line_tracking_width, line_tracking_height, CV_8UC1);

    mask(cv::Rect(1, 1, line_tracking_width, line_tracking_height)).copyTo(outputImage);

    return outputImage;
}

void image_main()
{
    cv::resize(raw_frame, resized_raw_Frame, cv::Size(line_tracking_width, line_tracking_height));

    binarizedFrame = image_binerize(resized_raw_Frame);

    track = find_road(binarizedFrame);

    left_line.clear();
    right_line.clear();
    mid_line.clear();
    left_line_filtered.clear();
    right_line_filtered.clear();
    mid_line_filtered.clear();

    left_line.resize(line_tracking_height, -1);
    right_line.resize(line_tracking_height, -1);
    mid_line.resize(line_tracking_height, -1);
    left_line_filtered.resize(line_tracking_height, -1);
    right_line_filtered.resize(line_tracking_height, -1);
    mid_line_filtered.resize(line_tracking_height, -1);

    uchar(*IMG)[line_tracking_width] = reinterpret_cast<uchar(*)[line_tracking_width]>(track.data);

    for (int i = 0; i < line_tracking_height; ++i)
    {
        int max_start = -1;
        int max_end = -1;
        int current_start = -1;
        int current_length = 0;
        int max_length = 0;

        for (int j = 0; j < line_tracking_width; ++j)
        {
            if (IMG[i][j])
            {
                if (current_length == 0)
                {
                    current_start = j;
                    current_length = 1;
                }
                else
                {
                    current_length++;
                }
                if (current_length >= max_length)
                {
                    max_length = current_length;
                    max_start = current_start;
                    max_end = j;
                }
            }
            else
            {
                current_length = 0;
                current_start = -1;
            }
        }
        if (max_length > 0)
        {
            left_line[i] = max_start;
            right_line[i] = max_end;
        }
        else
        {
            left_line[i] = -1;
            right_line[i] = -1;
        }
    }

    double a = 0.4;
    for (int row = line_tracking_height - 1; row >= 10; --row)
    {
        if (left_line[row] == -1 && right_line[row] == -1)
        {
            mid_line[row] = mid_line[row + 1];
            if (mid_line[row] > line_tracking_width / 2)
            {
                right_line[row] = line_tracking_width - 1;
                left_line[row] = mid_line[row + 1];
            }
            else
            {
                left_line[row] = 0;
                right_line[row] = mid_line[row + 1];
            }
        }
        else
        {
            mid_line[row] = (left_line[row] + right_line[row]) / 2;
        }
        if (row == line_tracking_height - 1)
        {
            left_line_filtered[row] = left_line[row];
            right_line_filtered[row] = right_line[row];
            mid_line_filtered[row] = mid_line[row];
        }
        else
        {
            left_line_filtered[row] = a * left_line[row] + (1 - a) * left_line_filtered[row + 1];
            right_line_filtered[row] = a * right_line[row] + (1 - a) * right_line_filtered[row + 1];
            // mid_line_filtered[row] = a * mid_line[row] + (1 - a) * mid_line_filtered[row + 1];
            mid_line_filtered[row] = (left_line_filtered[row] + right_line_filtered[row]) / 2.0;
        }
    }
}