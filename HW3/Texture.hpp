//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if (u < 0) u = 0;
		if (u > 1) u = 1;
		if (v < 0) v = 0;
		if (v > 1) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u,float v)
    {
        if(u < 0) u = 0;
        if(u > 1) u = 1;
        if(v < 0) v = 0;
        if(v > 1) v = 1;
        auto u_img = u * (width - 1);
        auto v_img = (1 - v) * (height - 1);
        auto color00 = getColor((std::floor(u_img))/width, 1-(std::floor(v_img))/height);
        auto color10 = getColor((std::ceil(u_img))/width, 1-(std::floor(v_img))/height);
        auto color01 = getColor((std::floor(u_img))/width, 1-(std::ceil(v_img))/height);
        auto color11 = getColor((std::ceil(u_img))/width, 1-(std::ceil(v_img))/height);
        float s = u_img - std::floor(u_img);
        float t = v_img - std::floor(v_img);
        auto color0 = (1 - s) * color00 + s * color10;
        auto color1 = (1 - s) * color01 + s * color11;
        auto color = (1 - t) * color0 + t * color1;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
