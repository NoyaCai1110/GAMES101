#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    //MYCODE
    float c = cos(rotation_angle * MY_PI / 180.0);
    float s = sin(rotation_angle * MY_PI / 180.0);
    model << c, -s, 0, 0,   
             s, c, 0, 0,     
             0, 0, 1, 0,     
             0, 0, 0, 1;
    //MYCODE END
    return model;
}
//MYCODE
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
    float x = axis[0]/(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    float y = axis[1]/(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    float z = axis[2]/(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    float c = cos(angle * MY_PI / 180.0);
    float s = sin(angle * MY_PI / 180.0);
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation << x*x+(1-x*x)*c, x*y*(1-c)-z*s, x*z*(1-c)+y*s, 0,
                x*y*(1-c)+z*s, y*y+(1-y*y)*c, y*z*(1-c)-x*s, 0,
                x*z*(1-c)-y*s, y*z*(1-c)+x*s, z*z+(1-z*z)*c, 0,
                0, 0, 0, 1;
    return rotation;
}
//MYCODE END
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    //MYCODE
    float n = zNear;
    float f = zFar;
    Eigen::Matrix4f persp = Eigen::Matrix4f::Identity();
    persp << n, 0, 0, 0,    
             0, n, 0, 0,     
             0, 0, n+f, -n*f,  
             0, 0, 1, 0;   
    float y = -2 * (n * tan(eye_fov * MY_PI / 180.0));
    float x =  y * aspect_ratio;
    Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity();
    ortho_scale << 2/x, 0, 0, 0,    
                   0, 2/y, 0, 0,   
                   0, 0, n-f, 0,   
                   0, 0, 0, 1;
    Eigen::Matrix4f ortho_translate = Eigen::Matrix4f::Identity();
    ortho_translate << 1, 0, 0, 0,    
                       0, 1, 0, 0,    
                       0, 0, 1, -(n+f)/2,     
                       0, 0, 0, 1;    
    projection = ortho_translate * ortho_scale * persp;
    //MYCODE END
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        //MYCODE
        //r.set_rotation(get_rotation((1, 1, 1), 30.0));
        //MYCODE END
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        //MYCODE
        //r.set_rotation(get_rotation((1, 1, 1), 30.0));
        //MYCODE END
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
