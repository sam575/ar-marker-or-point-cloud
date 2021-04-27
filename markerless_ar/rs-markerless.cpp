#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "cv-helpers.hpp"    // Helper functions for conversions between RealSense and OpenCV
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/rgbd.hpp>

using namespace cv;
using namespace rs2;
using namespace std;

void draw_cube(Mat& img, vector<Point2f>& imgpts);

colorizer colorize;
rs2::align align_to(RS2_STREAM_COLOR);

rs2::pointcloud pc;
rs2::points points_;

// Start the camera
pipeline pipe;

Vec3f viewpoint = { 0.0, 0.0, 0.0 };
int w = 640;
int h = 480;

// should be odd
int win_h = 51;
int win_w = 51;

int n = win_w * win_h;
Mat cv_points = Mat::zeros(n, 3, CV_32F);
Mat pc_normals = Mat::zeros(n, 6, CV_32F);
Mat normals = Mat::zeros(win_h, win_w, CV_32FC3);

Vec3f rvec;
Vec3f tvec;

bool mouse_event = false;
bool valid_event = false;
int mouse_x;
int mouse_y;

const auto window_name = "Display Image";
const auto window_ar = "Markerless AR";

Vec3f get_cv_rot(Vec3f& x, Vec3f& y, Vec3f& z) {
    // Create 3x3 rotation matrix 
    Mat rotm = Mat::zeros(3, 3, CV_32F);
    for (int i = 0; i < 3; i++) {
        rotm.at<float>(i, 0) = x[i];
    }
    for (int i = 0; i < 3; i++) {
        rotm.at<float>(i, 1) = y[i];
    }
    for (int i = 0; i < 3; i++) {
        rotm.at<float>(i, 2) = z[i];
    }
    
    Vec3f res;
    Rodrigues(rotm, res);
    return res;
}

void compute_pose(Vec3f point, Vec3f z_dir) {
    cout << "V:" << point << endl;
    cout << "N:" << z_dir << endl;

    z_dir = normalize(z_dir);
    Vec3f view_dir = normalize(point);
    Vec3f y_dir = normalize(z_dir.cross(view_dir));
    Vec3f x_dir = normalize(y_dir.cross(x_dir));

    rvec = 1.0 * get_cv_rot(x_dir, y_dir, z_dir);
    tvec = 1.0 * point;
}

bool gradient_based_normals(int x, int y, Mat& points_3d, Mat& depth_normals) {
    auto point = points_3d.at<Vec3d>(y, x);
    if (point[2] == 0.0)
        return false;
    valid_event = true;
    cout << "Valid event: Grad" << endl;
    compute_pose(point, depth_normals.at<Vec3d>(y, x));
    return true;
}

void fit_plane(int x, int y) {
    auto vertices = points_.get_vertices();

    if (vertices[y * w + x].z == 0) {
        return;
    }
    if (y < win_h / 2 || y >= h - win_h / 2 || x < win_w / 2 || x >= w - win_w / 2)
        return;

    valid_event = true;
    cout << "Valid event: fit" << endl;

    int cnt = 0;
    for (int i = y - win_h / 2; i < y + win_h / 2 + 1; i++) {
        for (int j = x - win_w / 2; j < x + win_w / 2 + 1; j++) {
            cv_points.at<float>(cnt, 0) = vertices[i * w + j].x;
            cv_points.at<float>(cnt, 1) = vertices[i * w + j].y;
            cv_points.at<float>(cnt, 2) = vertices[i * w + j].z;
            cnt += 1;
        }
    }

    ppf_match_3d::computeNormalsPC3d(cv_points, pc_normals, 100, true, viewpoint);
    for (int i = 0; i < win_h; i++) {
        for (int j = 0; j < win_w; j++) {
            normals.at<Vec3f>(i, j)[0] = pc_normals.at<float>(i * win_w + j, 3);
            normals.at<Vec3f>(i, j)[1] = pc_normals.at<float>(i * win_w + j, 4);
            normals.at<Vec3f>(i, j)[2] = pc_normals.at<float>(i * win_w + j, 5);
        }
    }
    compute_pose((Vec3f) vertices[y * w + x], normals.at<Vec3f>(win_h / 2, win_w / 2));
}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        mouse_event = true;
        mouse_x = x;
        mouse_y = y;
    }
}

int main(int argc, char* argv[]) try
{
    int cnt = 0;

    namedWindow(window_name, WINDOW_AUTOSIZE);

    rs2::pipeline_profile profile = pipe.start();

    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = color_stream.get_intrinsics();

    Mat camMatrix = Mat::zeros(3, 3, CV_32F);
    Mat distCoeffs = Mat::zeros(1, 5, CV_32F);
    camMatrix.at<float>(0, 0) = intr.fx;
    camMatrix.at<float>(0, 2) = intr.ppx;
    camMatrix.at<float>(1, 1) = intr.fy;
    camMatrix.at<float>(1, 2) = intr.ppy;
    camMatrix.at<float>(2, 2) = 1;

    float axis_len = 0.15;
    vector<Point3f> cube_axis;
    cube_axis.push_back(Point3f(0, 0, 0));
    cube_axis.push_back(Point3f(0, axis_len, 0));
    cube_axis.push_back(Point3f(axis_len, axis_len, 0));
    cube_axis.push_back(Point3f(axis_len, 0, 0));
    cube_axis.push_back(Point3f(0, 0, axis_len));
    cube_axis.push_back(Point3f(0, axis_len, axis_len));
    cube_axis.push_back(Point3f(axis_len, axis_len, axis_len));
    cube_axis.push_back(Point3f(axis_len, 0, axis_len));
    for (int i = 0; i < 8; i++) {
        cube_axis[i] -= Point3f(axis_len / 2, axis_len / 2, 0);
    }
    
    vector<Point2f> imgpts;
    Mat image, image1;
    rs2::colorizer c;

    bool grad_nor = true;
    Mat depth_mat, image2, surface, points_3d;
    rgbd::RgbdNormals RGBD;
    if (grad_nor) {
        RGBD = rgbd::RgbdNormals(h, w, CV_64F, camMatrix, 7, 2);
    }

    // Skips some frames to allow for auto-exposure stabilization
    for (int i = 0; i < 10; i++) pipe.wait_for_frames();

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        frameset data = pipe.wait_for_frames();
        // Make sure the frameset is spatialy aligned 
        // (each pixel in depth image corresponds to the same pixel in the color image)
        frameset aligned_set = align_to.process(data);
        auto depth = aligned_set.get_depth_frame();
        auto color = aligned_set.get_color_frame();
        auto color_mat = frame_to_mat(color);
        if (cnt == 0) {
            cout << "Depth units:" << depth.get_units() << endl;
        }

        imshow(window_name, color_mat);
        setMouseCallback(window_name, onMouse, 0);

        if (grad_nor) {
            depth_mat = depth_frame_to_meters(depth);
            GaussianBlur(depth_mat, depth_mat, Size(11, 11), 0, 0, BORDER_DEFAULT);
            rgbd::depthTo3d(depth_mat, camMatrix, points_3d);
            RGBD(points_3d, surface);
            imshow("Depth normals", surface);
        }

        auto colorized_depth = c.colorize(depth);
        imshow("Depth", frame_to_mat(colorized_depth));

        if (mouse_event) {
            pc.map_to(color);
            points_ = pc.calculate(depth);
            fit_plane(mouse_x, mouse_y);
        }

        if (valid_event) {
            valid_event = false;
            color_mat.copyTo(image);
            color_mat.copyTo(image1);

            //aruco::drawAxis(image1, camMatrix, distCoeffs, rvec, tvec, 0.1f);
            projectPoints(cube_axis, rvec, tvec, camMatrix, distCoeffs, imgpts);
            draw_cube(image1, imgpts);
            imshow("AR image - Plane fitting", image1);
            //imwrite("markerless_ar.png", image1);
            //imshow(window_ar, image);
        }

        if (grad_nor && mouse_event && gradient_based_normals(mouse_x, mouse_y, points_3d, surface)) {
            color_mat.copyTo(image2);
            projectPoints(cube_axis, rvec, tvec, camMatrix, distCoeffs, imgpts);
            draw_cube(image2, imgpts);
            imshow("AR image - Gradient based", image2);
        }
        mouse_event = false;
        valid_event = false;
        cnt++;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void draw_cube(Mat& img, vector<Point2f>& imgpts) {
    vector<vector<Point>> contours;
    vector<Point> bottom;
    vector<Point> top;
    for (int i = 0; i < 4; i++) {
        bottom.push_back(imgpts[i]);
        top.push_back(imgpts[i + 4]);
    }
    contours.push_back(bottom);
    contours.push_back(top);

    drawContours(img, contours, 0, Scalar(0, 255, 0), -3);
    drawContours(img, contours, 1, Scalar(0, 0, 255), 3);

    for (int i = 0; i < 4; i++) {
        line(img, imgpts[i], imgpts[i + 4], Scalar(255, 0, 0), 3);
    }
}