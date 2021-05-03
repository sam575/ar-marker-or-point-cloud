# Markerless AR using point cloud data

Overlay virtual objects on user selected points using surface normals computed from point cloud data. Point cloud data is acquired using [Intel's depth camera](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)

<p align="center">
  <img src="https://github.com/sam575/ar-marker-or-point-cloud/blob/master/images/markerless_full.JPG"  width="500"/>
</p>

## Requirements
* [Intel RealSense SDK 2.0 (v2.44.0)](https://www.intelrealsense.com/sdk-2/)
* OpenCV 4.5.0 (Refer this [link](https://learnopencv.com/install-opencv-on-windows/) for installation)
* Visual Studio 2019
* Windows

## Steps
* Open `markerless_ar_v1.sln`.
* Update the configuration in visual studio and `.props` files accordingly to use OpenCV and librealsense. Refer this [link](https://learnopencv.com/code-opencv-in-visual-studio/) to learn about adding external dependencies to visual studio.
* After the configuration is done, you can connect the [depth camera](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html) and click run in Visual studio.
* Click points in the "Display Image" window. A cube will be overlaid on the clicked point according to the surface normal.
* Two outputs with the virtual cube overlaid will be shown. One uses plane fitting on point clouds (`ppf_match_3d::computeNormalsPC3d`) and the other uses gradient response maps (`rgbd::RgbdNormals`).

## References
* https://dev.intelrealsense.com/docs/code-samples
