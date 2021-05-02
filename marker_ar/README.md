# ArUco Marker Tracking

Overlay virtual objects on ArUco markers using OpenCV ArUco marker tracking functions and Unity.

![Marker - Man animation](../images/aruco_man.gif)

## Requirements
* Unity 2019.4.21f1
* OpenCV 4.5.0 (You can refer [this](https://learnopencv.com/install-opencv-on-windows/) for installation.)
* ipython notebook
* Windows 10 (It should mostly work for Linux as well)

## Steps
* Generate custom ArUco marker boards using `python/create_aruco_board.ipynb`. You can also use `opencv/create_board.cpp` for generating default ArUco marker boards.
* **Camera calibration**:
  * Camera can be calibrated using `opencv/calibrate_camera.cpp` with the generated marker boards.
  * Copy the generated `calib.txt` to `aruco_unity` dir.
* Compile `opencv/aruco_unity.cpp` as a dll and copy it to `aruco_unity/Assets/Plugins/`. This step can be ignored, as I have already added the necessary dll's.
* Now open `aruco_unity/Assets/Scenes/Scene.unity` and update the intrinsic parameters of camera in the `Main Camera` object.
* Update the marker size and aruco dictionary parameters, if you are not using the same configuration.
* Finally, hit play in Unity.

## Troubleshooting
* If you are not able to detect markers, check the rejected markers by running `opencv/aruco_unity.cpp` independently. Try flipping the image.
* If `fx` is not close to `fy`, try using the physical camera option in Unity.

## References
* https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
* [HoloLensArucoTracking](https://github.com/KeyMaster-/HoloLensArucoTracking)
