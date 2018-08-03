# Color Tracker


## How to Run it
  1. Follow the instructions on starting optical flow
     ```bash
     sudo systemctl start aero-teraranger.service
     sudo systemctl start aero-optical-flow
     ```
  2. Run `roslaunch realsense_camera r200_nodelet_rgbd.launch` to start publishing the forwad-facing camera
  3. Run `detector.py` followed by `tracker.py`
     ```bash
     python detector.py
     python tracker.py
     ```
     - NOTE: you can run the detector with `python detector.py [min_red] [min_green] [min_blue] [max_red] [max_green] [max_blue]` substituting the [values] with custom threshold values
  4. Follow drone deployment protocol
  5. Takeoff drone and put it in position mode
  5. Hold target object in front of camera and switch to offboard mode
  6. Slowly move

## Robotless Testing
If you like you can test the detector on your laptop with a pre-recorded video tracking a red backpack, use the following instructions:
  1. Run `roscore` on your laptop
  2. Run `detector.py` followed by `image_feeder.py`
     ```bash
     python detector.py
     python image_feeder.py
     ```
  3. To see rostopic contents:
    - Start `rqt_image_view` for image feeds and switch to feed `/color_target/img`
    - Run `rostopic echo /color_target/pos` to see position values
