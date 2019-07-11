# How to connect to the drone 
1. Start up qGroundControl and the drone.
2. In qGroundControl, under `Connections` (hit the purple Q icon in the upper left),click on the connection for that drone and hit connect.
    * To connect for the first time, add a new connection with type TCP, port 5760, and IP address `[drone name].beaver.works` and name it with the drone name.
    * If you are reconnecting, you may need to re-enter the IP address, but the other settings will not change.

Now you can fly the drone in manual mode. However, to be able to go into position control mode, you need to start up teraranger, roscore, and aero-optical-flow:
1. Do `ssh -Y uav@[drone name].beaver.works` to ssh into it.
2. If that fails, you need to run:
    1. Start distance sensor (teraranger): `sudo systemctl start aero-teraranger.service`
    2. Start `roscore`
    3. In a new terminal, launch optical flow with: `sudo -E ~/bwsi-uav/catkin_ws/src/aero-optical-flow/build/aero-optical-flow`
3. Switch into position mode using the switch on the remote (middle position).
    * If the drone switches into altitude mode instead, that means it does not know its x-y position. Stop optical flow and restart it.
    * If drone continuously does not switch into position mode, you probably need to check the xacc, yacc, and zacc, under the analyze option in QGround, to see if you need to recalibrate
    * If the drone switches into stabilized mode, that means it does not know its altitude. Only solution is to restart the drone.

## Known errors/bugs
+ *Check potential error for both types of launch*, run `sudo du -sh /var/log/` several times. If storage is increasing visibly, stop teraranger with `sudo systemctl stop aero-teraranger.service`, clear the log with `sudo truncate -s 0 /var/log/syslog*`, and restart teraranger
### How to deal with EKF errors
Sometimes the drone will refuse to arm, complaining about EKF errors.
 * If you are lucky and the drone tells you which sensor is causing the problem (ex. `EKF YAW ERROR`), recalibrate that sensor (look under Settings > Sensors in qGroundControl).
 * If the drone merely complains `EKF INTERNAL CHECKS`, then recalibrate the level horizon and hope. If it still persists, try recalibrating the rest of the sensors and restarting. There are no consistent fixes for this error.