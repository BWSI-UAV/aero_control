# Obstacle avoidance challenge
This challenge will get you flying over and under 'obstacles' (AR tags).

The big design decision you need to make is how to remember when to return to your default height. Once you are flying over an obstacle, you will not be able to see it! Also think about how to integrate this with a line follower, which will also be issuing velocity commands, for the final challenge.

## Challenge
 - Start >1 m away from an AR tag and fly to the default height of 0.75 m
 - Fly forward until the AR tag is <= 1 m away from the drone
 - Based on AR tag height, fly over/under the AR tag 'obstacle' and return to the default height on the other side
 - Resume going forward (but pilot-in-command needs to take over before the drone hits the net).

#### Advanced Challenge
Integrate this code with your line follower so, instead of simply going forward at a set speed, you follow the line.

### Code steps:
We recommend you start by copying code (and a launch file) from your open loop controller.

 1. Cruise forward until you are close enough to an AR tag, using P control to control your height
    * Send x velocities, y velocities and a height setpoint to the streaming thread
    * In the streaming thread, calculate z velocities using current height and height setpoint. Use a proportional controller.
 2. Calculate the AR tag's height using the drone's height (accessible in `/mavros/local_position/pose`) and the height of the AR tag relative to the drone.
 3. Decide based on the height of the drone whether to go up or down.
    * Just change the height setpoint
 4. After a certain length of time, return to the default height
    * Use rospy.sleep(dur), NOT time.sleep(dur)
    * Reset the height setpoint to the default height

## Checkpoints

### Height control
Demonstrate the drone automatically controlling its height.
 * Graph the commanded z velocity with rqt_plot and show a TA before you fly
 * Hover the drone low over the ground and show it rises/lowers to 0.75 m
### Over/Under logic
Show the drone an AR tag and based on the AR tag's height the drone will decide whether to go over or under the AR tag
 * Make sure you are holding the drone over the ground so it knows its height off the ground
### State transitions
Do after completing Over/Under logic checkpoint. Demonstrate that the drone changes its height setpoint after detecting the AR tag to avoid the obstacle and, after a period of time, revert to the default height.
