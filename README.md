# DishWashing_Robot

** I don't know why my commits come up as Byron. But it is me that committed. Maybe because I'm using Byron's computer to commit. - Jenn

WHAT'S LEFT
- real robot
- stop due to user input
- COMMENTS!!!!

VIDEO CHECKLIST
1. real robot - ROS blah blah 
2. simulation with environment - trajectory, ikcon
3. collision - RRT, lineplaneintersection
4. GUI - joint movements, stop
5. visual servoing
6. intro and conclusion

----------------------------------------------------------------------

Assignment checklist

- WEEK 9 - SWMS ✓
- WEEK 11 - 1 min video ✓
- WEEK 12 - demo + video 

- dishwashing robot arm movement - jenn ✓

- insert real robot model - jenn ✓

- 1 GUI - Jenn
  - Teach Toolbox (move each joint individually) ✓
  - x,y,z end effector has to move there ✓
  
  2 E-STOP, must resume task after resuming - jenn ✓

- 4 active avoidance collision, visual servoing RMRC - Martin
  - a) react to an asynchronous stop signal by a user
  - b) robot will stop until there is no predicted collision or move to avoid the collision
       Using RRT to do path planning to avoid collision
        Current Issues:
            - Nodes generated are not in the workspace of the robot ✓-ISH
            - Nodes require a check collision function ✓
            - Path determined isn't the shortest path ✓-ISH
  - c) robot retreat from a simulated safety symbol using visual servoing and RMRC ✓ jenn
  
- 3 Environment - jenn
  - plate✓
  - sponge✓
  - sink✓
  - barrier
  - estop✓
  - light curtains

- integrate, clean up code, add comments - jenn

