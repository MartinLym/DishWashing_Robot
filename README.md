# DishWashing_Robot

Assignment checklist

- WEEK 9 - SWMS
- WEEK 11 - 1 min video
- WEEK 12 - demo + video

- make classes - jenn ✓
- dishwashing robot arm movement - jenn ✓
- find dh para for robot - martin ✓
- insert real robot model - martin

- 1 GUI - Jenn
  - Teach Toolbox (move each joint individually)
  - x,y,z end effector has to move there
  
  2 E-STOP, must resume task after resuming - jenn

- 4 active avoidance collision, visual servoing RMRC - Martin
  - a) react to an asynchronous stop signal by a user
  - b) robot will stop until there is no predicted collision or move to avoid the collision
       Using RRT to do path planning to avoid collision
        Current Issues:
            - Nodes generated are not in the workspace of the robot ✓
            - Nodes require a check collision function ✓
            - Path determined isn't the shortest path ✓-ISH, no priority just requires optimisation
  - c) robot retreat from a simulated safety symbol using visual servoing and RMRC ✓, visual servoing 
  
- 3 Environment - jenn & martin
  - plate✓
  - sponge✓
  - sink
  - plate stand
  - sponge(bubbles can appear)
  - barrier
  - estop
  - active sensors would stop the robot
  - other safety stuff
