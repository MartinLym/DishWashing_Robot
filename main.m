clc
clf
clear all

r = RobotControl();

%%
r.SimulateRobot();
%%
r.AnimateCollisionAvoidance();
%%
r.RetreatVS();
%%
clc
clf
clear all

r = RobotControl();
r.robot.model.teach();

