clc
clf
clear all

workspace = [-1 1 -1 1 -1 1];
scale = 0.25;
qStart = deg2rad([-90,0,180,26.8,0,63.2,0]);

r = RobotControl();
r.robot.model.plot(qStart,'workspace',workspace,'scale',scale,'nowrist');
hold on;
r.SimulateRobot();

%%
clc
clf
clear all

workspace = [-1 1 -1 1 -1 1];
scale = 0.25;
qStart = deg2rad([-90,0,180,26.8,0,63.2,0]);
r = RobotControl();

r.robot.model.plot(qStart,'workspace',workspace,'scale',scale,'nowrist');
hold on;
r.robot.model.teach();


