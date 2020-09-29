clc
clf
clear all

workspace = [-1 1 -1 1 -1 1];
scale = 0.25;
qStart = deg2rad([-81,0,-143,30.9,47.4,49,0]);

r = RobotControl();
r.robot.model.plot(qStart,'workspace',workspace,'scale',scale,'nowrist');
axis equal;
%r.SimulateRobot();

%%
clc
clf
clear all

workspace = [-1 1 -1 1 -1 1];
scale = 0.25;
qStart = deg2rad([-81,0,-143,30.9,47.4,49,0]);
r = RobotControl();

r.robot.model.plot(qStart,'workspace',workspace,'scale',scale,'nowrist');
hold on;
r.robot.model.teach();


