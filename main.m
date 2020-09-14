clc
clf
clear all

workspace = [-1 1 -1 1 -1 1];
scale = 0.25;
qStart = deg2rad([143 0 0 0 0 0 0]);
r = RobotControl();

r.robot.model.plot(qStart,'workspace',workspace,'scale',scale,'nowrist');
hold on;
r.SimulateRobots();

%%

clc
clf
clear all

workspace = [-1 1 -1 1 -1 1];
scale = 0.25;
qStart = deg2rad([143 0 0 0 0 90 0]);
r = RobotControl();

r.robot.model.plot(qStart,'workspace',workspace,'scale',scale,'nowrist');
hold on;
r.robot.model.teach();
