clc
clear
robot = HANSCUTE();
%% Setup

qStart = [-1.9 0.1080 0 1.8 0 1.1880 0];
qEnd = [-1.35 -0.4680 0 1.8 0.0360 1.8 0];%[0.11 0.1080 0 1.8 0 1.1880 0];

robot.model.teach(qStart)
hold on
show = false; % show = true to display the object for object collision size
diameter = 0.25;
height = 0.11;
% section below for defining objects in the workspace
obj(1) = Environment(0.2, -0.2 ,0, 'plate3.ply', show, diameter, height);
obj(2) = Environment(0.2, -0.2 ,0.03, 'plate3.ply', show, diameter, height);
obj(3) = Environment(0.2, -0.2 ,0.06, 'plate3.ply', show, diameter, height);

%% Start Movement
robot.model.animate(qStart);
startPose = robot.model.fkine(qStart);
%startPose = startPose(1:3, 4)';
endPose = robot.model.fkine(qEnd);
%endPose = endPose(1:3, 4)';

robot.moveCuteRobot(startPose, endPose, 1500, obj);