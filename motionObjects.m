clc
clear
%% Setup
robot = HANSCUTE();
qStart = [-1.9 0.1080 0 1.8 0 1.1880 0];
qEnd = [0.11 0.1080 0 1.8 0 1.1880 0];

robot.model.teach(qStart)
hold on
show = false;
diameter = 0.25;
height = 0.11;
obj(1) = Environment(0.2, -0.2 ,0, 'plate.ply', show, diameter, height);
obj(2) = Environment(0.2, -0.2 ,0.03, 'plate.ply', show, diameter, height);
obj(3) = Environment(0.2, -0.2 ,0.06, 'plate.ply', show, diameter, height);

%% Start Movement
startCoord = robot.model.fkine(qStart);
startCoord = startCoord(1:3, 4)';
endCoord = robot.model.fkine(qEnd);
endCoord = endCoord(1:3, 4)';

robot.moveCuteRobot(startCoord, endCoord, 1500, obj)