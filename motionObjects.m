clc
clear
robot = HANSCUTE();
q = [0 0 0 0 0 0 0];
%load("cyton_q.mat")
%robot.model.teach(q);


%% Setup

qStart = [-1.6 -1.1520 0 -0.6480 0 -1.3680 0];
qEnd = [-0.05 -1.1520 0 -0.6480 0 -1.3680 0];%[0.11 0.1080 0 1.8 0 1.1880 0];
qGrab = [-1.6 -1.1520 0 -0.6480 0 0.2160 pi/2];

robot.model.teach(qStart)
hold on
show = false; % show = true to display the object for object collision size
diameter = 0.25;
height = 0.11;
% section below for defining objects in the workspace
obj(1) = Environment(-0.2, 0.2 ,0, 'plate3.ply', show, diameter, height);
obj(2) = Environment(-0.2, 0.2 ,0.03, 'plate3.ply', show, diameter, height);
obj(3) = Environment(-0.2, 0.2 ,0.06, 'plate3.ply', show, diameter, height);

%% Start Movement
robot.model.animate(qStart);
startPose = robot.model.fkine(qStart);
%startPose = startPose(1:3, 4)';
endPose = robot.model.fkine(qGrab);
%endPose = endPose(1:3, 4)';

[qMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);

%%
for i = 1:5:size(qMatrix, 1)
   robot.model.animate(qMatrix(i,:)); 
end
%%
for i = 1:5:size(velMatrix, 1)
   robot.model.animate(velMatrix(i,:)); 
end