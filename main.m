clc
clear
robot = HANSCUTE();
robot.initROSConnection();

[armControllerPub, armControllerMsg, clawControllerPub, clawControllerMsg] = robot.setupHansPublisher();
%q0 = [-1.1382 0.3636 -0.8299 -0.5062 -0.9986 1.1658 0];
%q = [0 0 0 0 0 0 0];
qStart = [-0.1427 0.9081 -0.0430 0.7179 -0.0828 1.4404 -0.0077];
qGrab = [0.0227 1.4494 -0.0276 0.7333 -0.0874 -0.6169 -0.1546];
qCurrent = robot.obtainCurrentJointStates
qCurrent =qCurrent(1:7);

robot.model.teach(qCurrent);
hold on
show = false; % show = true to display the object for object collision size
diameter = 0.25;
height = 0.11;
% section below for defining objects in the workspace
obj(1) = Environment(0.2, -0.2 ,0, 'plate3.ply', show, diameter, height);
%%
qCurrent = robot.obtainCurrentJointStates
qCurrent =qCurrent(1:7);
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
robot.moveArm(sendQMatrix, armControllerPub, armControllerMsg);

%%
qCurrent = robot.obtainCurrentJointStates
qCurrent =qCurrent(1:7);
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qGrab);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
robot.moveArm(sendQMatrix, armControllerPub, armControllerMsg);