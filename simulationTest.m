clear
clc
robot = HANSCUTE();

q0 = [-0.1427 1.3761 -0.0430 0.7179 -0.0828 1.08 -0.1546];
qStart = [-0.1427 0.9081 -0.0430 0.7179 -0.0828 1.4760 -0.1546];
qSpongeStart = [-0.9771 0.7409 -0.0230 0.6489 -0.2869 1.6521 0.1135];
robot.model.teach(q0);
hold on

show = false; % show = true to display the object for object collision size
diameter = 0.2;
height = 0.11;
% section below for defining objects in the workspace
obj(1) = Environment(0.25, -0.25 , 0.0118, 'plate3.ply', show, diameter, height);
obj(2) = Environment(0.015, -0.15, 0.012, 'brick9.ply', true, 0.05, 0.05);
%%
qCurrent = q0;
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);

%% Time to Pickup Plate
wayPoint = obj(1).pickUpPlate();
startPose = robot.model.fkine(robot.model.getpos); 
endPose = wayPoint(:,:,1);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);

%%
q0 = robot.model.getpos;
q= [];
qMatrix = [];
steps = 100;
for i = 2:size(wayPoint,3)
    [qEnd, err, flag] = robot.model.ikcon(wayPoint(:,:,i), q0)
    s = lspb(0,1,steps);
    for j = 1:steps
        q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    qMatrix = [qMatrix;q];
    q0 = q(end,:);
end

trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%% Drop Plate
obj(3) = Environment(0.15, 0, 0.012, 'brick9.ply', true, 0.05, 0.05);

wayPoint = obj(3).dropPlate(robot);
qReset = [-0.7531 1.2372 0.0452 1.0415 0.0870 -0.6892 0.0219];
%%
q0 = robot.model.getpos;
q= [];
qMatrix = [];
steps = 100;
for i = 1:size(wayPoint,3)
    [qEnd, err, flag] = robot.model.ikcon(wayPoint(:,:,i), q0);
    s = lspb(0,1,steps);
    for j = 1:steps
        q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    qMatrix = [qMatrix;q];
    q0 = q(end,:);
end

trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%% Escape Plate
wayPoint = obj(1).pickUpPlate();
q0 = robot.model.getpos;
q= [];
qMatrix = [];
steps = 100;
for i = size(wayPoint,3):-1:2
    [qEnd, err, flag] = robot.model.ikcon(wayPoint(:,:,i), q0);
    s = lspb(0,1,steps);
    for j = 1:steps
        q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    qMatrix = [qMatrix;q];
    q0 = q(end,:);
end

trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%% Pick Up Sponge
q0 = robot.model.getpos;
qEnd = qSpongeStart;
s = lspb(0,1,steps);
for j = 1:steps
    q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
end
qMatrix = q;
trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%%
wayPoint = obj(2).pickUpSponge(robot);
q0 = robot.model.getpos;
q= [];
qMatrix = [];
steps = 100;
for i = 1:size(wayPoint,3)
    [qEnd, err, flag] = robot.model.ikcon(wayPoint(:,:,i), q0)
    s = lspb(0,1,steps);
    for j = 1:steps
        q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    qMatrix = [qMatrix;q];
    q0 = q(end,:);
end

trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%% Time to Clean
qCurrent = robot.model.getpos;
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qSpongeStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);
%%
qCleanStart = [-0.1381 1.0876 -0.1534 0.7194 -0.2301 0.4218 1.6880];
qCurrent = robot.model.getpos;
startPose = robot.model.fkine(qCurrent);
endPose = obj(3).pose * transl(0.07,0,-0.1);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);
%%
qCurrent = robot.model.getpos;
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qCleanStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);
%%
wayPoint = obj(3).cleanPlate(robot);
q0 = robot.model.getpos;
q = [];
qMatrix = [];
steps = 100;
for i = 1:size(wayPoint,3)
    [qEnd, err, flag] = robot.model.ikcon(wayPoint(:,:,i), q0);
    s = lspb(0,1,steps);
    for j = 1:steps
        q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    qMatrix = [qMatrix;q];
    q0 = q(end,:);
end

trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%% Spin to Clean
rotateEndEffector(robot,pi/2);
rotateEndEffector(robot,-pi/2);
%% Place Sponge Back
qCurrent = robot.model.getpos;
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qSpongeStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);
%%
wayPoint = obj(2).pickUpSponge(robot);
q0 = robot.model.getpos;
q= [];
qMatrix = [];
steps = 100;
for i = 1:size(wayPoint,3)
    [qEnd, err, flag] = robot.model.ikcon(wayPoint(:,:,i), q0)
    s = lspb(0,1,steps);
    for j = 1:steps
        q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    qMatrix = [qMatrix;q];
    q0 = q(end,:);
end

trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%%
qCurrent = robot.model.getpos;
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qSpongeStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);
%% Pick Up Plate to Rack ---- Remove Section Below
wayPoint = obj(1).pickUpPlate();
startPose = robot.model.fkine(robot.model.getpos); 
endPose = wayPoint(:,:,1);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);


q0 = robot.model.getpos;
q= [];
qMatrix = [];
steps = 100;
for i = 2:size(wayPoint,3)
    [qEnd, err, flag] = robot.model.ikcon(wayPoint(:,:,i), q0)
    s = lspb(0,1,steps);
    for j = 1:steps
        q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    qMatrix = [qMatrix;q];
    q0 = q(end,:);
end

trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%% Grab Plate to Rack
wayPoint = obj(3).dropPlate(robot);
q0 = robot.model.getpos;
q= [];
qMatrix = [];
steps = 100;
for i = 1:size(wayPoint,3)
    [qEnd, err, flag] = robot.model.ikcon(wayPoint(:,:,i), q0);
    s = lspb(0,1,steps);
    for j = 1:steps
        q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    qMatrix = [qMatrix;q];
    q0 = q(end,:);
end

trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix);
%% Rotate End Effector
function rotateEndEffector(robot,value)

    q0 = robot.model.getpos;
    q0Pose = robot.model.fkine(q0);
    q0PoseRotate = q0Pose * trotz(value);
    qEnd = robot.model.ikcon(q0PoseRotate, q0);
    steps = 100;
    qMatrix = zeros(100,7);
    s = lspb(0,1,steps);
    for j = 1:steps
        qMatrix(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    
    for i = 1:size(qMatrix,1)
       robot.model.animate(qMatrix(i,:)); 
    end
    
end
%%
function moveRobot(robot, qMatrix, trMatrix)
    epsilon = 0.1;
    W = diag([1 1 1 0.1 0.1 0.1]);
    lambdaMax = 5e-2;
    deltaT = 0.08;              % 0.06 < deltaT < 0.1
    positionError = zeros(3, size(qMatrix,1));
    angleError = zeros(3, size(qMatrix,1));
    T = trMatrix(:,:,1);
    time = 0.8;
    steps = 100;
    deltaT = time/steps; %seconds
    deltaT_msec = deltaT*(1000); %milliSeconds
    deltaT_Nsec = deltaT_msec*(1000000); %nanoSeconds
    
    
    for i = 1:5:size(trMatrix,3)-1
        
        q1 = robot.model.getpos;
        q1 = q1(1:7)
        T = robot.model.fkine(q1);
        deltaX = trMatrix(1:3,4,i+1) - T(1:3,4);
        Rd = trMatrix(1:3,1:3,i+1);
        Ra = T(1:3,1:3);
        Rdot = (Rd-Ra)/deltaT;
        S = Rdot*Ra';
        linear_velocity = deltaX/deltaT;
        angular_velocity = vex(S);
        deltaTheta = tr2rpy(Rd*Ra');
        xdot = W*[linear_velocity;angular_velocity];
        J = robot.model.jacob0(robot.model.getpos);
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon
            lambda = (1 - m(i)/epsilon) * lambdaMax;
        else
            lambda = 0;
        end
        invJ = pinv(J'*J + lambda *eye(7))*J';
        qdot(i,:) = (invJ*xdot)';
        q2 = robot.model.getpos + deltaT * qdot(i,:);
        robot.model.animate(q2);
        
    end
end