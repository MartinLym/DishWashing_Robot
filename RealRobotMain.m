clc
clear
robot = HANSCUTE();
robot.initROSConnection();
arduinoObj = arduino('COM9','Uno','Libraries','Ultrasonic');
ultrasonicObj = ultrasonic(arduinoObj,'D2','D3');

[armControllerPub, armControllerMsg, clawControllerPub, clawControllerMsg] = robot.setupHansPublisher();
qStart = [0 0.7503 -0.0706 0.8442 0.0169 1.5370 -0.069];
qSpongeStart = [-0.9771 0.7409 -0.0230 0.6489 -0.2869 1.6521 0.1135];
qCurrent = robot.getRealRobotJoints();

robot.model.teach(qCurrent);
hold on
show = false; % show = true to display the object for object collision size
diameter = 0.2;
height = 0.11;
% section below for defining objects in the workspace
obj(1) = Environment(0.25, -0.25 , 0.0115, 'plate3.ply', show, diameter, height);
obj(2) = Environment(0.015, -0.15, 0.012, 'brick9.ply', true, 0.05, 0.05);
%%
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qStart); %pointTransform
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%% Time to Pickup Plate
wayPoint = obj(1).pickUpPlate();

qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent); 
endPose = wayPoint(:,:,1);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);

%%
q0 = robot.getRealRobotJoints();
q= [];
qMatrix = [];
steps = 100;
for i = 1:size(wayPoint,3)
    qEnd = robot.model.ikcon(wayPoint(:,:,i), q0);
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
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
robot.moveClaw(-2, clawControllerPub, clawControllerMsg);
%% Drop Plate
obj(3) = Environment(0.15, 0, 0.012, 'brick9.ply', true, 0.05, 0.05);

wayPoint = obj(3).dropPlate(robot);
%%
q0 = robot.getRealRobotJoints();
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
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
robot.moveClaw(0, clawControllerPub, clawControllerMsg);

%% Escape Plate
wayPoint = obj(1).pickUpPlate();
q0 = robot.getRealRobotJoints();
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
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);

%% Pick Up Sponge
q0 = robot.getRealRobotJoints();
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
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
wayPoint = obj(2).pickUpSponge(robot);
q0 = robot.getRealRobotJoints();
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
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
robot.moveClaw(-2, clawControllerPub, clawControllerMsg);

%% Time to Clean
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qSpongeStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
qCleanBringin = [-0.1104 0.9311 -0.0245 0.7148 -0.0813 0.3344 1.6705];
qCurrent = robot.getRealRobotJoints;
startPose = robot.model.fkine(qCurrent);
endPose = obj(3).pose * transl(0.07,0,-0.1);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qCleanBringin);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
wayPoint = obj(3).cleanPlate(robot);
%%
q0 = robot.getRealRobotJoints();
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
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
rotateEndEffector(robot,-pi/2, armControllerPub, armControllerMsg);
rotateEndEffector(robot,pi/2, armControllerPub, armControllerMsg);
rotateEndEffector(robot,-pi/2, armControllerPub, armControllerMsg);
rotateEndEffector(robot,pi/2, armControllerPub, armControllerMsg);
%% Put Sponge Back
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qSpongeStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);

%% Sponge Placed Back
wayPoint = obj(2).pickUpSponge(robot);
q0 = robot.getRealRobotJoints();
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
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
robot.moveClaw(0, clawControllerPub, clawControllerMsg);
%% Grab Plate Again
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qSpongeStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
q0 = robot.getRealRobotJoints();
q= [];
qMatrix = [];
qEnd = [-0.3022 1.0485 0.1917 0.7915 0.4326 1.2855 1.1582];
steps = 100;
s = lspb(0,1,steps);
for j = 1:steps
    q(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
end
qMatrix = [qMatrix;q];
trMatrix = zeros(4, 4, size(qMatrix,1));
for i = 1:size(qMatrix,1)
    trMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
robot.moveClaw(-1, clawControllerPub, clawControllerMsg);

%%
qPlaceStart = [0.2807 0.8606 0.1243 0.7639 0.0399 1.4297 2.5];
qPlace = [0.7931 0.9756 0.0123 0.9373 0.0506 1.2364 2.5];
%%
obj(4) = Environment(0.1, 0.15, 0.012, 'brick9.ply', true, 0.05, 0.05);
%%
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qPlaceStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qPlace);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
robot.moveClaw(0, clawControllerPub, clawControllerMsg);
%%
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qPlaceStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
qCurrent = robot.getRealRobotJoints();
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qStart); %pointTransform
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg);
%%
function rotateEndEffector(robot,value, armControllerPub, armControllerMsg)

    q0 = robot.getRealRobotJoints();
    q0Pose = robot.model.fkine(q0);
    q0PoseRotate = q0Pose * trotz(value);
    qEnd = robot.model.ikcon(q0PoseRotate, q0);
    steps = 100;
    qMatrix = zeros(100,7);
    
    time = 0.7;
    steps = 100;
    deltaT = time/steps; %seconds
    deltaT_msec = deltaT*(1000); %milliSeconds
    deltaT_Nsec = deltaT_msec*(1000000); %nanoSeconds
    secCounter = 0;
    nSecCounter = 0;
    
    s = lspb(0,1,steps);
    for j = 1:steps
        qMatrix(j,:) = (1-s(j)) * q0 + s(j) * qEnd;
    end
    
    for i = 1:size(qMatrix,1)-1
        velocity = (qMatrix(i+1,:) - qMatrix(i,:))/deltaT;
        armControllerMsg.Points.Positions = qMatrix(i,:);
        armControllerMsg.Points.Velocities = velocity;
        armControllerMsg.Points.TimeFromStart.Sec = secCounter;
        armControllerMsg.Points.TimeFromStart.Nsec = nSecCounter;
        nSecCounter = int16(nSecCounter + (deltaT_Nsec/2));
        send(armControllerPub, armControllerMsg);
        pause(0.01);
    end
    
    if nSecCounter >= 1e+09
            nSecCounter = 0;
            secCounter = secCounter + 1;
        end
    
end
%%
function moveRobot(robot, qMatrix, trMatrix, armControllerPub, armControllerMsg)
    epsilon = 0.1;
    W = diag([1 1 1 0.1 0.1 0.1]);
    lambdaMax = 5e-2;
    deltaT = 0.08;              % 0.06 < deltaT < 0.1
    positionError = zeros(3, size(qMatrix,1));
    angleError = zeros(3, size(qMatrix,1));

    T = trMatrix(:,:,1);
    time = 0.7;
    steps = 100;
    deltaT = time/steps; %seconds
    deltaT_msec = deltaT*(1000); %milliSeconds
    deltaT_Nsec = deltaT_msec*(1000000); %nanoSeconds
    pointCounter = 1;
    secCounter = 0;
    nSecCounter = 0;
    
    for i = 1:5:size(trMatrix,3)-1
        q1 = robot.model.getpos;
        q1 = q1(1:7);
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
        
        speedModulator = obtainSpeedModulator();
        velocity = (((q2 - q1)/deltaT) * speedModulator;
        armControllerMsg.Points.Positions = q2;
        armControllerMsg.Points.Velocities = velocity;
        armControllerMsg.Points.TimeFromStart.Sec = secCounter;
        armControllerMsg.Points.TimeFromStart.Nsec = nSecCounter;
        nSecCounter = int16(nSecCounter + (deltaT_Nsec/2));
        send(armControllerPub, armControllerMsg);
        
        if nSecCounter >= 1e+09
            nSecCounter = 0;
            secCounter = secCounter + 1;
        end
    end
end

function speedModulator = obtainSpeedModulator()
    maxDistance = 0.5;
    distance = readDistance(ultrasonicObj);
    if distance > 0.5
        distance = 0.5;
    elseif distance < 0.2
        distance = 0.2;        
    end
    speedModulator = distance/maxDistance;
end