clear
clc
robot = HANSCUTE();

q0 = [-0.1427 1.3761 -0.0430 0.7179 -0.0828 1.08 -0.1546];
qStart = [-0.1427 0.9081 -0.0430 0.7179 -0.0828 1.4760 -0.1546];
robot.model.teach(q0);
hold on

show = false; % show = true to display the object for object collision size
diameter = 0.2;
height = 0.11;
% section below for defining objects in the workspace
obj(1) = Environment(0.25, -0.25,0.05, 'plate3.ply', show, diameter, height);
%%
qCurrent = q0;
startPose = robot.model.fkine(qCurrent);
endPose = robot.model.fkine(qStart);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);

%%
wayPoint = obj(1).pickUpPlate();
for i = 1:size(wayPoint,3)
    trplot(wayPoint(:,:,i), 'frame', i, 'color', 'b', 'length',0.1);
    P = mkgrid(2,0.02,wayPoint(i));
    plot_sphere(P, 0.0075, 'b');
end
%%
startPose = robot.model.fkine(robot.model.getpos); 
endPose = wayPoint(:,:,1); %pointTransform
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);

%%
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
wayPoint = obj(1).dropPlate(robot);
trplot(wayPoint(:,:,1), 'frame', i, 'color', 'r', 'length',0.1);

%%
qCurrent = robot.model.getpos;
startPose = robot.model.fkine(qCurrent);
endPose = wayPoint(:,:,1);
[qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);
moveRobot(robot, qMatrix, trMatrix);

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
        angular_velocity = vex(S)
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
        q2 = robot.model.getpos + deltaT * qdot(i,:)
        robot.model.animate(q2);
        
    end
end