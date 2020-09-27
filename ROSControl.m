masterURI = "http://marty-Surface-Pro:11311";
ip = "192.168.0.6";

rosshutdown
setenv('ROS_MASTER_URI',masterURI);
setenv('ROS_IP',ip);
rosinit
%% Current Status of Joints
jointStateTopic = '/joint_states';
jointStateMsgType = 'sensor_msgs/JointState';
jointStateSub = rossubscriber(jointStateTopic, jointStateMsgType);

pause(1);
jointStateNames = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"];

%%  Arm Controller Publisher Setup
armControllerTopic = 'cute_arm_controller/command';
armControllerMsgType = 'trajectory_msgs/JointTrajectory';
armControllerMsg = rosmessage(armControllerMsgType);
armControllerMsg.JointNames = jointStateNames;
armControllerPub = rospublisher(armControllerTopic, armControllerMsgType);
steps = 100;
for i = 1:steps
    armControllerTrajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    armControllerMsg.Points = [armControllerMsg.Points; armControllerTrajPoint];
end

%% Claw Controller Setup
clawControllerTopic = '/claw_controller/command';
clawControllerMsgType = 'std_msgs/Float64';
clawControllerMsg = rosmessage(clawControllerMsgType);
clawControllerPub = rospublisher(clawControllerTopic, clawControllerMsgType);

%%
clawControllerMsg.Data = 0;
send(clawControllerPub,clawControllerMsg);
clawControllerSub = rossubscriber(clawControllerTopic);
receive(clawControllerSub, 10);
%% Setup Environment
robot = HANSCUTE();
show = false;
diameter = 0.25;
height = 0.11;
obj(1) = Environment(-0.2, 0.2 ,0, 'plate3.ply', show, diameter, height);

currentq = jointStateSub.LatestMessage.Position';
currentqArm = currentq(:, 3:9);
currentqClaw = currentq(:, 1:2);
robot.model.teach(currentqArm);
%% Obtain Motion Joint Matrices
q0 = zeros(1,7);
qStart = [-1.6 -1.1520 0 -0.6480 0 0.2160 0];
startPose = robot.model.fkine(currentqArm);
endPose = robot.model.fkine(qStart);

[qMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix] = robot.obtainMotionMatrices(startPose, endPose, 1500, obj);

%% Move in MATLAB Simulation
for j = 1:5:size(qMatrix, 1)
    robot.model.animate(qMatrix(j,:));
end

%%
time = 2.0;
velocityMatrix = zeros(size(qMatrix,1),7);
deltaT = time/steps;
deltaT_msec = deltaT*(1000);
testq = zeros(steps,7);
pointCounter = 1;
%qMatrix = [qMatrix; qMatrix(end,:)];

for i = 1:size(qMatrix,1)
    if i == size(qMatrix, 1)
       break; 
    end
    
    velocityMatrix(i,:) = (qMatrix(i+1,:) - qMatrix(i,:))/deltaT;
    
    armControllerMsg.Points(pointCounter,1).Positions = qMatrix(i, :);
    armControllerMsg.Points(pointCounter,1).Velocities = velocityMatrix(i, :);
    armControllerMsg.Points(pointCounter,1).TimeFromStart.Nsec = round(deltaT_msec*(i-1));
    %testq(pointCounter,:) = qMatrix(i, :);
    pointCounter = mod(i,steps+1);
    if pointCounter == 0
%         for j = 1:5:size(testq, 1)
%            robot.model.animate(testq(j,:)); 
%         end
        send(armControllerPub, armControllerMsg);
        armControllerSub = rossubscriber(armControllerTopic);
        receive(armControllerSub,10);
        
        pointCounter = 1;
    end
end
display('FINISHED!!!!')

