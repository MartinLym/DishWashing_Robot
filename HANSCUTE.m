classdef HANSCUTE < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;
        pointCloud;
        startPose;
        qStart;
        endPose;
        qEnd;
    end
    
    methods
        function self = HANSCUTE()
            self.getCuteRobot();
            self.initCuteRobot();
        end
        
        function getCuteRobot(self)
            pause(0.001);
            name = ['bob'];
            qlimH = [-2.5, 2.5];
            qlimV = [-1.8, 1.8];
            
            L1 = Link('d', 0.0872,'a', 0,'alpha', -pi/2,'offset', 0,'qlim', qlimH);
            L2 = Link('d', 0,'a', 0,'alpha', pi/2,'offset', 0,'qlim', qlimV); %qlimV
            L3 = Link('d', 0.0768,'a', 0,'alpha', -pi/2,'offset',0,'qlim', qlimH); %qlimH
            L4 = Link('d', 0,'a', 0.0488,'alpha', -pi/2,'offset', -pi/2,'qlim', qlimV);
            L5 = Link('d', 0,'a', 0.0663,'alpha', pi/2,'offset', 0,'qlim', qlimV);
            L6 = Link('d', 0,'a', 0,'alpha', pi/2,'offset', pi/2,'qlim',qlimV); %qlimV
            L7 = Link('d', 0.055,'a', 0,'alpha', 0,'offset', pi/2,'qlim', qlimH); %rotate offset by -pi/2
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
        end
        
        function initCuteRobot(self) %Used to identify the workspace of the robot
            stepRads = deg2rad(60);
            qlimH = [-2.5, 2.5];
            qlimV = [-1.8, 1.8];
            qlim = [qlimH(1), qlimH(2);
                     qlimV(1), qlimV(2);
                     qlimH(1), qlimH(2);
                     qlimV(1), qlimV(2);
                     qlimV(1), qlimV(2);
                     qlimV(1), qlimV(2);
                     qlimH(1), qlimH(2)];
                 
            pointCloudSizeCute = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1));
            self.pointCloud = zeros(pointCloudSizeCute,3);
            counter = 1;
            tic
            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    % Don't need to worry about joint 6, just assume it=0
                                    q7 = 0;
                                    %                     for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q = [q1,q2,q3,q4,q5,q6,q7];
                                    tr = self.model.fkine(q);
                                    self.pointCloud(counter,:) = tr(1:3,4)';
                                    if self.pointCloud(counter,3) < 0
                                       self.pointCloud(counter,3) = 0; 
                                    end
                                    counter = counter + 1;
                                    if mod(counter/pointCloudSizeCute * 100,1) == 0
                                        display(['HANSCUTE: After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudSizeCute * 100),'% of poses']);
                                    end
                                    %                     end
                                end
                            end
                        end
                    end
                end
            end
            %plot3(self.pointCloud(:,1), self.pointCloud(:,2), self.pointCloud(:,3), 'r.')
            
        end
        
        function [qMatrix, sendQMatrix, velMatrix, trMatrix, poseMatrix, coordMatrix, positionError, angleError, m] = obtainMotionMatrices(self, startPose, endPose, numNodes, obj) % Uses RRT* to avoid inputted objects
            % Grabs start pose and end pose for RRT* path planning
            self.startPose = startPose
            self.endPose = endPose;
            self.qStart = self.model.ikcon(startPose, self.model.getpos);
            self.qEnd = self.model.ikcon(endPose, self.model.getpos);
            
            startCoord = self.startPose(1:3, 4)';
            endCoord = self.endPose(1:3, 4)';
            q_conf.coord = startCoord;
            q_conf.cost = 0;
            q_conf.parent = 0;
            q_goal.coord = endCoord;
            q_goal.cost = 0;
            stepSize = 0.03; % Placement of next node radius size
            
            node(1) = q_conf; 
            numOfObj = numel(obj); % used to iterate through the number of objects
            
            % Section below for the bounds of the rrt* points within the
            % workspace
            x_min = 0;                         % Changed to 0 from min(self.pointCloud(:,1)) this would confine the workspace of RRT
            x_max = max(self.pointCloud(:,1));
            y_min = min(self.pointCloud(:,2));
            y_max = max(self.pointCloud(:,2));
            z_min = min(self.pointCloud(:,3));
            %display(z_min)
            z_max = max(self.pointCloud(:,3));
            
            for i = 1:1:numNodes
                [q_near, q_rand, val] = getNearNode(node, x_min, x_max, y_min, y_max, z_min, z_max);
                q_new.coord = steer3d(q_rand, q_near.coord, val, stepSize); % obtains a new node
                
                q1Node = q_near.coord;
                q2Node = q_new.coord;
                
                % checks if the new node is colliding with any objects, if
                % safe = 1, no collision in environment 
                % safe = 0, there is collision and will retrieve new node
                % from the beginning of for loop
                safe = checkCollision(obj, numOfObj, q1Node, q2Node); 
                
                if safe == 1
                   q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
                   
                   % section below checks the new node for any neighbour
                   % nodes to connect to
                   q_nearest = [];
                   r = 0.05; % <----- radius to search for nearest neighbour nodes
                   neighbor_count = 1;
                   for j = 1:1:length(node)
                       if (dist_3d(node(j).coord, q_new.coord)) <= r
                           q_nearest(neighbor_count).coord = node(j).coord;
                           q_nearest(neighbor_count).cost = node(j).cost;
                           neighbor_count = neighbor_count+1;
                       end
                   end
                   
                   q_min = q_near;
                   C_min = q_new.cost;
                   % Iterates through the nearest neighbour nodes and finds
                   % the closest one
                   for k = 1:1:length(q_nearest)
                       if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min
                           q_min = q_nearest(k);
                           C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
                           %line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');
                           hold on
                       end
                   end
                   
                   for j = 1:1:length(node)
                       if node(j).coord == q_min.coord
                           q_new.parent = j;
                       end
                   end
                   
                   node = [node q_new];
                   
                end
            end
            
            % coordMatrix layout [x y z]
            [coordMatrix, numWayPoints] = generatePath(node, q_goal); % generates the shortest path through the tree
            
            % Section below is for RMRC
            [qMatrix, trMatrix, poseMatrix] = obtainPoseJointMatrices(self, coordMatrix, numWayPoints);
            [sendQMatrix, velMatrix, positionError, angleError, m] = obtainVelocityMatrix(self, qMatrix, trMatrix);
                        
        end
        
        function [qMatrix, trMatrix, poseMatrix] = obtainPoseJointMatrices(self, coordMatrix, numWayPoints) 
                steps = 100;                
                q = [];
                s = lspb(0,1,steps); 
                
                for j = 1:steps
                    q(j,:) = (1-s(j))*self.qStart + s(j)*self.qEnd;
                end
                % Obtain the rotation matrix to change as it moves through
                % the trajectory
                idxCounter = ceil((size(q,1)/numWayPoints));
                idxMatrix = [];
                for i = 1:idxCounter:size(q,1)
                    idxMatrix = [idxMatrix i];
                end
                idxMatrix = [idxMatrix size(q,1)];
                poseMatrix = zeros(4,4,size(idxMatrix,2));
                poseMatrix(:,:,1) = self.startPose;
                
                % Concatenate the rotation matrix with the position vector
                % from coordMatrix to also avoid objects
                for k = 2:size(idxMatrix,2)
                    poseMatrix(:,:,k) = self.model.fkine(q(idxMatrix(k),:));
                    poseMatrix(:,4,k) = [coordMatrix(k-1,:)';1];
                end
                
                % Obtain the total joint configuration to prepare for
                % velocity matrix
                qMatrix = [];
                qChange = self.qStart;
                for i = 1:size(idxMatrix,2)
                    qStart = qChange;
                    q = nan(steps,7);
                    [qEnd, err, exitFlag] = self.model.ikcon(poseMatrix(:,:,i), qStart);
                    s = lspb(0,1,steps); 
                    
                    for j = 1:steps
                        q(j,:) = (1-s(j))*qStart + s(j)*qEnd;
                    end
                    
                    qMatrix = [qMatrix; q];
                    qChange = qEnd;
                end
                
                % Obtain the total transforms from trajectory to find
                % velocity matrix
                trMatrix = zeros(4,4,size(qMatrix, 1));
                for i = 1:size(qMatrix, 1)
                    trMatrix(:,:,i) = self.model.fkine(qMatrix(i,:));
                end
        end
        
        function [sendQMatrix, velMatrix, positionError, angleError, m] = obtainVelocityMatrix(self,qMatrix, trMatrix)
            T = trMatrix(:,:,1);
            q0 = self.qStart;
            
            epsilon = 0.1;
            W = diag([1 1 1 0.1 0.1 0.1]);
            lambdaMax = 5e-2;
            deltaT = 0.08;              % 0.06 < deltaT < 0.1
            positionError = zeros(3, size(qMatrix,1));
            angleError = zeros(3, size(qMatrix,1));
            
            sendQMatrix = zeros(size(qMatrix,1), 7);
            velMatrix = zeros(size(qMatrix,1), 7);
            sendQMatrix(1,:) = self.model.ikcon(T,q0);
            
            for i = 1:size(qMatrix,1)-1
                T = self.model.fkine(sendQMatrix(i,:));
                deltaX = trMatrix(1:3,4,i+1) - T(1:3,4);    % Calculates the position error
                Rd = trMatrix(1:3,1:3,i+1);
                Ra = T(1:3,1:3);
                Rdot = (Rd-Ra)/deltaT;
                S = Rdot*Ra';
                
                linear_velocity = deltaX/deltaT;
                angular_velocity = vex(S); %[S(3,2);S(1,3);S(2,1)]; 
                deltaTheta = tr2rpy(Rd*Ra');
                xdot = W*[linear_velocity;angular_velocity]; %nu
                J = self.model.jacob0(sendQMatrix(i,:));
                m(i) = sqrt(det(J*J'));
                
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*lambdaMax;
                else
                    lambda = 0;
                end
                invJ = pinv(J'*J + lambda *eye(7))*J';
                qdot(i,:) = (invJ*xdot)'; 
                
                for j = 1:7
                   if sendQMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)
                       qdot(i,j) = 0;
                   elseif sendQMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)
                       qdot(i,j) = 0;
                   end                    
                end
                
                sendQMatrix(i+1,:) = sendQMatrix(i,:) + deltaT * qdot(i,:);
                velMatrix(i,:) = (sendQMatrix(i+1,:) - sendQMatrix(i,:))/deltaT;
                positionError(:,i) = deltaX;
                angleError(:,i) = deltaTheta;
            end
            velMatrix(end,:) = velMatrix(end-1,:);
        end
        
    end
    
    methods (Static)
        function initROSConnection()
           rosshutdown
           ipHANS = 'http://10.42.0.1:11311';
           rosinit(ipHANS);
        end
        
        function currentJoint = obtainCurrentJointStates()
            jointStateTopic = '/joint_states';
            jointStateMsgType = 'sensor_msgs/JointState';
            jointStateSub = rossubscriber(jointStateTopic, jointStateMsgType);
            receive(jointStateSub,10);
            currentJoint = jointStateSub.LatestMessage.Position';
        end
        
        function clawStateSub = obtainCurrentClawStateSub()
           clawStateTopic = '/claw_controller/state';
           clawStateMsgType = 'dynamixel_msgs/JointState';
           clawStateSub = rossubscriber(clawStateTopic, clawStateMsgType);
        end
        
        function [armControllerPub, armControllerMsg, clawControllerPub, clawControllerMsg] = setupHansPublisher()
           jointStateNames = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"];
           armControllerTopic = 'cute_arm_controller/command';
           armControllerMsgType = 'trajectory_msgs/JointTrajectory';
           armControllerMsg = rosmessage(armControllerMsgType);
           armControllerMsg.JointNames = jointStateNames;
           armControllerPub = rospublisher(armControllerTopic, armControllerMsgType);
           steps = 1;
           
           %for i = 1:steps
               armControllerTrajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
               armControllerMsg.Points = [armControllerMsg.Points; armControllerTrajPoint];
           %end
           
           clawControllerTopic = '/claw_controller/command';
           clawControllerMsgType = 'std_msgs/Float64';
           clawControllerMsg = rosmessage(clawControllerMsgType);
           clawControllerPub = rospublisher(clawControllerTopic, clawControllerMsgType);
           
        end
        
        function moveClaw(val, clawControllerPub, clawControllerMsg)
            clawControllerMsg.Data = val;
            send(clawControllerPub,clawControllerMsg);
        end
        
        function moveArm(qMatrix, armControllerPub, armControllerMsg)
            time = 2;
            steps = 100;
            deltaT = time/steps; %seconds
            deltaT_msec = deltaT*(1000); %milliSeconds
            deltaT_Nsec = deltaT_msec*(1000000); %nanoSeconds
            pointCounter = 1;
            secCounter = 0;
            nSecCounter = 0;
            
            for i = 1:size(qMatrix,1)
                if i == size(qMatrix, 1)
                    break;
                end
                
                velocityMatrix(i,:) = (qMatrix(i+1,:) - qMatrix(i,:))/deltaT;
                armControllerMsg.Points(pointCounter,1).Positions = qMatrix(i, :);
                armControllerMsg.Points(pointCounter,1).Velocities = velocityMatrix(i, :);
                armControllerMsg.Points(pointCounter,1).TimeFromStart.Sec = secCounter;
                armControllerMsg.Points(pointCounter,1).TimeFromStart.Nsec = nSecCounter;
                pointCounter = mod(i,steps+1);
                nSecCounter = nSecCounter + deltaT_Nsec;
                
                if nSecCounter >= 1e+09
                    nSecCounter = 0;
                    secCounter = secCounter + 1;
                end
                
                if pointCounter == 0
                    send(armControllerPub, armControllerMsg);
                    armControllerSub = rossubscriber('cute_arm_controller/command');
                    %receive(armControllerSub,20);
                    pointCounter = 1;
                    pause(time);
                    display('100 steps')
                end
            end
        end
        
    end
end

function safe = checkCollision(obj, numOfObj, q1Node, q2Node)
    for i = 1:1:numOfObj
        faces = obj(i).objEnvironment.faces;
        vertex = obj(i).objEnvironment.vertex;
        faceNormals = obj(i).objEnvironment.faceNormals;
        
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectionNode, checkNode] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane, q1Node, q2Node);
            if checkNode == 1 && IsIntersectionPointInsideTriangle(intersectionNode,vertex(faces(faceIndex,:)',:))
                plot3(intersectionNode(1),intersectionNode(2),intersectionNode(3), 'r*', 'MarkerSize', 10);
                %display([i,intersectionNode(1),intersectionNode(2),intersectionNode(3)], 'Intersection');
                safe = 0;
                break
            end
            safe = 1;
        end
        
    end
end

function [q_near, q_rand, val] = getNearNode(node, x_min, x_max, y_min, y_max, z_min, z_max) 
    % section below chooses a random point within the bounds
    xRand = (x_max - x_min).*rand(1);%x_min + (x_max + x_max) * rand(1);
    yRand = y_min + (y_max + y_max) * rand(1);
    zRand = (z_max - z_min).*rand(1);
    q_rand = [xRand yRand zRand];
    %plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  'black')
    
    ndist = [];
    for i = 1:1:length(node)
        n = node(i);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = node(idx);
end

function [coordMatrix, counter] = generatePath(node, q_goal)
    D = [];
    for j = 1:1:length(node)
        tmpdist = dist_3d(node(j).coord, q_goal.coord);
        D = [D tmpdist];
    end
    % Search backwards from goal to start to find the optimal least cost path
    [val, idx] = min(D);
    q_final = node(idx);
    q_goal.parent = idx;
    q_end = q_goal;
    node = [node q_goal];
    counter = 0;
    coordMatrix = [];
    while q_end.parent ~= 0
        start = q_end.parent;
        coordMatrix = [q_end.coord; coordMatrix];
        line([q_end.coord(1), node(start).coord(1)], [q_end.coord(2), node(start).coord(2)], [q_end.coord(3), node(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
        hold on
        q_end = node(start);
        counter = counter + 1;
    end
end
