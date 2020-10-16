classdef HansCute < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;
        pointCloud;
        startPose;
        qStart;
        endPose;
        qEnd;
        
        workspace = [-1 1 -1 1 -1 1];
        scale = 0.25;
    end
    
    methods
        function self = HansCute()
            self.GetCuteRobot();
            self.InitCuteRobot();
            self.PlotAndColourRobot();
        end
        %%
        function GetCuteRobot(self)
            pause(0.001);
            name = ['bob'];
            qlimH = [-2.5, 2.5];
            qlimV = [-1.8, 1.8];
            
            L1 = Link('d', 0.0872,'a', 0,'alpha', -pi/2,'offset', 0,'qlim', qlimH);
            L2 = Link('d', 0,'a', 0,'alpha', pi/2,'offset', 0,'qlim', qlimV); %qlimV
            L3 = Link('d', 0.0768,'a', 0,'alpha', -pi/2,'offset',0,'qlim', qlimH); %qlimH
            L4 = Link('d', 0,'a', 0.0488,'alpha', -pi/2,'offset', -pi/2,'qlim', qlimV);
            L5 = Link('d', 0,'a', 0.0663,'alpha', pi/2,'offset', 0,'qlim', qlimV);
            L6 = Link('d', 0,'a', 0,'alpha', -pi/2,'offset', -pi/2,'qlim',qlimV); %qlimV
            L7 = Link('d', 0.1,'a', 0,'alpha', 0,'offset', 0,'qlim', qlimH);
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
        end
        %%
        function PlotAndColourRobot(self)
            for linkIndex = 1:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            %             if isempty(findobj(get(gca,'Children'),'Type','Light'))
            %camlight
            %             end
            self.model.delay = 0;
            for linkIndex = 1:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        %%
        function InitCuteRobot(self) %Used to identify the workspace of the robot
            disp('Calculating workspace...');
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
                                        %disp(['HANSCUTE: After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudSizeCute * 100),'% of poses']);
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
        %%
        function [qMatrix] = ObtainMotionMatrices(self, startPose, endPose, numNodes, obj) % Uses RRT* to avoid inputted objects
            % Grabs start pose and end pose for RRT* path planning
            self.startPose = startPose;
            self.endPose = endPose;
            self.qStart = self.model.ikcon(startPose, self.model.getpos);
            self.qEnd = self.model.ikcon(endPose);
            
            startCoord = self.startPose(1:3, 4)';
            endCoord = self.endPose(1:3, 4)';
            q_conf.coord = startCoord;
            q_conf.cost = 0;
            q_conf.parent = 0;
            q_goal.coord = endCoord;
            q_goal.cost = 0;
            stepSize = 0.04; % Placement of next node radius size
            
            node(1) = q_conf;
            numOfObj = numel(obj); % used to iterate through the number of objects
            
            % Section below for the bounds of the rrt* points within the
            % workspace
            x_min = min(self.pointCloud(:,1));
            x_max = max(self.pointCloud(:,1));
            y_min = min(self.pointCloud(:,2));
            y_max = max(self.pointCloud(:,2));
            z_min = min(self.pointCloud(:,3));
            z_max = max(self.pointCloud(:,3));
            
            for i = 1:1:numNodes
                [q_near, q_rand, val] = GetNearNode(node, x_min, x_max, y_min, y_max, z_min, z_max);
                q_new.coord = steer3d(q_rand, q_near.coord, val, stepSize); % obtains a new node
                
                q1Node = q_near.coord;
                q2Node = q_new.coord;
                
                % checks if the new node is colliding with any objects, if
                % safe = 1, no collision in environment
                % safe = 0, there is collision and will retrieve new node
                % from the beginning of for loop
                safe = CheckCollision(obj, numOfObj, q1Node, q2Node);
                
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
            [coordMatrix, numWayPoints] = GeneratePath(node, q_goal); % generates the shortest path through the tree
            
            % Section below is for RMRC
            [qMatrix, trMatrix, poseMatrix] = ObtainPoseJointMatrices(self, coordMatrix, numWayPoints);
            [velMatrix, error] = ObtainVelocityMatrix(self, qMatrix, trMatrix);
            
        end
        %%
        function [qMatrix, trMatrix, poseMatrix] = ObtainPoseJointMatrices(self, coordMatrix, numWayPoints)
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
            % velocity
            trMatrix = zeros(4,4,size(qMatrix, 1));
            for i = 1:size(qMatrix, 1)
                trMatrix(:,:,i) = self.model.fkine(qMatrix(i,:));
            end
        end
        %%
        function [velMatrix, error] = ObtainVelocityMatrix(self,qMatrix, trMatrix)
            velMatrix = zeros(size(qMatrix,1), 7);
            %velMatrix(1,:) = qMatrix(1, :);
            error = nan(6, size(qMatrix, 1));
            deltaT = 0.05;
            
            for i = 1:size(qMatrix,1)-1
                dq = qMatrix(i+1,:) - qMatrix(i,:);
                dq = max(dq);
                poseDot = (trMatrix(:,:,i+1) - trMatrix(:,:,i)) / deltaT; %deltaT
                dRDot = poseDot(1:3, 1:3);
                R = trMatrix(1:3,1:3,i);
                
                sMatrix = dRDot * R';
                angVel = vex(sMatrix);
                nu = [poseDot(1:3,4); angVel];
                
                J = self.model.jacob0(qMatrix(i,:));
                q = pinv(J) * nu;
                q = q';
                N = null(J);
                nspm = norm(J * N);
                
                %error(:,i) = nu - J*q';
                velMatrix(i,:) = qMatrix(i,:) + deltaT*q + nspm;
                
            end
            velMatrix(end,:) = velMatrix(end-1,:);
        end
        
    end
    
    %%
    methods (Static)
        
    end
end

%% outside of HANSCUTE class
function safe = CheckCollision(obj, numOfObj, q1Node, q2Node)
for i = 1:1:numOfObj
    faces = obj(i).objEnvironment.faces;
    vertex = obj(i).objEnvironment.vertex;
    faceNormals = obj(i).objEnvironment.faceNormals;
    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectionNode, checkNode] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane, q1Node, q2Node);
        if checkNode == 1 && IsIntersectionPointInsideTriangle(intersectionNode,vertex(faces(faceIndex,:)',:))
            %plot3(intersectionNode(1),intersectionNode(2),intersectionNode(3), 'r*', 'MarkerSize', 10);
            %display([i,intersectionNode(1),intersectionNode(2),intersectionNode(3)], 'Intersection');
            safe = 0;
            break
        end
        safe = 1;
    end
    
end
end

function [q_near, q_rand, val] = GetNearNode(node, x_min, x_max, y_min, y_max, z_min, z_max)
% section below chooses a random point within the bounds
xRand = x_min + (x_max + x_max) * rand(1);
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

%%
function [coordMatrix, counter] = GeneratePath(node, q_goal)
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

%%
function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

intersectionPoint = [0 0 0];
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;
D = dot(planeNormal,u);
N = -dot(planeNormal,w);
check = 0; %#ok<NASGU>
if abs(D) < 10^-7        % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check = 2;
        return
    else
        check = 0;       %no intersection
        return
    end
end

%compute the intersection parameter
sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

if (sI < 0 || sI > 1)
    check= 3;          %The intersection point  lies outside the segment, so there is no intersection
else
    check=1;
end
end

%%

function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                %plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end
    end
end
end
