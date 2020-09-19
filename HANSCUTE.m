classdef HANSCUTE < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;
        pointCloud;
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
            L2 = Link('d', 0,'a', 0.0628,'alpha', pi/2,'offset', 0,'qlim', qlimV); %qlimV
            L3 = Link('d', 0.07683,'a', 0,'alpha', -pi/2,'offset', 0,'qlim', qlimH); %qlimH
            L4 = Link('d', 0,'a', 0.048827,'alpha', -pi/2,'offset', -pi/2,'qlim', qlimV);
            L5 = Link('d', 0,'a', 0.06663,'alpha', pi/2,'offset', 0,'qlim', qlimV);
            L6 = Link('d', 0,'a', 0.06663,'alpha', -pi/2,'offset', -pi/2,'qlim',qlimV); %qlimV
            L7 = Link('d', 0.055,'a', 0,'alpha', 0,'offset', 0,'qlim', qlimH);
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
        end
        
        function initCuteRobot(self)
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
        
        function moveCuteRobot(self, startCoord, endCoord, numNodes, obj)
            q_conf.coord = startCoord;
            q_conf.cost = 0;
            q_conf.parent = 0;
            q_goal.coord = endCoord;
            q_goal.cost = 0;
            stepSize = 0.04;
            
            node(1) = q_conf; 
            numOfObj = numel(obj);
            
            x_min = min(self.pointCloud(:,1));
            x_max = max(self.pointCloud(:,1));
            y_min = min(self.pointCloud(:,2));
            y_max = max(self.pointCloud(:,2));
            z_min = min(self.pointCloud(:,3));
            z_max = max(self.pointCloud(:,3));
            
            for i = 1:1:numNodes
                [q_near, q_rand, val] = getNearNode(node, x_min, x_max, y_min, y_max, z_min, z_max);
                q_new.coord = steer3d(q_rand, q_near.coord, val, stepSize);
                
                q1Node = q_near.coord;
                q2Node = q_new.coord;
                
                safe = checkCollision(obj, numOfObj, q1Node, q2Node);
                
                if safe == 1
                   %disp('here'); 
                   q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
                   
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
                   
                   for k = 1:1:length(q_nearest)
                       if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min
                           q_min = q_nearest(k);
                           C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
                           line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');
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
            
            [coordMatrix, counter] = generatePath(node, q_goal);
            
            moveThroughPath(self, coordMatrix, counter);
        end
        
    end
    
    methods (Static)
        
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
    %x_max = 0.4;
    %y_max = 0.4;
    %z_max = 0.4;
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

function moveThroughPath(self, coordMatrix, counter)
    trMatrix = zeros(4,4,counter);
    for i = 1:1:counter
        coordTr = trotz(0);
        coordTr(:,4) = [coordMatrix(i,:)'; 1];
        trMatrix(:,:,i) = coordTr;
    end
    
    mask = [1 1 1 0 0 0];
    for i = 1:1:counter
        q0 = self.model.getpos();
        q1 = self.model.ikine(trMatrix(:,:,i), q0, mask);
        steps = 2;
        while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q0,q1,steps)))),1))
            steps = steps + 1;
        end
        qMatrix = jtraj(q0,q1,steps);
        for i = 1:steps
           self.model.animate(qMatrix(i,:)); 
        end
    end
end