classdef Environment
    % ENVIRONMENT Summary of this class goes here
    % Detailed explanation goes here
    
    properties
        % Defines object appearance and pose
        pose;
        nPose = zeros(4,4,3);
        vertexCount;
        midPoint;
        verts;
        mesh_h;
                
        % Defines object 
        centrePt;
        objEnvironment;
    end
    
    methods
        function self = Environment(posX, posY, posZ, file, show, side, height)
            % section below for object appearance and pose
            self.pose = makehgtform('translate', [posX, posY, posZ]) * trotx(pi);
            [f,v,data] = plyread(file,'tri');
            self.vertexCount = size(v,1);
            self.midPoint = sum(v)/self.vertexCount;
            self.verts = v - repmat(self.midPoint,self.vertexCount,1);

            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.mesh_h = trisurf(f, v(:,1) + self.pose(1,4), v(:,2) + self.pose(2,4), ...
                v(:,3)+ self.pose(3,4),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            % section below for object avoidance collision
            self.centrePt = [posX, posY, posZ];
            self.centrePt(3) = self.centrePt(3) + height;
            lower = self.centrePt;
            self.centrePt(3) = self.centrePt(3) - (height*2);
            upper = self.centrePt;
            
            plotOptions.plotFaces = show;
            [vertex,faces,faceNormals] = RectangularPrism(lower-side/2, upper+side/2, plotOptions);
            field1 = 'vertex'; 
            field2 = 'faces'; 
            field3 = 'faceNormals';
            self.objEnvironment = struct(field1, vertex, field2, faces, field3, faceNormals);
            
            distanceBase2Obj = distance_2d([0;0;0], self.pose(1:3,4))
            nPoseX = ((distanceBase2Obj-(side/2)) / distanceBase2Obj) * self.pose(1,4);
            nPoseY = ((distanceBase2Obj-(side/2)) / distanceBase2Obj) * self.pose(2,4);
            nPoseZ = self.pose(3,4);
            thetaZ = atan2(posY,posX);
            self.nPose(:,:,1) = makehgtform('translate', [nPoseX, nPoseY, nPoseZ]) * trotz(thetaZ + pi/2) * trotx(pi);
            self.nPose(:,:,2) = self.nPose(:,:,1);
            self.nPose(:,:,3) = self.nPose(:,:,2);
        end
        
        function wayPointPoses = pickUpPlate(self) % 3 Waypoints
            wayPointPoses = zeros(4,4,3);
            self.nPose(:,:,1) = self.nPose(:,:,1) * transl(0,-0.12, -0.02);
            self.nPose(:,:,2) = self.pose * trotz(-pi/4) * trotx(-pi/2) * transl(0, -0.05, -0.1);% * trotx(-pi/2) * transl(0, -0.02, 0.1);
            self.nPose(:,:,3) = self.nPose(:,:,2) * transl(0, -0.05, 0.05) * trotx(-pi/6);
            
            for i = 1:size(wayPointPoses,3)
                wayPointPoses(:,:,i) = self.nPose(:,:,i);
            end
        end
        
        function wayPointPoses = dropPlate(self, robot) % 3 Waypoints
            wayPointPoses = zeros(4,4,3);
            
            self.nPose(:,:,1) = robot.model.fkine(robot.model.getpos);
            self.nPose(:,:,1) = self.nPose(:,:,1) * troty(pi/3) * transl(0.03, 0, 0.03) ;
            self.nPose(:,:,2) = self.nPose(:,:,1) * troty(pi/4) * transl(0.03, 0, 0.03);
            self.nPose(:,:,3) = self.pose * trotz(-pi/2) * trotx(-pi/2) * troty(pi/4) * transl(0,-0.03,-0.1); %  
            
            for i = 1:size(wayPointPoses,3)
                wayPointPoses(:,:,i) = self.nPose(:,:,i);
            end
        end
        
        function wayPointPoses = pickUpSponge(self, robot)
            wayPointPoses = zeros(4,4,3);
            self.nPose(:,:,1) = self.pose * transl(0, 0.02, -0.1);
            self.nPose(:,:,2) = self.nPose(:,:,1);
            self.nPose(:,:,3) = self.pose * transl(0,0.05,-0.02);
            
            for i = 1:size(wayPointPoses,3)
                wayPointPoses(:,:,i) = self.nPose(:,:,i);
            end
        end
        
        function wayPointPoses = cleanPlate(self, robot)
            wayPointPoses = zeros(4,4,6);
            self.nPose(:,:,1) = robot.model.fkine(robot.model.getpos);
            self.nPose(:,:,1) = self.nPose(:,:,1) * transl(-0.03,-0.03,0);
            self.nPose(:,:,2) = self.nPose(:,:,1) * transl(-0.03,0.03,0);
            self.nPose(:,:,3) = self.nPose(:,:,2) * transl(0.03,0.03,0);
            self.nPose(:,:,4) = self.nPose(:,:,3) * transl(0.03,-0.03,0);
            self.nPose(:,:,5) = self.nPose(:,:,4) * transl(-0.03,-0.03,0);
            self.nPose(:,:,6) = self.pose * transl(0,0,-0.05);
            
            for i = 1:size(wayPointPoses,3)
                wayPointPoses(:,:,i) = self.nPose(:,:,i);
            end
        end
             
    end
end

function d = distance_2d(q1,q2)
d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2);
end