classdef Environment
    % ENVIRONMENT Summary of this class goes here
    % Detailed explanation goes here
    
    properties
        % Defines object appearance and pose
        pose;
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
                        
        end
        
        function Sponge(self)
            % load brick
            [FaceData,VertexData,data] = plyread('sponge2.ply','tri');
            
            % Get vertex count
            self.vertexCount = size(VertexData,1);
            
            % Move center point to origin
            self.midPoint = sum(VertexData)/self.vertexCount;
            self.verts = VertexData - repmat(self.midPoint,self.vertexCount,1);
            
            % Create a transform to describe the location (at the origin, since it's centered
            self.pose = eye(4);
            
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            brickVertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            self.mesh = trisurf(FaceData,self.verts(:,1),self.verts(:,2), self.verts(:,3) ...
            ,'FaceVertexCData',brickVertexColours,'EdgeColor','interp','EdgeLighting','flat');                        
        end
        
        function MoveSponge(self,newPose)
            self.pose = newPose;
            
            % transform the vertices
            updatedPoints = [self.pose*[self.verts,ones(self.vertexCount,1)]']';
            
            % update the mesh vertices in the patch handle
            self.mesh.Vertices = updatedPoints(:,1:3);
            
            drawnow();
        end
        
        %% end of functions
    end
end

