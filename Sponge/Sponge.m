classdef Sponge
    % This class creates the bricks using plyread and updates brick
    % locations.
    
    properties
        pose;
        vertexCount;
        midPoint;
        verts;
        mesh;
    end
    
     methods
        function self = Sponge()
            % load brick
            [brickFaceData,brickVertexData,brickData] = plyread('brick9.ply','tri');
            
            % Get vertex count
            self.vertexCount = size(brickVertexData,1);
            
            % Move center point to origin
            self.midPoint = sum(brickVertexData)/self.vertexCount;
            self.verts = brickVertexData - repmat(self.midPoint,self.vertexCount,1);
            
            % Create a transform to describe the location (at the origin, since it's centered
            self.pose = eye(4);
            
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            brickVertexColours = [brickData.vertex.red, brickData.vertex.green, brickData.vertex.blue] / 255;
            
            self.mesh = trisurf(brickFaceData,self.verts(:,1),self.verts(:,2), self.verts(:,3) ...
            ,'FaceVertexCData',brickVertexColours,'EdgeColor','interp','EdgeLighting','flat');                        
        end
        
        
        function MoveSponge(self,newPose)
            self.pose = newPose;
            
            % transform the vertices
            updatedBrickPoints = [self.pose*[self.verts,ones(self.vertexCount,1)]']';
            
            % update the mesh vertices in the patch handle
            self.mesh.Vertices = updatedBrickPoints(:,1:3);
            
            drawnow();
        end
        
        %%
     end
end

