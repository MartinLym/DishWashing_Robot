classdef Sponge < handle
    % This class creates the sponge using plyread and updates sponge
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
            [FaceData,VertexData,data] = plyread('sponge.ply','tri');
            
            % Get vertex count
            self.vertexCount = size(VertexData,1);
            
            % Move center point to origin
            self.midPoint = sum(VertexData)/self.vertexCount;
            self.verts = VertexData - repmat(self.midPoint,self.vertexCount,1);
            
            % Create a transform to describe the location (at the origin, since it's centered
            self.pose = eye(4);
            
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            VertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            self.mesh = trisurf(FaceData,self.verts(:,1),self.verts(:,2), self.verts(:,3) ...
            ,'FaceVertexCData',VertexColours,'EdgeColor','interp','EdgeLighting','flat');                        
        end
        
        %%
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

