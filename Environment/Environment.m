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
        
        %%
%         function [vertex,face,faceNormals] = RectangularPrism(lower,upper,plotOptions,axis_h)
%             if nargin<4
%                 axis_h=gca;
%                 if nargin<3
%                     plotOptions.plotVerts=false;
%                     plotOptions.plotEdges=true;
%                     plotOptions.plotFaces=true;
%                 end
%             end
%             hold on
%             
%             vertex(1,:)=lower;
%             vertex(2,:)=[upper(1),lower(2:3)];
%             vertex(3,:)=[upper(1:2),lower(3)];
%             vertex(4,:)=[upper(1),lower(2),upper(3)];
%             vertex(5,:)=[lower(1),upper(2:3)];
%             vertex(6,:)=[lower(1:2),upper(3)];
%             vertex(7,:)=[lower(1),upper(2),lower(3)];
%             vertex(8,:)=upper;
%             
%             face=[1,2,3;1,3,7;
%                 1,6,5;1,7,5;
%                 1,6,4;1,4,2;
%                 6,4,8;6,5,8;
%                 2,4,8;2,3,8;
%                 3,7,5;3,8,5;
%                 6,5,8;6,4,8];
%             
%             if 2 < nargout
%                 faceNormals = zeros(size(face,1),3);
%                 for faceIndex = 1:size(face,1)
%                     v1 = vertex(face(faceIndex,1)',:);
%                     v2 = vertex(face(faceIndex,2)',:);
%                     v3 = vertex(face(faceIndex,3)',:);
%                     faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
%                 end
%             end
%             %% If plot verticies
%             if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
%                 for i=1:size(vertex,1);
%                     plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
%                     text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
%                 end
%             end
%             
%             %% If you want to plot the edges
%             if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
%                 links=[1,2;
%                     2,3;
%                     3,7;
%                     7,1;
%                     1,6;
%                     5,6;
%                     5,7;
%                     4,8;
%                     5,8;
%                     6,4;
%                     4,2;
%                     8,3];
%                 
%                 for i=1:size(links,1)
%                     plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
%                         [vertex(links(i,1),2),vertex(links(i,2),2)],...
%                         [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
%                 end
%             end
%             
%             %% If you want to plot the edges
%             if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
%                 tcolor = [.2 .2 .8];
%                 
%                 patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','lineStyle','none');
%             end
%             
%         end
        
        %% end of functions
    end
end

