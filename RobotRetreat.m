classdef RobotRetreat < handle
    % This class uses visual servoing to simulate the robot
    % reacting to safety symbol by retreating backwards to maintain a
    % safe distance between itself and the symbol
    
    properties
        robot;
        
        P;
        lambda;
        cam;
        depth;
        fps;
        Tc0;
        pStar;
        vel_p;
        uv_p;
        history;
    end
    
    methods
        function self = RobotRetreat()
            self.robot = HansCute();
        end
        
        %%
        function SetPointsVS(self)
            x = 0.5;
            y = 0;
            z = 0.15;
            
            % insert object into environment
            Environment(x,y,z,'sign.ply', false, 0.1, 0.1);
            
            %Create 3D points (actual points in figure)
            self.P=[x,x,x,x;
                y-0.075,y+0.075,y+0.075,y-0.075; %0.15 away
                z-0.075,z-0.075,z+0.075,z+0.075]; %0.15 away
            
        end
        
        %%
        function InitialiseVS(self)
            % Create image target (points in the image plane)
            self.pStar = [300 300 600 600; % bottomleft,topleft,topright,bottomright
                600 300 300 600];
            
            % Add the camera
            self.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                'resolution', [1024 1024], 'centre', [512 512],'name', 'Camera');
            
            % frame rate
            self.fps = 25;
            
            %Define values
            %gain of the controler
            self.lambda = 0.6;
            
            %depth of the IBVS
            self.depth = mean(self.P(1,:));
            %self.depth = 0.2;
            
            %Display robot
            q0 = [0,0,0,pi/2,0,0,0];
            qCurrent = self.robot.model.getpos();
            self.Tc0= self.robot.model.fkine(q0);
            self.robot.model.animate(jtraj(qCurrent,q0,30));
            drawnow
        end
        
        %%
        function PlotPointsVS(self)
            % plot camera and points
            self.Tc0= self.robot.model.fkine(self.robot.model.getpos);
            self.cam.T = self.Tc0;
            
            % Display points in 3D and the camera
            self.cam.plot_camera('Tcam',self.Tc0, 'label','scale',0.04);
            %plot_sphere(self.P, 0.005, 'b')
            lighting gouraud
            light
            
            %Project points to the image
            p = self.cam.plot(self.P, 'Tcam', self.Tc0);
            
            %camera view and plotting
            self.cam.clf()
            self.cam.plot(self.pStar, '*'); % create the camera view
            self.cam.hold(true);
            self.cam.plot(self.P, 'Tcam', self.Tc0, 'o'); % create the real world view
            pause(2)
            self.cam.hold(true);
            self.cam.plot(self.P);    % show initial view
            
            %Initialise display arrays
            self.vel_p = [];
            self.uv_p = [];
            self.history = [];
        end
        
        %%
        function AnimateVS(self)
            q0 = [0,0,0,pi/2,0,0,0];
            
            for ksteps = 1:150
                %ksteps = ksteps + 1;
                
                % compute the view of the camera
                uv = self.cam.plot(self.P);
                
                % compute image plane error as a column
                e = self.pStar-uv;   % feature error
                e = e(:);
                Zest = [];
                
                % compute the Jacobian
                if isempty(self.depth)
                    % exact depth from simulation (not possible in practice)
                    pt = homtrans(inv(Tcam), self.P);
                    J = self.cam.visjac_p(uv, pt(3,:) );
                elseif ~isempty(Zest)
                    J = self.cam.visjac_p(uv, Zest);
                else
                    J = self.cam.visjac_p(uv, self.depth);
                end
                
                % compute the velocity of camera in camera frame
                try
                    v = self.lambda * pinv(J) * e;
                catch
                    status = -1;
                    return
                end
                %fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
                
                %compute robot's Jacobian and inverse
                J2 = self.robot.model.jacobn(q0);
                Jinv = pinv(J2);
                % get joint velocities
                qp = Jinv*v;
                
                %Maximum angular velocity cannot exceed 180 degrees/s
                ind=find(qp>pi);
                if ~isempty(ind)
                    qp(ind)=pi;
                end
                ind=find(qp<-pi);
                if ~isempty(ind)
                    qp(ind)=-pi;
                end
                
                %Update joints
                q = q0 + (1/self.fps)*qp';
                self.robot.model.animate(q);
                
                %Get camera location
                Tc = self.robot.model.fkine(q);
                self.cam.T = Tc;
                drawnow
                
                % update the history variables
                hist.uv = uv(:);
                vel = v;
                hist.vel = vel;
                hist.e = e;
                hist.en = norm(e);
                hist.jcond = cond(J);
                hist.Tcam = Tc;
                hist.vel_p = vel;
                hist.uv_p = uv;
                hist.qp = qp;
                hist.q = q;
                
                self.history = [self.history hist];
                
                pause(1/self.fps)               
                
                %update current joint position
                q0 = q;
            end
        end
        
%%        
    end
end

