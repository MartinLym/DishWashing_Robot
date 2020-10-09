clf
clear

Ppt = [662 362 362 662; 362 362 662 662];
pStar = [430 568 568 430;500 500 362 362];
%pStar = [593 430 430 593;362 362 525 525];

%P = mkgrid(2, 0.05, transl(-0.5, 0, 0.175)*troty(pi/2))
P=[-0.35,-0.35,-0.35,-0.35; %0.1479, 0.1457
   -0.01,0.01,0.01,-0.01;
   0.13,0.13,0.15, 0.15];


robot = HANSCUTE();
%q0 = [0 -0.6680 0 -1.1 0 0 0];
q0 = [-0.15 -0.8949 0 -0.7523 0 0.0569 0];
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'HANScam');

fps = 25;
lambda = 0.8;
depth = mean(P(1,:));
epsilon = 0.1;
lambdaMax = 5e-2;
%%
Tc0 = robot.model.fkine(q0);
robot.model.teach(q0);

Tc0 = robot.model.fkine(robot.model.getpos);
cam.T = Tc0;
cam.plot_camera('Tcam', Tc0, 'scale',0.05)

plot_sphere(P, 0.0075, 'b');
%plot_sphere(PP, 0.0075, 'r');
lighting gouraud
light

p = cam.plot(P, 'Tcam', Tc0);
cam.clf();
cam.plot(pStar, '*');
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o');
pause(2)
cam.hold(true);
cam.plot(P) 

vel_p = [];
uv_p = [];
history = [];

%%
depth1 = depth;
depth2 = depth;
depth3 = depth;
depth4 = depth;
depthTest = depth;
ksteps = 0;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(P);
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P);
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv,-0.05);
        end
        
        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
        
        %compute robot's Jacobian and inverse
        J2 = robot.model.jacobn(q0);
        %m(ksteps) = sqrt(det(J2*J2'));
        %if m(ksteps) < epsilon  % If manipulability is less than given threshold
        %    lambda2 = (1 - m(ksteps)/epsilon)*lambdaMax;
        %else
        %    lambda2 = 0;
        %end
        %Jinv = pinv(J2' * J2 + lambda2 * eye(7))*J2';
        % get joint velocities
        Jinv = pinv(J2);
        qp = Jinv*v;
         
         %Maximum angular velocity cannot exceed 180 degrees/s
         %{
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end
         %}
        %Update joints 
        q = q0 + (1/fps)*qp';
        robot.model.animate(q)

        %Get camera location
        Tc = robot.model.fkine(q);
        cam.T = Tc;
        depth1 = sqrt((P(1,1) - Tc(1,4))^2 + (P(2,1) - Tc(2,4))^2 + (P(3,1) - Tc(3,4))^2 );
        depth2 = sqrt((P(1,2) - Tc(1,4))^2 + (P(2,2) - Tc(2,4))^2 + (P(3,2) - Tc(3,4))^2 );
        depth3 = sqrt((P(1,3) - Tc(1,4))^2 + (P(2,3) - Tc(2,4))^2 + (P(3,3) - Tc(3,4))^2 );
        depth4 = sqrt((P(1,4) - Tc(1,4))^2 + (P(2,4) - Tc(2,4))^2 + (P(3,4) - Tc(3,4))^2 );
        depthTest = [depth1;depth2;depth3;depth4]
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

        history = [history hist];

        pause(1/fps)

        if ~isempty(500) && (ksteps > 500)
            break;
        end
        
        %update current joint position
        q0 = q;
 end %loop finishes
 
 %{
%% 1.5 Plot results
figure()            
plot_p(history,pStar,cam)
figure()
plot_camera(history)
figure()
plot_vel(history)
figure()
plot_robjointpos(history)
figure()
plot_robjointvel(history)

%% Functions for plotting

 function plot_p(history,uv_star,camera)
            %VisualServo.plot_p Plot feature trajectory
            %
            % VS.plot_p() plots the feature values versus time.
            %
            % See also VS.plot_vel, VS.plot_error, VS.plot_camera,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
            
            if isempty(history)
                return
            end
            figure();
            clf
            hold on
            % image plane trajectory
            uv = [history.uv]';
            % result is a vector with row per time step, each row is u1, v1, u2, v2 ...
            for i=1:numcols(uv)/2
                p = uv(:,i*2-1:i*2);    % get data for i'th point
                plot(p(:,1), p(:,2))
            end
            plot_poly( reshape(uv(1,:), 2, []), 'o--');
            uv(end,:)
            if ~isempty(uv_star)
                plot_poly(uv_star, '*:')
            else
                plot_poly( reshape(uv(end,:), 2, []), 'rd--');
            end
            axis([0 camera.npix(1) 0 camera.npix(2)]);
            set(gca, 'Ydir' , 'reverse');
            grid
            xlabel('u (pixels)');
            ylabel('v (pixels)');
            hold off
        end

       function plot_vel(history)
            %VisualServo.plot_vel Plot camera trajectory
            %
            % VS.plot_vel() plots the camera velocity versus time.
            %
            % See also VS.plot_p, VS.plot_error, VS.plot_camera,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
            if isempty(history)
                return
            end
            clf
            vel = [history.vel]';
            plot(vel(:,1:3), '-')
            hold on
            plot(vel(:,4:6), '--')
            hold off
            ylabel('Cartesian velocity')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('v_x', 'v_y', 'v_z', '\omega_x', '\omega_y', '\omega_z')
        end

        function plot_camera(history)
            %VisualServo.plot_camera Plot camera trajectory
            %
            % VS.plot_camera() plots the camera pose versus time.
            %
            % See also VS.plot_p, VS.plot_vel, VS.plot_error,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.

            if isempty(history)
                return
            end
            clf
            % Cartesian camera position vs time
            T = reshape([history.Tcam], 4, 4, []);
            subplot(211)
            plot(transl(T));
            ylabel('camera position')
            grid
            subplot(212)
            plot(tr2rpy(T))
            ylabel('camera orientation')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('R', 'P', 'Y');
            subplot(211)
            legend('X', 'Y', 'Z');
        end

        function plot_robjointvel(history)
          
            if isempty(history)
                return
            end
            clf
            vel = [history.qp]';
            plot(vel(:,1:6), '-')
            hold on
            ylabel('Joint velocity')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', '\omega_5', '\omega_6')
        end

 function plot_robjointpos(history)
          
            if isempty(history)
                return
            end
            clf
            pos = [history.q]';
            plot(pos(:,1:7), '-')
            hold on
            ylabel('Joint angle')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6')
        end
     %}