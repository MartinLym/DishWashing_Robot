classdef RobotControl
    
    properties
        robot;

        sponge;
        plate;       
        stack;
        sink;        
    end
    
    methods
        %% Initialise classes
        
        function self = RobotControl()
            disp('Initialising robot and environment...');
            self.robot = HANSCUTE();
            hold on;
            self.sponge = Sponge();
            self.plate = Plate();
            camlight;
            
            self.sponge.MoveSponge(transl(0.1,0.07,0)*trotx(pi));
            self.plate.MovePlate(transl(-0.02,0.22,0)*trotx(pi));
            
            self.stack = Environment(-0.02,0.22,-0.1, 'stackedplates.ply', false, 0.21, 0.14);
            self.sink = Environment(0,-0.06,-0.32, 'sink.ply', false, 0.58, 0.5);
        end
        
        %% Run simulation
        
        function SimulateRobot(self)
            qStart = deg2rad([-90,0,180,26.8,0,63.2,0]);
            startPose = self.robot.model.fkine(qStart);
            
            % set dishwashing locations
            trDirty = transl(-0.02,0.22,0)*trotx(pi);
            trRack = transl(-0.15,0,0)*trotx(pi);
            trSponge = transl(0.1,0.07,0)*trotx(pi);
            trScrub  = transl(-0.15,0,0.01)*trotx(pi);
            trClean = transl(-0.02,-0.22,0)*trotx(pi);
            
            disp('Grabbing plate...')
            [qMatrix1, velMatrix1, trMatrix1, poseMatrix1, coordMatrix1] = ...
                self.robot.obtainMotionMatrices(startPose,self.plate.pose,1500,self.sink);
            self.AnimateRobot(qMatrix1,false,false); % start to dirty
            
            [qMatrix2, velMatrix2, trMatrix2, poseMatrix2, coordMatrix2] = ...
                self.robot.obtainMotionMatrices(self.plate.pose,trRack,1500,self.sink);
            self.AnimateRobot(qMatrix2,false,true); % dirty to rack
            
            disp('Cleaning plate...')
            [qMatrix3, velMatrix3, trMatrix3, poseMatrix3, coordMatrix3] = ...
                self.robot.obtainMotionMatrices(trRack,self.sponge.pose,1500,self.sink);
            self.AnimateRobot(qMatrix3,false,false); % rack to sponge
            
            [qMatrix4, velMatrix4, trMatrix4, poseMatrix4, coordMatrix4] = ...
                self.robot.obtainMotionMatrices(self.sponge.pose,trScrub,1500,self.sink);
            self.AnimateRobot(qMatrix4,true,false); % sponge to rack
            
            self.Scrub(); % simulate scrubbing
            
            disp('Returning sponge...')
            [qMatrix5, velMatrix4, trMatrix4, poseMatrix4, coordMatrix4] = ...
                self.robot.obtainMotionMatrices(self.sponge.pose,trSponge,1500,self.sink);
            self.AnimateRobot(qMatrix5,true,false); % rack to return sponge
                    
            [qMatrix6, velMatrix4, trMatrix4, poseMatrix4, coordMatrix4] = ...
                self.robot.obtainMotionMatrices(self.sponge.pose,trRack,1500,self.sink);
            self.AnimateRobot(qMatrix6,false,false); % sponge to rack   
            
            disp('Putting away clean plate...')
            [qMatrix7, velMatrix4, trMatrix4, poseMatrix4, coordMatrix4] = ...
                self.robot.obtainMotionMatrices(trRack,trClean,1500,self.sink);
            self.AnimateRobot(qMatrix7,false,true); % rack to clean  
            
            disp('Getting new plate...')
            [qMatrix8, velMatrix4, trMatrix4, poseMatrix4, coordMatrix4] = ...
                self.robot.obtainMotionMatrices(trClean,trDirty,1500,self.sink);
            self.AnimateRobot(qMatrix8,false,false); % rack to clean
                      
        end
        
        %% Control robot movements (old)
        
        function MoveRobot(self,trGoal,qGuess,sMarker,pMarker)
            steps = 30;
            s = lspb(0,1,steps);
            self.qMatrix = ones(steps,7);
            
            qStart = self.robot.model.getpos();
            qEnd = self.robot.model.ikcon(trGoal,qGuess);
            
            pickUpSponge = sMarker;
            pickUpPlate = pMarker;
            
            for i = 1:steps
                self.qMatrix(i,:) = (1-s(i))*qStart + s(i)*qEnd;
                self.robot.model.animate(self.qMatrix(i,:))
                
                if pickUpSponge == true
                    trSponge = self.robot.model.fkine(self.qMatrix(i,:));
                    self.sponge.MoveSponge(trSponge);
                end
                
                if pickUpPlate == true
                    trPlate = self.robot.model.fkine(self.qMatrix(i,:));
                    self.plate.MovePlate(trPlate);
                end
            end
        end
        
        %% Animate robot movements
        
        function AnimateRobot(self,qMatrix,sMarker,pMarker)
            pickUpSponge = sMarker;
            pickUpPlate = pMarker;
            
            for i = 1:10:size(qMatrix, 1)
                self.robot.model.animate(qMatrix(i,:));
                
                if pickUpSponge == true
                    trSponge = self.robot.model.fkine(qMatrix(i,:));
                    self.sponge.MoveSponge(trSponge)
                end
                
                if pickUpPlate == true
                    trPlate = self.robot.model.fkine(qMatrix(i,:));
                    self.plate.MovePlate(trPlate);
                end
            end
        end
        
        %% Simulate scrubbing plate
        
        function Scrub(self)
            steps = 30;
            s = lspb(0,1,steps);
            
            qCurrent = self.robot.model.getpos();
            q1 = [qCurrent(1,1),qCurrent(1,2),qCurrent(1,3),qCurrent(1,4),qCurrent(1,5),qCurrent(1,6),-pi]; % spin end-effector 360 deg
            q2 = [qCurrent(1,1),qCurrent(1,2),qCurrent(1,3),qCurrent(1,4),qCurrent(1,5),qCurrent(1,6),pi];
            
            for u = 1:2
                for i = 1:steps
                    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
                    trSponge = self.robot.model.fkine(qMatrix(i,:));
                    self.sponge.MoveSponge(trSponge);
                    
                end
                for i = 1:steps
                    qMatrix(i,:) = (1-s(i))*q2 + s(i)*q1;
                    trSponge = self.robot.model.fkine(qMatrix(i,:));
                    self.sponge.MoveSponge(trSponge);
                end
            end
        end
        
        %% GUI
        % Cartesian (x,y,z) input to move robot
        
        function CartesianInput(self,cart)
            qStart = self.robot.model.getpos();
            trGoal = transl(cart)*trotx(pi);
            qEnd = self.robot.model.ikcon(trGoal);
            
            steps = 30;
            s = lspb(0,1,steps);
            self.qMatrix = ones(steps,7);
            
            disp(['Moving end-effector to [',num2str(trGoal(1,4)),',',num2str(trGoal(2,4)),',',num2str(trGoal(3,4)),']']);
            
            for i = 1:steps
                self.qMatrix(i,:) = (1-s(i))*qStart + s(i)*qEnd;
                self.robot.model.animate(self.qMatrix(i,:));
                drawnow();
            end
        end
        
        %% GUI
        % Slider input to move robot
        
        function SliderInput(self,jointNum,val)
            qJoint = self.robot.model.getpos();
            qJoint(1,jointNum) = deg2rad(val);
            self.robot.model.animate(qJoint);
            drawnow();
        end
        
        %% end of functions
    end
end

