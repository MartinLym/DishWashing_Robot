classdef RobotControl
    %% Main class for GUI
    
    properties
        robot;
        sponge;
        plate;
        enviro;
        stack;
        qMatrix;
        
        retreat;
    end
    
    methods
        %% Initialise classes
        
        function self = RobotControl()
            disp('Initialising robot and environment...');
            
            % insert robot
            self.robot = HansCute();
            hold on;
                                
            % insert environment
            self.sponge = Sponge();
            self.plate = Plate();
            
            show = false;
            self.enviro = Environment(-0.02,0.14,-0.045,'dirtyplates.ply', show, 0.13, 0.18);           
            self.enviro = Environment(-0.02,-0.15,-0.045,'cleanplates.ply', show, 0.13, 0.18);
            self.enviro = Environment(0,-0.06,-0.28,'enviro.ply', show, 0.9, 0.77);
            %camlight;
            
            % set initial positions
            qStart = ([0,0,0,0,0,0,0]);
            self.robot.model.animate(qStart);           
            self.plate.MovePlate(transl(-0.02,0.14,0.075)*trotx(pi));
            self.sponge.MoveSponge(transl(0.15,0.07,0.06)*trotx(pi));
            
            % initialise visual servoing
            self.retreat = RobotRetreat();
        end
                  
        %% Run simulation
        
        function SimulateRobot(self) 
            qReset = ([0,0,0,0,0,0,0]);
            trReset = self.robot.model.fkine(qReset);
            
            % set dishwashing locations
            trDirty = transl(-0.02,0.14,0.08)*trotx(pi);
            trRack = transl(-0.15,0,0.045)*trotx(pi);
            trSponge = transl(0.15,0.07,0.06)*trotx(pi);
            trScrub  = transl(-0.15,0,0.065)*trotx(pi);
            trClean = transl(-0.02,-0.15,0.09)*trotx(pi);
            
            % waypoints
            way1 = transl(-0.02,0.14,0.3)*trotx(pi);
            way2 = trReset;
            way3 = transl(-0.02,-0.15,0.3)*trotx(pi);          

            disp('Grabbing plate...')
            self.MoveRobot(way1,false,false);
            self.MoveRobot(trDirty,false,false); % start to dirty
            self.MoveRobot(way1,false,true);
            self.MoveRobot(trRack,false,true); % dirty to rack
            
            disp('Grabbing sponge...')
            self.MoveRobot(way2,false,false);
            self.MoveRobot(trSponge,false,false); % rack to sponge
            self.MoveRobot(way2,true,false);
            self.MoveRobot(trScrub,true,false); % sponge to rack
            
            disp('Cleaning plate...')
            self.Scrub(); % simulate scrubbing
            self.MoveRobot(way2,true,false);
            self.MoveRobot(trSponge,true,false); % rack to sponge
            self.MoveRobot(way2,false,false);
            self.MoveRobot(trRack,false,false); % sponge to rack
            
            disp('Putting away clean plate...')
            self.MoveRobot(way3,false,true);
            self.MoveRobot(trClean,false,true); % rack to clean
            
            disp('Getting new plate...')
            self.MoveRobot(trReset,false,false);
                       
        end
        
        %% Control robot movements
        
        function MoveRobot(self,trGoal,sMarker,pMarker)
            steps = 50;
            s = lspb(0,1,steps);
            self.qMatrix = ones(steps,7);
            
            qStart = self.robot.model.getpos();
            qEnd = self.robot.model.ikcon(trGoal);
            
            pickUpSponge = sMarker;
            pickUpPlate = pMarker;
            
            for i = 1:steps
                self.qMatrix(i,:) = (1-s(i))*qStart + s(i)*qEnd;
                self.robot.model.animate(self.qMatrix(i,:))
                pause(0.1);
                
                if pickUpSponge == true
                    trSponge = self.robot.model.fkine(self.qMatrix(i,:));
                    trSponge(3,4) = trSponge(3,4)-0.01;
                    self.sponge.MoveSponge(trSponge);
                end
                
                if pickUpPlate == true
                    trPlate = self.robot.model.fkine(self.qMatrix(i,:));
                    trPlate(3,4) = trPlate(3,4)-0.01;
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
                    trSponge(3,4) = trSponge(3,4)-0.01;
                    self.sponge.MoveSponge(trSponge);
                    
                end
                for i = 1:steps
                    qMatrix(i,:) = (1-s(i))*q2 + s(i)*q1;
                    trSponge = self.robot.model.fkine(qMatrix(i,:));
                    trSponge(3,4) = trSponge(3,4)-0.01;
                    self.sponge.MoveSponge(trSponge);
                end
            end
        end
        
        %% COLLISION DETECTION/AVOIDANCE
        function AnimateCollisionAvoidance(self)
            self.stack = Environment(-0.02,0.15,0.045,'dirtyplates.ply', false, 0.13, 0.18);
            
            t1 = transl(0.2,0.2,0.1)*trotx(pi);
            t2 = transl(-0.2,0.2,0.1)*trotx(pi);
            
            [qMatrix1] = self.robot.ObtainMotionMatrices(t1,t2,1500,self.stack);
            
            for i = 1:5:size(qMatrix1,1)
                self.robot.model.animate(qMatrix1(i,:));
                pause(0.1);
            end
            
        end
        
        %% VISUAL SERVOING           
        
        function RetreatVS(self)
            self.retreat.SetPointsVS();
            self.retreat.InitialiseVS();
            self.retreat.PlotPointsVS();
            pause(2);
            self.retreat.AnimateVS();                     
        end
        
        %% GUI
        % Cartesian (x,y,z) input to move robot
        
        function CartesianInput(self,cart)
            qStart = self.robot.model.getpos();
            trGoal = transl(cart)*trotx(pi);
            qEnd = self.robot.model.ikcon(trGoal);
            
            steps = 30;
            s = lspb(0,1,steps);
            qMatrix = ones(steps,7);
            
            disp(['Moving end-effector to [',num2str(trGoal(1,4)),',',...
                num2str(trGoal(2,4)),',',num2str(trGoal(3,4)),']']);
            
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*qStart + s(i)*qEnd;
                self.robot.model.animate(qMatrix(i,:));
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

