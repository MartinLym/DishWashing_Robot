classdef RobotControl
    
    properties
        robot;       
        enviro;
        sponge;
        plate1;
        plate2;
        plate3;
        qMatrix;
    end
    
    methods 
        %% Initialise classes
        function self = RobotControl()
            self.robot = CUTE();
            hold on;
            self.sponge = Sponge();
            self.plate1 = Plate();
            self.plate2 = Plate();
            self.plate3 = Plate();
            camlight;
            
            self.sponge.MoveSponge(transl(-0.2,0.2,0)*trotz(pi/10));
            
            self.plate1.MovePlate(transl(0,0.25,0)*trotx(pi));
            self.plate2.MovePlate(transl(0,0.25,0.02)*trotx(pi));
            self.plate3.MovePlate(transl(0,0.25,0.04)*trotx(pi));
        end
        
        %% Run Simulation
        
        function SimulateRobot(self)           
            trDirty = transl(0,0.25,0.04);
            trRack = transl(-0.25,0,0);
            trSponge = transl(-0.2,0.22,0);
            trScrub  = transl(-0.25,0,0.02);
            trClean = transl(0,-0.25,0);
            
            % waypoints
            way1 = deg2rad([-90,-45,180,25,0,60,0]); % between start to dirty stack
            way2 = deg2rad([-55,-90,180,25,0,60,0]); % between dirty to rack
            way3 = deg2rad([-15,-90,180,25,0,60,0]); % between rack to sponge
            way4 = deg2rad([45,-90,180,25,0,60,0]); % between rack plate to clean stack
            
            disp('Grabbing plate...')
            self.MoveRobot(trDirty,way1,false,false); % start to dirty           
            self.MoveRobot(trRack,way2,false,true); % dirty to rack
            
            disp('Cleaning plate...')
            self.MoveRobot(trSponge,way3,false,false); % rack to sponge
            self.MoveRobot(trScrub,way3,true,false); % sponge to rack
            self.Scrub(); % simulate scrubbing          
            self.MoveRobot(trSponge,way3,true,false); % rack to sponge
            self.MoveRobot(trRack,way3,false,false); % sponge to rack
            
            disp('Putting away clean plate...')
            self.MoveRobot(trClean,way4,false,true); % rack to clean
            
            disp('Getting new plate...')
            self.MoveRobot(trDirty,way4,false,false);
        end
        
        %% Control robot movements
        
        function MoveRobot(self,trGoal,trInit,sMarker,pMarker)
            steps = 20;
            s = lspb(0,1,steps);
            self.qMatrix = ones(steps,7);
            
            qStart = self.robot.model.getpos();
            qEnd = self.robot.model.ikcon(trGoal,trInit);
            
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
                    self.plate3.MovePlate(trPlate);
                end               
            end
        end
        
        %% Simulate scrubbing plate
        
        function Scrub(self)
            steps = 30;
            s = lspb(0,1,steps);
            
            qStart = self.robot.model.getpos();
            q1 = [qStart(1,1),qStart(1,2),qStart(1,3),qStart(1,4),qStart(1,5),qStart(1,6),-pi];
            q2 = [qStart(1,1),qStart(1,2),qStart(1,3),qStart(1,4),qStart(1,5),qStart(1,6),pi];
            
            for u = 1:2
                for i = 1:steps
                    self.qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
                    trSponge = self.robot.model.fkine(self.qMatrix(i,:));
                    self.sponge.MoveSponge(trSponge);
                    
                    self.qMatrix(i,:) = (1-s(i))*q2 + s(i)*q1;
                    trSponge = self.robot.model.fkine(self.qMatrix(i,:));
                    self.sponge.MoveSponge(trSponge);
                end
            end
        end 
        
        %% end of functions
    end
end

