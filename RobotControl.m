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
            
            self.sponge.MoveSponge(transl(-0.1,0.22,0)*trotx(pi));
            
            self.plate1.MovePlate(transl(0.1,0.22,0)*trotx(pi));
            self.plate2.MovePlate(transl(0.1,0.22,0.02)*trotx(pi));
            self.plate3.MovePlate(transl(0.1,0.22,0.04)*trotx(pi));
        end
        
        %% Run simulation
        
        function SimulateRobot(self)           
            qStart = deg2rad([-90,0,180,26.8,0,63.2,0]);
            
            % set dishwashing locations
            trDirty = transl(0.1,0.22,0.04)*trotx(pi);
            trRack = transl(-0.25,0,0)*trotx(pi);
            trSponge = transl(-0.1,0.22,0)*trotx(pi);
            trScrub  = transl(-0.22,0,0.03)*trotx(pi);
            trClean = transl(0,-0.22,0)*trotx(pi);
            
            % q0 guesses
            guess1 = deg2rad([-78,-75,-143,30.9,47.4,49,0]); % between start to dirty stack
            guess2 = deg2rad([-24.1,-75,-143,30.9,47.4,49,0]); % between dirty to rack
            guess3 = deg2rad([-12.7,-75,-143,30.9,47.4,49,0]); % between rack to sponge
            guess4 = deg2rad([81.9,-62,-143,49.5,61.8,49,0]); % between rack plate to clean stack
            
            disp('Grabbing plate...')
            self.MoveRobot(self.plate3.pose,guess1,false,false); % start to dirty           
            self.MoveRobot(trRack,guess2,false,true); % dirty to rack
            
            disp('Cleaning plate...')
            self.MoveRobot(trSponge,guess3,false,false); % rack to sponge
            self.MoveRobot(trScrub,guess3,true,false); % sponge to rack
            self.Scrub(); % simulate scrubbing          
            self.MoveRobot(trSponge,guess3,true,false); % rack to sponge
            self.MoveRobot(trRack,guess3,false,false); % sponge to rack
            
            disp('Putting away clean plate...')
            self.MoveRobot(trClean,guess4,false,true); % rack to clean
            
            disp('Getting new plate...')
            self.MoveRobot(trDirty,guess4,false,false);
            
%             % waypoints
%             trWay1 = transl(-0.2,0.2,0.2)*trotx(pi);
%             trWay2 = transl(-0.2,-0.2,0.2)*trotx(pi);
%             
%             % simulate diswashing
%             disp('Grabbing plate...')
%             self.MoveRobot(self.plate3.pose,false,false);
%             self.MoveRobot(trWay1,false,true);
%             self.MoveRobot(trRack,false,true);
%             
%             disp('Grabbing sponge...')
%             self.MoveRobot(trWay1,false,false);
%             self.MoveRobot(trSponge,false,false);
%             self.MoveRobot(trWay1,true,false);
%             
%             disp('Cleaning plate...')
%             self.MoveRobot(trScrub,true,false);
%             self.Scrub();
%             self.MoveRobot(trWay1,true,false);
%             self.MoveRobot(trSponge,true,false);
%             self.MoveRobot(trWay1,false,false);
%             
%             disp('Putting away clean plate...')
%             self.MoveRobot(trRack,false,false);
%             self.MoveRobot(trWay2,false,true);
%             self.MoveRobot(trClean,false,true);
%             
%             disp('Getting new plate...')
%             self.MoveRobot(trWay2,false,false);
%             self.MoveRobot(trDirty,false,false);
        end
        
        %% Control robot movements
        
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
                    self.plate3.MovePlate(trPlate);
                end               
            end
        end
        
        %% Simulate scrubbing plate
        
        function Scrub(self)
            steps = 30;
            s = lspb(0,1,steps);
            
            qCurrent = self.robot.model.getpos();
            q1 = [qCurrent(1,1),qCurrent(1,2),qCurrent(1,3),qCurrent(1,4),qCurrent(1,5),qCurrent(1,6),-pi];
            q2 = [qCurrent(1,1),qCurrent(1,2),qCurrent(1,3),qCurrent(1,4),qCurrent(1,5),qCurrent(1,6),pi];
            
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

