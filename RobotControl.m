classdef RobotControl
    
    properties
        robot;
        enviro;
        sponge;
        qMatrix;
    end
    
    methods
        function self = RobotControl()
            self.robot = Cute();
            hold on;            
            self.sponge = Sponge();
            self.sponge.MoveSponge(transl(0.25,0.2,0.16));
        end
        
        %%
        function SimulateRobots(self)
            qStart = deg2rad([130 0 0 0 0 90 0]);
            
            trDirty = transl(-0.07,0.25,0.17);
            trRack = transl(0.25,0,0.17);
            trSponge = transl(0.25,0.2,0.16); %transl(0.15,0.27,0.16);
            trClean = transl(0,-0.28,0.16);
            
            % waypoint
            way1 = deg2rad([143 50 0 0 0 90 0]); % between start to dirty
            way2 = deg2rad([100 90 0 0 0 90 0]); % between dirty to rack
            way3 = deg2rad([65 90 0 0 0 90 0]); % between rack to sponge
            way4 = deg2rad([8 90 0 0 0 90 0]); % between cleaned plate to clean stack
            
            disp('start to dirty')
            self.MoveRobot(trDirty,way1,false,false); % start to dirty
            disp('dirty to rack')
            self.MoveRobot(trRack,way2,false,true); % dirty to rack
            disp('rack to sponge')
            self.MoveRobot(trSponge,way3,false,false); % rack to sponge
            disp('sponge to rack')
            self.MoveRobot(trRack,way3,true,false); % sponge to rack
            
            disp('scrub motion')
            self.Scrub();
            
            disp('rack to sponge')
            self.MoveRobot(trSponge,way3,true,false); % rack to sponge
            disp('sponge to rack')
            self.MoveRobot(trRack,way3,false,false); % sponge to rack
            disp('rack to clean')
            self.MoveRobot(trClean,way4,false,true); % rack to clean
        end
        
        %%
        function MoveRobot(self,trGoal,trInit,sMarker,pMarker)
            steps = 30;
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
                    
                    
                end
                
            end
        end
        
        function Scrub(self)
            steps = 30;
            s = lspb(0,1,steps);
            %steps = deg2rad(30);
            qStart = self.robot.model.getpos();
            q1 = [qStart(1,1),qStart(1,2),qStart(1,3),qStart(1,4),qStart(1,5),qStart(1,6),-pi];
            q2 = [qStart(1,1),qStart(1,2),qStart(1,3),qStart(1,4),qStart(1,5),qStart(1,6),pi];
            
            
            for i = 1:steps
                %trSponge = trotz(i);
                self.qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
                trSponge = self.robot.model.fkine(self.qMatrix(i,:));
                self.sponge.MoveSponge(trSponge);
                
                self.qMatrix(i,:) = (1-s(i))*q2 + s(i)*q1;
                trSponge = self.robot.model.fkine(self.qMatrix(i,:));
                self.sponge.MoveSponge(trSponge);
            end
        end
        
        
        
        %% end of functions
    end
end

