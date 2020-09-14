classdef RobotControl
    
    properties
        cute;
        
        qMatrix;
    end
    
    methods
        function self = RobotControl()
            self.cute = Cute();
        end
        
        %%
        function SimulateRobots(self)
           self.MoveRobot(); 
        end
        
        %%
        function MoveRobot(self,qInit,qGoal,steps)
            s = lspb(0,1,steps);
            self.qMatrix = ones(steps,7);
            
            qStart = self.cute.model.getpos();
            qEnd = self.cute.model.ikcon(qGoal,qInit);
            
            for i = 1:steps
                self.qMatrix(i,:) = (1-s(i))*qStart + s(i)*qEnd;
                self.cute.modelanimate(self.qMatrix(i,:))
            end
        end
        
        %% end of functions
    end
end

