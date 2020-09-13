classdef CUTE < handle
    properties 
        model;
        workspace = [-1 1 -1 1 -1 1];   
        
        jointStateTopic;
        jointStateSub;
        jointStates; 
        
        jointNames;
        jointTrajMsgType;
        jointTrajTopic;
        jointTrajPub;
        jointTrajMsg;
        steps;
    end
    
    methods
        
        function self = CUTE(useGripper)
            self.GetCUTEROBOT();
            
            self.jointStates = zeros(1,9);
            self.jointStateTopic = '/joint_states';
            %self.jointStateSub = rossubscriber(self.jointStateTopic, 'sensor_msgs/JointState');
            
            self.jointNames = ["joint1", ...
                               "joint2", ...
                               "joint3", ...
                               "joint4", ...
                               "joint5", ...
                               "joint6", ...
                               "joint7"];
                               
            self.jointTrajMsgType = 'trajectory_msgs/JointTrajectory';
            self.jointTrajTopic = '/cute_arm_controller/command';
            self.jointTrajMsg = rosmessage(self.jointTrajMsgType);
            self.jointTrajMsg.JointNames = self.jointNames;
            %self.jointTrajPub = rospublisher(self.jointTrajTopic,self.jointTrajMsgType);

            self.steps = 0;
        end
        
        function GetCUTEROBOT(self)
            pause(0.001);
            name = ['bob'];
            qlimH = [-2.5, 2.5];
            qlimV = [-1.8, 1.8];
            
            L1 = Link('d', 0.0872,'a', 0,'alpha', 0,'offset', -pi/2,'qlim', qlimH);
            L2 = Link('d', 0.0628,'a', 0,'alpha', pi/2,'offset', pi/2,'qlim', qlimV);
            L3 = Link('d', 0.07683,'a', 0,'alpha', -pi/2,'offset', 0,'qlim', qlimH);
            L4 = Link('d', 0.048827,'a', 0,'alpha', pi/2,'offset', 0,'qlim', qlimV);
            L5 = Link('d', 0.06663,'a', 0,'alpha', pi/2,'offset', pi/2,'qlim', qlimV);
            L6 = Link('d', 0.06663,'a', 0,'alpha', -pi/2,'offset', 0,'qlim', qlimV);
            L7 = Link('d', 0.027,'a', 0,'alpha', -pi/2,'offset', -pi/2,'qlim', qlimH);
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
            %self.model.base = self.model.base * transl(0,0,0.1);
        end
        %{
            L1 = Link('d', 0.0872,'a', 0,'alpha', pi,'offset', 0,'qlim', qlimH);
            L2 = Link('d', 0.0628,'a', 0,'alpha', -pi,'offset', 0,'qlim', qlimV);
            L3 = Link('d', 0.07683,'a', 0,'alpha', 0,'offset', 0,'qlim', qlimH);
            L4 = Link('d', 0.048827,'a', 0,'alpha', 0,'offset', 0,'qlim', qlimV);
            L5 = Link('d', 0.06663,'a', 0,'alpha', pi/2,'offset', 0,'qlim', qlimV);
            L6 = Link('d', 0.06663,'a', 0,'alpha', -1.8142,'offset', 0,'qlim', qlimV);
            L7 = Link('d', 0.027,'a', 0,'alpha', -1.2645,'offset', 0,'qlim', qlimH);
        %}
        
        function jointStates = getJointStates(self)
            self.updateJointStates();
            jointStates = self.jointStates;
        end
        
        function updateJointStates(self)
            %UPDATEJOINTSTATES Updates the robot joint states
            jointStateMsg = self.jointStateSub.LatestMessage;
            if ~isempty(jointStateMsg)
                self.jointStates = jointStateMsg.Position';
           end
        end
        
        function initPublisher(self, interpolationSteps)
            self.steps = interpolationSteps;
            for i = 1:self.steps
               jointTrajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint')
               self.jointTrajMsg.Points = [self.jointTrajMsg.Points; jointTrajPoint];
            end
        end
        
    end
end