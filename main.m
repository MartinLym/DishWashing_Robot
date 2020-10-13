clc
clf
clear all

r = RobotControl();
% disp('Press enter')
% pause();

%%
 r.SimulateRobot();

%%
clc
clf
clear all

workspace = [-1 1 -1 1 -1 1];
scale = 0.25;
qStart = deg2rad([-81,0,-143,30.9,47.4,49,0]);
r = RobotControl();

r.robot.model.plot(qStart,'workspace',workspace,'scale',scale,'nowrist');
hold on;
r.robot.model.teach();


%% 
clc
clf
clear all
r = RobotControl();
r.RetreatVS();
%%
x = 0.3;
y = 0;
z = 0.2;
r.retreat.SetPointsVS(x,y,z);
%%
r.retreat.SetPointsVS(x,y,z);
r.retreat.InitialiseVS();
r.retreat.PlotPointsVS();
r.retreat.AnimateVS(); 

%%
clc
clf 
clear all
r = HansCute();
            q0 = [0,0,0,pi/2,0,0,0];
             r.model.animate([0,0,0,0,0,0,0]);
            qCurrent = r.model.getpos();
            self.Tc0= r.model.fkine(q0);           
            r.model.animate(jtraj(qCurrent,q0,50));
