robot = HANSCUTE();
%%
qStart = [-1.5 1.404 -1.55 0.108 0.252 1.404 0];%[-1.7 0.1779 -0.6378 1.7094 -0.3737 1.4760 -0.0108];

qGoal = [-1 1.404 -1.55 0.108 0.252 1.404 0];
%%[-0.8 0.1779 -0.6378 1.7094 -0.3737 1.4760 -0.0108];%

figure(1)
robot.model.teach(qStart);
%% Obtain Q Matrix and Pose Matrix
steps = 100;
T = robot.model.fkine(qGoal);
[qEnd, err, exitFlag] = robot.model.ikcon(T, qStart);
s = lspb(0,1,steps);  
qMatrix = nan(steps,7);
for j = 1:steps
 qMatrix(j,:) = (1-s(j))*qStart + s(j)*qEnd;
end

TrMatrix = zeros(4,4,steps);
for i = 1:size(qMatrix, 1)
   TrMatrix(:,:,i) = robot.model.fkine(qMatrix(i,:));
end

%%
errorMax = nan(6, steps-1);
errorMin = nan(6, steps);
deltaT = 0.05;

newQMatrixMax = zeros(size(qMatrix,1),7);
newQMatrixMax(1,:) = qMatrix(1,:);

newQMatrixMin = zeros(size(qMatrix,1),7);
newQMatrixMin(1,:) = qMatrix(1,:);
minManipMeasure = 0.1;

for i = 1:steps-1
    dq = qMatrix(i+1,:) - qMatrix(i,:); %obtain q joint difference 
    dqMax = max(dq);
    dqMin = min(dq);
    TrDotMax = (TrMatrix(:,:,i+1) - TrMatrix(:,:,i))/deltaT; % find pose difference then divide by max joint difference
    TrDotMin = (TrMatrix(:,:,i+1) - TrMatrix(:,:,i))/dqMin;
    
    dRDotMax = TrDotMax(1:3,1:3); % obtain rotation matrix from pose difference
    dRDotMin = TrDotMin(1:3,1:3);
    R = TrMatrix(1:3,1:3,i); % obtain current rotation matrix
    
    Smax = dRDotMax * R';
    Smin = dRDotMin * R';
    Smax = vex(Smax);
    Smin = vex(Smin);
    
    spacialVmax = [TrDotMax(1:3,4);Smax];
    spacialVmin = [TrDotMin(1:3,4);Smin];
    
    
    J = robot.model.jacob0(qMatrix(i,:));
    q = pinv(J) * spacialVmax;
    q = q';
    N = null(J);
    nspm = norm( J * N);
    
    errorMax(:,i) = spacialVmax - J*q';
    newQMatrixMax(i+1,:) = qMatrix(i,:) + dqMax*q + nspm;
    
    qMin = pinv(J) * spacialVmin;
    qMin = qMin';
    errorMin(:,i) = spacialVmin - J*qMin';
    newQMatrixMin(i+1,:) = qMatrix(i,:) + dqMin*qMin + nspm;
    
    m(:,i)= sqrt(det(J*J')); 
end
%%
for i = 1:1:size(newQMatrixMax, 1)
   robot.model.animate(newQMatrixMax(i,:)); 
end
%%
for i = 1:1:size(newQMatrixMax, 1)
   robot.model.animate(newQMatrixMin(i,:)); 
end
%%
figure(2)
plot(errorMin','Linewidth',1)
ylabel('Error Min (m/s)')
xlabel('Step')
legend('x-velocity','y-velocity','z-velocity','wx','wy','wz');
figure(3)
plot(errorMax','Linewidth',1)
ylabel('Error Max (m/s)')
xlabel('Step')
legend('x-velocity','y-velocity','z-velocity','wx','wy','wz');
figure(4)
plot(m,'k','LineWidth',1);                                                
title('Manipulability of Cyton')
ylabel('Manipulability')
xlabel('Step')

%% END
posDot = (TrMatrix(1:3,4,2) - TrMatrix(1:3,4,1))/deltaT;
skewMatrix = ((TrMatrix(1:3,1:3,2) * TrMatrix(1:3,1:3,1))' - 1) * deltaT

%% Resolve Motion Rate Control
dq = qMatrix(2,:) - qMatrix(1,:);
dqMax = max(dq);
dqMin = min(dq);
TrDotMax = (TrMatrix(:,:,2) - TrMatrix(:,:,1))/deltaT;
TrDotMin = (TrMatrix(:,:,2) - TrMatrix(:,:,1))/dqMin;

TrDot = (TrMatrix(:,:,2) - TrMatrix(:,:,1));

%%
dRDotMax = TrDotMax(1:3,1:3);
dRDotMin = TrDotMin(1:3,1:3);
R = TrMatrix(1:3,1:3,1);

Smax = dRDotMax * R';
Smin = dRDotMin * R';
Smax = vex(Smax);
Smin = vex(Smin);

spacialVmax = [TrDotMax(1:3,4);Smax];
spacialVmin = [TrDotMin(1:3,4);Smin];

deltaT = 0.05;
J = robot.model.jacob0(qMatrix(1,:));
q = pinv(J) * spacialVmax;
q = q';
N = null(J);
nspm = norm( J * N);
newQMatrix = qMatrix(1,:) + q + nspm

%{
%Jarm = J(:,1:6)
m(:,i)= sqrt(det(Jarm*Jarm'));       

%qDot = inv(Jarm) * spacialVmax;
%error = spacialVmax - Jarm*qDot;
newQMatrix = qMatrix(1,1:6) + deltaT * qDot';
qMatrix(1,:);
qMatrix(2,:);
%}