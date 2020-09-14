clc
clf
clear all

workspace = [-1 1 -1 1 -1 1];
scale = 0.15;
q = [0,0,0,0,0,0,0];
r = RobotControl();

r.cute.model.plot(q,'workspace',workspace,'scale',scale,'nowrist');
hold on;
r.cute.model.teach();

% s = Environment();
% s.MoveSponge(transl(0,0.5,0));