function [ xdot ] = Controller_Roller_Combo( t, x, Edges_All, Edge_All_Tube, Loop_Con, L2th, Order, A, b, target, speed, BT, C )
%Intake Desired Node Positions, convert to Roller Commands, Simulate
%Dynamics with Roller Commands

%Use the Controller
%From a few specified motions, determine how all nodes and edges should
%change
[ x_opt, Ldot ] = Controled_Motion_Waypoints( t, x, Edges_All, Edge_All_Tube, Loop_Con, L2th, Order, A, b, target, speed );

%From the Desired Edge Motions, extract the Roller Changes
Command=L2th*Ldot;
%It looks like rollers 1 and 2 are the ground rollers.

%Can Turn off Rollers at this point
Command(6)=0;
%Could Potentially turn off the Commands

%Play Back the Commands through the Rollers
[xdot] = Dynamics_Rollers( t, x, Edges_All, C, Order, BT, Command );

%Check to make sure that the commands match
Check=max(abs(x_opt-xdot));

end

