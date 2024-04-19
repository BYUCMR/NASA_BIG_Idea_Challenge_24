function [ xdot ] = Dynamics_Rollers( x, Edges, C, Order, BT, Command )
% Build the Jacobian
%Solve
n=length(x)/3;

Ldot=BT*Command; %Shift to length changes
input_total=[Ldot; zeros(90-12,1)];

R_tot=Get_Rtot( x, n, Edges, Order );
J=[R_tot; C];
xdot= J\input_total; %Solve this thing

end

