function [ xdot ] = Dynamics_CL_Roller( t, x, Edges, C, Order, Target, BT, Node_Speed )
% Build the Jacobian
%Solve
n=length(x)/3;
% [R]=Rigidity_Matrix_Edges(Edges,reshape(x,n,3));
% Num_Steps=1;
% x_vec=x;
% [Angles, Grad]=Check_Angle_Constraints( Angles_Con, x_vec, Num_Steps ); %This is wrong somehow.
% R_Angle=Grad(1:2*n,:)';

%% The Controller for the Roller Modules

%Note that this is the very begining of some distributed control
L_cur=Pos_Tube_Oct( Edges, reshape(x,n,3) );%Compute the Position along the tube
Error=Target-L_cur;
%This is a speed, not a distance moved. This seems wrong.
% Close_Nodes=find(abs(Error)<Node_Speed);  %Identify if nodes are close to the target
% Command=-Node_Speed.*sign(Error);
% Command(Close_Nodes)=-Error(Close_Nodes);

thresh=.005*Node_Speed*1.01;
Close_Nodes=find(abs(Error)<thresh);  %Identify if nodes are close to the target
Command=-Node_Speed.*sign(Error);
Command(Close_Nodes)=-Error(Close_Nodes);

Ldot=BT*Command; %Shift to length changes
input_total=[Ldot; zeros(90-12,1)];
%%
% R_Angle=Grad';
%Also Need to Generate Order, the planarity joint.
% [ Output, Grad ] = Planartiy_Con( x_vec, Order(:,[ 2, 3, 4, 1]));
% R_Planar=Grad';   %This seems right!s

% [ Output, Grad2 ] = Planartiy_Con( x_vec, Order(:,[4, 5, 2, 1]));
% R_Planar2=Grad2'; 
% Get the Bisection Constraints
% [~, Grad_2]=Bisection_Constraint( x_vec, Order );
% [~, Grad_Angle]=Equal_Angle_Constraint_Shift(x_vec, Order);
% [~, Grad_Angle] = Planar_Bisection_Constraint( x_vec, Order );
% R_Bisect=Grad_2';
% R_Bisect=Grad_Angle';
% R_tot=[R; R_Bisect; R_Planar; R_Planar2]; %Generate the total planarity joint

R_tot=Get_Rtot( x, n, Edges, Order );
J=[R_tot; C];
xdot= J\input_total; %Solve this thing
% Generate the Angle Rigidity Matrix



% Num_Steps=1;
%The function assumes three dimensions, here only 2D is necessary, so we
%add the zeros for the z components
% x_vec=[reshape(x_tot,2*n,1); zeros(n,1)];  
% [Angles, Grad]=Check_Angle_Constraints( Angle_Vertices, x_vec, Num_Steps )
% R_Angle=Grad(1:2*n,:)';

% J=[R; R_Angle; C];
% Jinv=inv(J); %Probablxdy should use a bacslash operator instead



%% Determine the output
% input_total=zeros(12,1);
% Indices=[1, 2 , 4]; %The edges of R that coorespond to such thing
% input_total(Indices)=Ldot;
% xdot= J\input_total; %Solve this thing


end

