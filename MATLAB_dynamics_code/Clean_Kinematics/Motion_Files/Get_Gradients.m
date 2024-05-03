function [ c, ceq, dc, Gradient ] = Get_Gradients( x, Edges, C, Order )
%Compute the Gradients of all of the different things.
c=[];
dc=[];
n=length(x)/3;
x_mat=reshape(x,n,3);
Edge_Lengths=Get_Lengths_E( Edges, x_mat );
[R]=Rigidity_Matrix_Edges(Edges,reshape(x,n,3));
Num_Steps=1;
x_vec=x;
% [Angles, Grad]=Check_Angle_Constraints( Angles_Con, x_vec, Num_Steps ); %This is wrong somehow.
% R_Angle=Grad(1:2*n,:)';


% R_Angle=Grad';
%Also Need to Generate Order, the planarity joint.
[ Output, Grad ] = Planartiy_Con( x_vec, Order(:,[ 2, 3, 4, 1]));
R_Planar=Grad';   %This seems right!s

[ Output2, Grad2 ] = Planartiy_Con( x_vec, Order(:,[4, 5, 2, 1]));
R_Planar2=Grad2'; 
% Get the Bisection Constraints
% [~, Grad_2]=Bisection_Constraint( x_vec, Order );
% [~, Grad_Angle]=Equal_Angle_Constraint_Shift(x_vec, Order);
[Output_Bisect, Grad_Angle] = Planar_Bisection_Constraint( x_vec, Order );
% R_Bisect=Grad_2';
R_Bisect=Grad_Angle';
% R_tot=[R; R_Bisect; R_Planar; R_Planar2]; %Generate the total planarity joint
% J=[R_tot; C];
% xdot= J\input_total; %Solve this thing

ceq=[Edge_Lengths; Output; Output2; Output_Bisect];
Gradient=[R; R_Planar; R_Planar2; R_Bisect]';


end

