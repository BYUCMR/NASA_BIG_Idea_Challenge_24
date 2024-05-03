function [c, g_c ceq, g_ceq ] = Get_Constraints_Position( x,  Edges, C, Order )
%Move to Optimizing configurations. Check the Gradient of Thing

%Generate the values 
n=length(x)/3;
[R]=Rigidity_Matrix_Edges(Edges,reshape(x,n,3));
Num_Steps=1;
x_vec=x;
% [Angles, Grad]=Check_Angle_Constraints( Angles_Con, x_vec, Num_Steps ); %This is wrong somehow.
% R_Angle=Grad(1:2*n,:)';


% R_Angle=Grad';
%Also Need to Generate Order, the planarity joint.
[ Output1, Grad1 ] = Planartiy_Con( x_vec, Order(:,[ 2, 3, 4, 1]));
R_Planar=Grad1';   %This seems right!s

[ Output2, Grad2 ] = Planartiy_Con( x_vec, Order(:,[4, 5, 2, 1]));
R_Planar2=Grad2'; 
% Get the Bisection Constraints
% [~, Grad_2]=Bisection_Constraint( x_vec, Order );
% [~, Grad_Angle]=Equal_Angle_Constraint_Shift(x_vec, Order);
[Output3, Grad_Angle] = Planar_Bisection_Constraint( x_vec, Order );
% R_Bisect=Grad_2';
R_Bisect=Grad_Angle';
% R_tot=[R; R_Bisect; R_Planar; R_Planar2]; %Generate the total planarity joint
% J=[R_tot; C];
% xdot= J\input_total; %Solve this thing

Result=[Output1; Output2; Output3];
g_ceq=[Grad1, Grad2, Grad_Angle];

end

