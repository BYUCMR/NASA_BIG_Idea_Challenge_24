function [ R_tot ] = Get_Rtot( x, n, Edges, Order )
%Generate the R_tot matrix, which can then be combined with C to get the
%Inverted Jacobian

x_vec=x;
[R]=Rigidity_Matrix_Edges(Edges,reshape(x,n,3));
% [Angles, Grad]=Check_Angle_Constraints( Angles_Con, x_vec, Num_Steps ); %This is wrong somehow.
% R_Angle=Grad(1:2*n,:)';


% R_Angle=Grad';
%Also Need to Generate Order, the planarity joint.
[ ~, Grad ] = Planartiy_Con( x_vec, Order(:,[ 2, 3, 4, 1]));
R_Planar=Grad';   %This seems right!s

[ ~, Grad2 ] = Planartiy_Con( x_vec, Order(:,[4, 5, 2, 1]));
R_Planar2=Grad2'; 
% Get the Bisection Constraints
% [~, Grad_2]=Bisection_Constraint( x_vec, Order );
% [~, Grad_Angle]=Equal_Angle_Constraint_Shift(x_vec, Order);
[~, Grad_Angle] = Planar_Bisection_Constraint( x_vec, Order );
% R_Bisect=Grad_2';
R_Bisect=Grad_Angle';
R_tot=[R; R_Bisect; R_Planar; R_Planar2]; %Generate the total planarity joint
% J=inv([R_tot; C]);

end

