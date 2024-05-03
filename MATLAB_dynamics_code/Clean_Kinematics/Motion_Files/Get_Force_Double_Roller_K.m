function [ Torque ] = Get_Force_Double_Roller_K( x, Edges, C, Order, F)
%Get the Force on the Given Edges for a particular configuration.

%The Force is the Force Vector applied at each node

n=length(x)/3;

R_tot=Get_Rtot( x, n, Edges, Order );

% [R]=Rigidity_Matrix_Edges(Edges,reshape(x,n,3));
% % [Angles, Grad]=Check_Angle_Constraints( Angles_Con, x_vec, Num_Steps ); %This is wrong somehow.
% % R_Angle=Grad(1:2*n,:)';
% 
% 
% % R_Angle=Grad';
% %Also Need to Generate Order, the planarity joint.
% [ ~, Grad ] = Planartiy_Con( x_vec, Order(:,[ 2, 3, 4, 1]));
% R_Planar=Grad';   %This seems right!s
% 
% [ ~, Grad2 ] = Planartiy_Con( x_vec, Order(:,[4, 5, 2, 1]));
% R_Planar2=Grad2'; 
% % Get the Bisection Constraints
% % [~, Grad_2]=Bisection_Constraint( x_vec, Order );
% % [~, Grad_Angle]=Equal_Angle_Constraint_Shift(x_vec, Order);
% [~, Grad_Angle] = Planar_Bisection_Constraint( x_vec, Order );
% % R_Bisect=Grad_2';
% R_Bisect=Grad_Angle';
% R_tot=[R; R_Bisect; R_Planar; R_Planar2]; %Generate the total planarity joint
J=inv([R_tot; C]);

% J=inv([R_tot; C]);
%Generate the Resulting Torques
Torque=J'*F;

% n=length(x)/3;
% [R]=Rigidity_Matrix_Edges(Edges,reshape(x,n,3));
% Num_Steps=1;
% x_vec=x;
% [~, Grad]=Check_Angle_Constraints( Angles_Con, x_vec, Num_Steps ); %This is wrong somehow.
% % R_Angle=Grad(1:2*n,:)';
% R_Angle=Grad';
% %Also Need to Generate Order, the planarity joint.
% [ ~, Grad ] = Planartiy_Con( x_vec, Order);
% R_Planar=Grad';   %This seems right!s
% R_tot=[R; R_Angle; R_Planar]; %Generate the total planarity joint
% % J=[R_tot; C];
% 
% J=inv([R_tot; C]);
% Torque=J'*F;

end

