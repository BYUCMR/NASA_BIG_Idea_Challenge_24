function [ x_opt ] = Controled_Motion( t, x, Edges_All, Edge_All_Tube, Loop_Con, L2th, Order, A, b )
%Take in a desired motion. It will sovle for a velocity of each node
% that minimizes roller effort while satisfying all constraints

%The incoming A specifies the motion of all of the specified nodes

%Note! I need to import seperate Edge Lists for the Tubes and for the
%constraints
N_True=size(Edge_All_Tube,1);
n=length(x)/3;
R_tot=Get_Rtot( x, n, Edges_All, Order );
R=R_tot(1:N_True,:);
R_con=R_tot(N_True+1:end,:);
b_con=zeros(size(R_con,1),1);
%The constraint on the constant volume between true lengths
A_LoopCon=Loop_Con*R;
b_LoopCon=zeros(size(Loop_Con,1),1);

%Pick an Objective
Obj=[R];  %This objective is minimizing change in edge length, not necessarily roller.

%Augment with Angle Constraints
Aeq=[R_con; A_LoopCon; A]; 
beq=[b_con;     b_LoopCon; b];

% I can solve this thing with a matrix inverse, or using the quadprog
% solver
H=2*Obj'*Obj;  %I think I need factor fo two to get scaling right.

%Could solve using a matrix inverse...
% [x_lambda]=inv([H'*H, Aeq'; Aeq, zeros(size(Aeq,1),size(Aeq',2))])*[zeros(size(H,1),1); beq];

[x_opt,fval,exitflag,output]=quadprog(H,[],[],[],Aeq, beq); %Solve the quadratic program
% It seems like with only equality constraints this should be solved
% directly by forming the augmented lagrangian and finding the inverse
% [x_lambda(1:length(x_opt)), x_opt]
end

