function [ x_opt, Ldot ] = Controled_Motion_Waypoints( t, x, Edges_All, Edge_All_Tube, Loop_Con, L2th, Order, A, b, target, speed )
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
% Obj=[L2th*R]; %Minimize the Motion of the Roller Nodes
f=[];
H=2*(Obj'*Obj);  %I think I need factor fo two to get scaling right.

% % Maintain equal Edge Lengths
% L=Get_Lengths_E(Edge_All_Tube, reshape(x,n,3)); %Compute the Length Vector
% L_norm=mean(L(1:3))*ones(size(Edge_All_Tube,1),1);
% dx_desired=(2*(L'-L_norm')*R)'; %How do I wan to manage this?
% Obj=eye(length(x));
% f=dx_desired;
% H=Obj;

%in some cases I could just solve with the matrix inverse

%% Head Towards a Targt
%This guy is messing things up if I only want to control part of the CoM,
%obviously. 
enable_Waypoints=true;
if enable_Waypoints
    b_des=(target-A(1:3,:)*x)/norm(target-A(1:3,:)*x)*speed;  %This only really works for CoM, not for other points
    b(1:3)=b_des(1:3);
end
%Probably need to do one of these for each point? If we want to match speed
%at least.
%%
%Augment with Angle Constraints
Aeq=[R_con; A_LoopCon; A]; 
beq=[b_con;     b_LoopCon; b];

%Could solve using a matrix inverse...
% [x_opt]=inv([H'*H, Aeq'; Aeq, zeros(size(Aeq,1),size(Aeq',2))])*[zeros(size(H,1),1); beq];
% options =  optimoptions('Display','off');
% x0=Aeq\(-beq);
options = optimset('Display', 'off','MaxIter',10000); %'algorithm','trust-region-reflective');
x0=zeros(length(f),1);
[x_opt,fval,exitflag,output]=quadprog(H,f,[],[],Aeq, beq,[],[],x0,options); %Solve the quadratic program
% It seems like with only equality constraints this should be solved
% directly by forming the augmented lagrangian and finding the inverse
% [x_lambda(1:length(x_opt)), x_opt]
if ~(exitflag==1)
    output.message
    disp('Warning, Bad Solve')
end
All_Inputs=R_tot*x_opt;
Ldot=All_Inputs(1:12); %Isolate the First 12 of these.

end

