function [ x_opt, Ldot, err ] = Controled_Motion_Nodes_Constraints( x, Edges_All, Loop_Con, L2th, A, b, L_norm, Cost_Index, Adj )
%Take in a desired motion. It will sovle for a velocity of each node
% that minimizes roller effort while satisfying all constraints

%The incoming A specifies the motion of all of the specified nodes

%Note! I need to import seperate Edge Lists for the Tubes and for the
%constraints
N_True=size(Edges_All,1);
n=length(x)/3;
R=Rigidity_Matrix_Edges(Edges_All,reshape(x,n,3));
A_LoopCon=Loop_Con*R;
b_LoopCon=zeros(size(Loop_Con,1),1);

%% Pick an Objective


switch Cost_Index
    
    case 1
        Obj=[R];  %This objective is minimizing change in edge length, not necessarily roller.
        % Obj=[L2th*R]; %Minimize the Motion of the Roller Nodes
        f=[];
        H=2*(Obj'*Obj);  %I think I need factor fo two to get scaling right.

    case 2
        %Formation Objective
        %Compute the Edge Lengths. 
        Lengths=Get_Lengths_E(Edge_All_Tube,reshape(x,n,3));
        L_norm_vec=Lengths; %Include other edges as constrained?
        L_norm_vec(1:size(Edge_All_Tube,1))=L_norm;
        Gain=100;
        dx=Gain*((Lengths-L_norm_vec)'*R)';  %Note that this is only for the tube ends.  I think that makes sense

        Mat=blkdiag(zeros(6),eye(n-6));
        H=blkdiag(Mat,Mat,Mat);
        f=dx;

    case 3  %Minimum Roller Motion?
        
        
end


% % Maintain equal Edge Lengths
% L=Get_Lengths_E(Edge_All_Tube, reshape(x,n,3)); %Compute the Length Vector
% L_norm=mean(L(1:3))*ones(size(Edge_All_Tube,1),1);
% dx_desired=(2*(L'-L_norm')*R)'; %How do I wan to manage this?
% Obj=eye(length(x));
% f=dx_desired;
% H=Obj;

%in some cases I could just solve with the matrix inverse

% %% Head Towards a Targt
% %This guy is messing things up if I only want to control part of the CoM,
% %obviously. 
% enable_Waypoints=true;
% if enable_Waypoints
%     b_des=(target-A(1:3,:)*x)/norm(target-A(1:3,:)*x)*speed;  %This only really works for CoM, not for other points
%     b(1:3)=b_des(1:3);
% end
%Probably need to do one of these for each point? If we want to match speed
%at least.
%%
%Augment with Angle Constraints
Aeq=[A; A_LoopCon]; 
beq=[b; b_LoopCon];

%Could solve using a matrix inverse...
% [x_opt]=inv([H'*H, Aeq'; Aeq, zeros(size(Aeq,1),size(Aeq',2))])*[zeros(size(H,1),1); beq];
% options =  optimoptions('Display','off');
% x0=Aeq\(-beq);
options = optimset('Display', 'off','MaxIter',1000); %'algorithm','trust-region-reflective');
x0=zeros(length(x),1);
Acon = [];
bcon = [];
% [x_opt,fval,exitflag,output]=quadprog(H,f,Acon, bcon, Aeq, beq,[],[],x0,options); %Solve the quadratic program
% It seems like with only equality constraints this should be solved
% directly by forming the augmented lagrangian and finding the inverse
% [x_lambda(1:length(x_opt)), x_opt]


% Integrate one step then check constraints
% If broken add constraints rerun optimization

% for every edge greater than Lmax append R(ind,:) to Acon
% 
% if edge length > threshhold:
%     set constraint where row in R()*x_dot <= 0
%     Acon = R(ind,:)

err = 0;
count = 0;

while true
    count = count + 1;
    
    [x_opt,fval,exitflag,output]=quadprog(H,f,Acon, bcon, Aeq, beq,[],[],x0,options); %Solve the quadratic program


    x_new = x_opt*0.001+x;
    L = Get_Lengths_E(Edges_All, reshape(x_new,n,3));
    
    temp = reshape(L,3,n-2);
    Triangle_Lengths = L'*Loop_Con';

    flag = 0;
    
    for i=1:length(L)
        if L(i) < 0.2
            flag = 1;
            disp("Too Short");
            Acon = [Acon; -R(i,:)];
            bcon = [bcon; 0];
        end
    end

    

    [Out, d_out] = Enforce_Ratio_Eig(Adj, x_new);
    
    if Out < 0.05
        flag = 1;
        Acon = [Acon; -d_out'];
        bcon = [bcon; 0.1*count];
    end

    if size(Acon,1) > 100
        disp("broken")
        exitflag=0;
    end
    if flag==0
        break;
    end
    
    
    if ~(exitflag==1)
        err = 1;
        disp(exitflag);
        disp(output);
        disp('Warning, Bad Solve')
        x_opt=zeros(length(x0),1); %If unable to solve, simply return 0s for the desired velocities.
        break;
    end

   
end

Out

 % for i=1:length(Triangle_Lengths)
    %     if Triangle_Lengths(i) > 3.01
    %         disp(Triangle_Lengths(i));
    %         m = max(temp(:,i));
    %         [row, col] = find(temp(:,i) == m);
    %         Acon = [Acon; (R((i-1)*3+row,:))]; %negative if too small
    %         bcon = [bcon; 0];
    %     end
    % end

L_new = Get_Lengths_E(Edges_All, reshape(x_new,n,3));


All_Inputs=R*x_opt;
Ldot=All_Inputs(1:size(Edges_All,1)); %Isolate the ones that refer to the edges

end

