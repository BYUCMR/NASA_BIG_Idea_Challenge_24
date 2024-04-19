clc
clear
addpath(genpath(pwd))
%%

%Add three edges that hang down from this thing and enable me to control
%it?  That woul dbe ideal...
%ALso pivot such that this thing works differently

Nodes=[3 4 5;
        1 6 4;
        1 3 2];
in2m=2.54/100
d_space=2.5*in2m;  %The spacing between the two rollers
d_offset=10*in2m;   %The distance between the kinematic joint and tube
d_offset_normal=0*in2m; %The Offset to Move Normal to the plane of the tube
d_offset_base=2*in2m;   %Offset of how big the box is. 
L_tube=134*in2m;

% [ x_temp, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All ] = Generate_Shape_Order( Nodes, d_space, d_offset, d_offset_normal, d_offset_base, L_tube );
[ x_temp, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All, n_true, Edges_True ] = Generate_Shape_Order( Nodes, d_space, d_offset, d_offset_normal, d_offset_base, L_tube );

x_all=x_temp;

r2=atan2(x_temp(4,3)-x_temp(1,3), x_temp(4,2)-x_temp(1,2))+pi
R_x=[1 0 0; 
    0 cos(r2) sin(r2);
    0 -sin(r2) cos(r2)];
x_temp= (R_x* x_temp')'
x_all=x_temp
        % x_all(:,2)=x_temp(:,3);
        % x_all(:,3)=-x_temp(:,2);
        % x_all(:,3)=x_all(:,3)-min(x_all(:,3));
x_all(:,3)=x_temp(:,3)-min(x_temp(:,3))
    n_all=size(x_all,1);
    close all
    Indices_Tube=ones(size(Edges_Tube,1));
    Indices_Con=2*ones(size(Edges_Con,1));
%     Plot_Robot_Edges(Edges_Tube,reshape(x_all,n_all,3),Indices_Tube, .10);
%     Plot_Robot_Edges(Edges_Con,reshape(x_all,n_all,3),Indices_Con, .05);
% %     hold(Find_Result,'on')
% %     plot3(x_all(1:n,1),x_all(1:n,2),x_all(1:n,3),'o')
% axis equal

Order=T2T_Qual;
L2th=inv(B_T'*B_T)*B_T'; %The psuedo inverse NOT SURE IF THIS IS CORRECT!
Edges_All=[Edges_Tube; Edges_Con]
Edges_All_Tube=Edges_Tube;
%I should plot the passive nodes differently? 

%Define which nodes are the passive nodes.  The trick is they are not true
%kinematic nodes.  They are something else. 
for i=1:length(Edges_True)
    Passive(i)=Edges_True{i}(1,1); %Assume the first thing in the cycle is passive.
end
% Passive=[1,3]; %Note that this is implicit in how I defined the tube routing with Initialize_Octahedron
n_all=size(x_all,1);
N_Edges_True=size(Edges_Tube,1);

%Information for plotting the nodes, doesn't directly control kinematics
    d_up=3.25*in2m;  %Distance from the center of the node
    d_down=1.15*in2m;
    d_length=8/2*in2m;
    d_width=5/2*in2m;

% Show a Diagram of the Robot with things labeled

figure
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
xlabel('x')
hold on
Number_Nodes( x_all(1:n_true,:))
axis equal
title('Nodes')

%% %Show a Diagram of the Robot with things labeled

figure
subplot 121  
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
axis equal
hold on

for i=1:size(Edges_All,1)
    Node1=x_all(Edges_All(i,1),:);
    Node2=x_all(Edges_All(i,2),:);
    Center=(Node1+Node2)/2;
    text(Center(1),Center(2),Center(3),num2str(i))
end
title('Edges')

%
subplot 122
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
hold on
Number_Nodes( x_all )
axis equal
title('Nodes')

%Prepare a Plotting-level Figure

%% Define Contact with the Environment
Support=[9 10 15]; %The Support Polygon Base
% Support=[1, 27, 18]; %For some reason part of this doesn't work, which seems wrong.
C=zeros(6,3*n_all);  %6 Constraints needed in this case. 
C(1,Support(1))=1; %Fix Everything on the First Point
C(2,Support(1)+n_all)=1;
C(3,Support(1)+2*n_all)=1;
C(4,Support(2)+2*n_all)=1;
C(5,Support(2)+n_all)=1;
C(6,Support(3)+2*n_all)=1;
rank(C)

C_hist(:,:,1)=C;

%% Assign Mass Properties

m_meausured=2.83;    %Mass of an active roller module
m_passive=1.6;       %Mass of a passive roller

%Determine the Center of Mass
m_node=ones(1,n_all-n_true)*m_meausured/2; %Assume all other nodes have arbitrary mass
m_kin=zeros(1,n_true); %Assume the joints have no mass

m_tot=[m_kin,m_node];

%This is assuming that the two passive nodes are together. I can probably
%just assume equal lengths for this. 
for it=1:length(Passive)
   Neighbors=find(Edges_Con(:,1)==Passive(it));
   for j=1:length(Neighbors)    %Hardcoding the Fact that Each of these nodes has four neighbros
       m_tot(Edges_Con(Neighbors(j),2))=m_passive/2;       
   end
end

M=(m_tot/sum(m_tot));
CoM=M*x_all;
Z=zeros(1,n_all);

M_Mat=[M, Z, Z; 
       Z, M, Z;
       Z, Z, M];

y_tot=reshape(x_all,3*n_all,1)';  %Do rows or columns?   Right now I have each time step being a row.

%% Determine the Nominal Length of Each Edge
Lengths=Get_Lengths_E(Edges_All_Tube,x_all);
L_norm=Lengths(1); %The optimum length for the triangles %Set the Default Length

%% Determine the Graph Laplacian of the robots (not just of the points themselves)

N_robots=n_true;  %The number of actual robots

%This part here is assuming every node has 

for i=1:N_robots
   First=find(Order(:,1)==i);
   Local{i}=i;
   for j=1:length(First)
        Local{i}=[Local{i} Order(First(j),[2,4])];
   end
%    Row_Coor(i,:)=find(Order(:,1)==i); %Which rows of order coorpesond? 
end

%Generate the Robot Laplacian Elsewhere.


% for i=1:size(Edges_Tube,1)
%         [a, b]=find(Local==Edges_Tube(i,1));
%         [c, d]=find(Local==Edges_Tube(i,2));
%         Edge_R2R(i,:)=[a,c];   %Isn't this just the graph of the initial robot?
% %         Edges_T_L{a}=[Edges_T_L{a}; i];
% %         Edges_T_L{c}=[Edges_T_L{c}; i];
% end
Edge_R2R=[];
for i=1:length(Edges_True)
    Edge_R2R=[Edge_R2R; Edges_True{i}];
end
Adj=zeros(N_robots);
for i=1:size(Edge_R2R,1)
    Adj(Edge_R2R(i,1),Edge_R2R(i,2))=1;
end
Adj=Adj+Adj';
L_robots=diag(sum(Adj))-Adj;


%Generate the "Local" nodes (which nodes are the responsibility of each
%robot)

%% Main Loop



count=1; %A number to count the number of iterations
tic




figure
Node_Pos_All=zeros(1,size(B_T,2));
t_vec=0;

%% Move 1

Support=[9, 10, 15];
C_Lock=zeros(length(Support)*3,3*n_all);
for i=1:length(Support)
        C_Lock(3*(i-1)+1,Support(i))=1;
        C_Lock(3*(i-1)+2,Support(i)+n_all)=1;
        C_Lock(3*(i-1)+3,Support(i)+2*n_all)=1;
end

% Move_Con=zeros(3,3*n_all);
% Move_Con(1,2*n_all+1)=1;
% Move_Con(2,2*n_all+3)=1;
% Move_Con(3,2*n_all+4)=1;
% b_con=[0; 0; 0];

Nodes_Move=[1 3 4]; %I could add more edges, or jsut adjust this...
Move_Con=zeros(3*length(Nodes_Move),3*n_all);
for i=1:length(Nodes_Move)
        Move_Con(3*(i-1)+1,Nodes_Move(i))=1;
        Move_Con(3*(i-1)+2,Nodes_Move(i)+n_all)=1;
        Move_Con(3*(i-1)+3,Nodes_Move(i)+2*n_all)=1;
end
b_con=repmat([0; 0; -1],[3 1]);

A_move{1}=Move_Con;
A_lock{1}=C_Lock;
b_move{1}=b_con; 
b_lock{1}=zeros(size(A_lock{1},1),1);
t_move{1}=.5;

%% Move 1- Center of Mass
Support=[9, 10, 15];
C_Lock=zeros(length(Support)*3,3*n_all);
for i=1:length(Support)
        C_Lock(3*(i-1)+1,Support(i))=1;
        C_Lock(3*(i-1)+2,Support(i)+n_all)=1;
        C_Lock(3*(i-1)+3,Support(i)+2*n_all)=1;
end

% Move_Con=zeros(3,3*n_all);
% Move_Con(1,2*n_all+1)=1;
% Move_Con(2,2*n_all+3)=1;
% Move_Con(3,2*n_all+4)=1;
% b_con=[0; 0; 0];

Nodes_Move=[1 3 4]; %I could add more edges, or jsut adjust this...
Move_Con=zeros(3*length(Nodes_Move),3*n_all);
for i=1:length(Nodes_Move)
        Move_Con(3*(i-1)+1,Nodes_Move(i))=1;
        Move_Con(3*(i-1)+2,Nodes_Move(i)+n_all)=1;
        Move_Con(3*(i-1)+3,Nodes_Move(i)+2*n_all)=1;
end
b_con=repmat([0; 0; 1],[3 1]);

A_move{2}=Move_Con;
A_lock{2}=C_Lock;
b_move{2}=b_con; 
b_lock{2}=zeros(size(A_lock{1},1),1);
t_move{2}=1;

%% Move 2- Left
Support=[9, 10, 15];
C_Lock=zeros(length(Support)*3,3*n_all);
for i=1:length(Support)
        C_Lock(3*(i-1)+1,Support(i))=1;
        C_Lock(3*(i-1)+2,Support(i)+n_all)=1;
        C_Lock(3*(i-1)+3,Support(i)+2*n_all)=1;
end

Nodes_Move=[1 3 4]; %I could add more edges, or jsut adjust this...
for i=1:length(Nodes_Move)
        Move_Con(3*(i-1)+1,Nodes_Move(i))=1;
        Move_Con(3*(i-1)+2,Nodes_Move(i)+n_all)=1;
        Move_Con(3*(i-1)+3,Nodes_Move(i)+2*n_all)=1;
end
b_con=repmat([0; -1; 0],[3 1]);
% b_con=[1; 1; 1];

A_move{3}=Move_Con;
A_lock{3}=C_Lock;
b_move{3}=b_con; 
b_lock{3}=zeros(size(A_lock{1},1),1);
t_move{3}=.4;

%% Move 2- Down
Support=[9, 10, 15];
C_Lock=zeros(length(Support)*3,3*n_all);
for i=1:length(Support)
        C_Lock(3*(i-1)+1,Support(i))=1;
        C_Lock(3*(i-1)+2,Support(i)+n_all)=1;
        C_Lock(3*(i-1)+3,Support(i)+2*n_all)=1;
end

Nodes_Move=[1 3 4]; %I could add more edges, or jsut adjust this...
for i=1:length(Nodes_Move)
        Move_Con(3*(i-1)+1,Nodes_Move(i))=1;
        Move_Con(3*(i-1)+2,Nodes_Move(i)+n_all)=1;
        Move_Con(3*(i-1)+3,Nodes_Move(i)+2*n_all)=1;
end
b_con=repmat([0; 0; -1],[3 1]);
% b_con=[1; 1; 1];

A_move{4}=Move_Con;
A_lock{4}=C_Lock;
b_move{4}=b_con; 
b_lock{4}=zeros(size(A_lock{1},1),1);
t_move{4}=.5;

%% Move 3- Hand

Support=[9, 10, 15];
C_Lock=zeros(length(Support)*3,3*n_all);
for i=1:length(Support)
        C_Lock(3*(i-1)+1,Support(i))=1;
        C_Lock(3*(i-1)+2,Support(i)+n_all)=1;
        C_Lock(3*(i-1)+3,Support(i)+2*n_all)=1;
end

Node_Move=[1 3 4];
C_Move=zeros(length(Node_Move)*3,3*n_all);
for i=1:length(Node_Move)
        C_Move(3*(i-1)+1,Node_Move(i))=1;
        C_Move(3*(i-1)+2,Node_Move(i)+n_all)=1;
        C_Move(3*(i-1)+3,Node_Move(i)+2*n_all)=1;
end

b_con=repmat([0; 0; 1],[3 1]);

t_move_1=.5;
Movement_i=5;
A_move{Movement_i}=[C_Move];
A_lock{Movement_i}=C_Lock;
b_move{Movement_i}=[b_con]; 
b_lock{Movement_i}=zeros(size(A_lock{Movement_i},1),1);
t_move{Movement_i}=.5;


%%

        dt=.01;

        tic
N_Moves=length(A_move);
Cost_Fun_Index=2;
for Movement_i=1:N_Moves

        for it_t=1:length(0:dt:t_move{Movement_i})

            %% Extract the Values from the GUI
%             data=guidata(obj);
%             result=data.popupmenu4;
%             Node_Control=result.Value;
%             Node_Index = str2num(data.Node_Control_Edit.String);
%             Cost_Fun_Index=data.popupmenu5.Value;
%             Cent_Decentralized_On=data.popupmenu6.Value; %Distributed or Centralized Computation
%             Turn_On_Broadcast=data.broadcast_command.Value;
        %     Node_Control=0;  %For now Hardcoded (picks the node, or the center of mass)

            A=[ A_move{Movement_i}; A_lock{Movement_i}];  
            b=[ b_move{Movement_i}; b_lock{Movement_i}];

            Loop_Con=[];
            for i=1:N_subsections
                Elements=3;
                Loop_Con=blkdiag(Loop_Con, ones(1,Elements));
            end

        %% Compute the required Motion (either centralized or decentralized)

        %Do the Bookkeeping for the state vectors.

        %Choose whether to use centralized or decentralized control
        
        Cent_Decentralized_On=1;
        if Cent_Decentralized_On==1
            [ x_opt, Ldot ] = Controled_Motion_Nodes( y_tot(end,:)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Fun_Index);
        else
            %Compute the decentralized control
            [ x_opt, Ldot ] = Controled_Motion_Nodes_Dist_Any(y_tot(end,:)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Fun_Index, L_robots, Local, Passive);

        %     [ x_opt, Ldot ] = Controled_Motion_Nodes_Dist( y_tot(end,:)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b, L_norm, Cost_Fun_Index);
        end

        %From the Desired Edge Motions, extract the Roller Changes
        %THis L2th encodes which nodes are passive and which are not. 
        Command=L2th*Ldot;  %Need to do this with local information
        %It looks like rollers 1 and 2 are the ground rollers.

        %Can Potentially simulate unrecognized failure of the rollers
        % Command(6)=0;
        %Could Potentially turn off the Commands

        %Play Back the Commands through the Rollers
%         [xdot] = Dynamics_Rollers_L( y_tot(end,:)', Edges_All, A_lock{Movement_i}, Order, Ldot );
        xdot=x_opt;  %Cheating, but fixes some constraints issues
        %% Estimate the next position. 

        %Perform the Euler Integration Update (Does this need to account for the
        %real time gap?  In practice, I could measure the new state, as opposed to
        %compute it through forward propogation. 

        y_tot=[y_tot; xdot'*dt+y_tot(end,:)];
        %Could extract some closed loop position information here
%         t_vec(count+1)=t_vec(end)+dt;
%         command_hist(count+1,:)=[Command_Normalized];
%         Node_Control_hist(count+1)=[Node_Control];
        %Determine the New Node Positions
        Node_Pos_All=[Node_Pos_All; Command'*dt+Node_Pos_All(end,:)];
%         %

        count=count+1;

        end

end
%%

% Some Resorting probably needed to get this to work. 

%Update a display in the GUI with the frequency? 

%% Show all of the plotting information
% plot(t_vec, M_M)
%Add plots for all of the nodes as well to show the motions? 

%% Some Plotting after runnign the simulation


figure
Plot_Edges(Edges_All, reshape(y_tot(1,:)',n_all,3),'-k')  %All of the Required Edges
axis equal
hold on
Plot_Edges(Edges_All, reshape(y_tot(end,:)',n_all,3),'-r')  %All of the Required Edges

n=size(x_all,1)
        xmin=min(min(y_tot(:,1:n))) ;
        xmax=max(max(y_tot(:,1:n)))  ;
        ymin=min(min(y_tot(:,n+1:2*n)));
        ymax=max(max(y_tot(:,n+1:2*n)));
        zmin=min(min(y_tot(:,2*n+1:end)));
        zmax=max(max(y_tot(:,2*n+1:end)));

%% Show the Low Resolution Video of this
figure
node_plot=5;
for i=1:1:size(y_tot,1)
    clf
    Plot_Edges(Edges_All, reshape(y_tot(i,:)',n_all,3),'-r')
    title(num2str(i))
    axis equal
    hold on
    axis([xmin,xmax,ymin,ymax,zmin,zmax])
    plot3(y_tot(:,node_plot),y_tot(:,node_plot+n_all), y_tot(:,node_plot+2*n_all ))

    drawnow
    
end

%% Generate the High Resolution Video of this.
clear F
figure('position',[100 100 1000 650])

%%
Frames=1:10:size(y_tot,1);
count=1;
for i_mov=length(Frames) %:length(Frames)
        clf
        y_vec_R=y_tot(Frames(i_mov),:)';

        Plot_Robot_Shadow( Edges_Tube, reshape(y_vec_R,n_all,3), .07, [65/255  105/255 225/255] )
        [Edge_Con, x_conPlot]=Plot_Connections( y_vec_R, T2T_Qual );
        Plot_Robot_Joint_Shadow(Edge_Con,x_conPlot,ones(size(T2T_Qual,1),1),.025);



        % Plot_Robot_Edges_Color_Spec( Edges_Tube, reshape(y_vec_R,n_all,3), .07, [65/255  105/255 225/255] )
        % [Edge_Con, x_conPlot]=Plot_Connections( y_vec_R, T2T_Qual );
        % Plot_Robot_Joint_Edges(Edge_Con,x_conPlot,ones(size(T2T_Qual,1),1),.025);
        
        %Plot the Node/Node Connection Joints with Spheres? 
        Plot_Node_Boxes(y_vec_R, T2T_Qual, d_up, d_down, d_length, d_width );
        box off
        grid off
        axis off
        axis equal
        axis([xmin,xmax,ymin,ymax,zmin,zmax])
        Ground=[xmin ymin; xmax ymin; xmax ymax; xmin ymax];
        patch(Ground(:,1), Ground(:,2),zmin*ones(length( Ground(:,2)),1),[.9 .9 .9]);

        lightangle(18,41)
        h.FaceLighting = 'gouraud';
        h.AmbientStrength = 0.3;
        h.DiffuseStrength = 0.8;
        h.SpecularStrength = 0.9;
        h.SpecularExponent = 25;
        h.BackFaceLighting = 'unlit';
        
        F(count)=getframe(gcf);
        count=count+1;
end

%%  Save the Resulting Movie
SAVEMOVIE=1;
if SAVEMOVIE==1
    v = VideoWriter('BYU_Robot_Crane');
    v.Quality = 100;    % Default 75, but I think this only applies to compressed videos
    v.FrameRate = 30;
    open(v)
    writeVideo(v,F)
    close(v)
end



% subplot 211  %There may be something weird about where the plotting goes to smooth out the timing.
% plot([0,t_vec])
% subplot 212
% plot(1:size(Node_Pos_All,1),Node_Pos_All')

%% Experiment with sorting and unsorting the commands
% 
% %Commands_Data
% Sent_Commands=[0  0  0  0 0  0   0  0;
%                0 -7 -7 12 0  6   7 -14;
%                0 4  4 -9  0 -5  -4 9;
%                0 -6 4 -9  0  5   1 4;
%                0  0 0  0  0  0   0 0];
%            
% %Some weirdness here about this. 
% Command_sort=[5, 1, 8, 2, 6, 4, 7, 3];  %What does this vector do exactly?
% 
% Sent_Commands_Sim=Sent_Commands(2,Command_sort);

%%
% Command_Rollers=1:8;
% 
% Command_Rollers_Switch=Command_Rollers(Command_sort);
% 
% % Here is how to switch the order of these things
% test(Command_sort)=Command_Rollers;
% 
% test(Command_sort)


%%
% save('Simulation_Run_2')








