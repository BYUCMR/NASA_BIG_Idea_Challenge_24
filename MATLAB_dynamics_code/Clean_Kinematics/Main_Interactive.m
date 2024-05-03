clc
clear

%Ideas:  Add physical colors to the node connections to make them more
%obvious? 

%The File that Runs the Meat of the simulation while the GUI is active

addpath(genpath(pwd))
in2m=2.54/100;


%% Initialize the Robot

%Geometry of the Nodes
d_space=2.5*in2m;  %The spacing between the two rollers
d_offset=10*in2m;   %The distance between the kinematic joint and tube
d_offset_normal=0*in2m; %The Offset to Move Normal to the plane of the tube
d_offset_base=2*in2m;   %Offset of how big the box is. 
L_tube=134*in2m;
[ x_all, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All ] = Initialize_Octahedron(d_space, d_offset, d_offset_normal, L_tube  );
Order=T2T_Qual;
L2th=inv(B_T'*B_T)*B_T'; %The psuedo inverse
Edges_All=[Edges_Tube; Edges_Con]
Edges_All_Tube=Edges_Tube;
Passive=[1,3]; %Note that this is implicit in how I defined the tube routing with Initialize_Octahedron

n_all=size(x_all,1);
N_Edges_True=size(Edges_Tube,1);

%Information for plotting the nodes, doesn't directly control kinematics
    d_up=3.25*in2m;  %Distance from the center of the node
    d_down=1.15*in2m;
    d_length=8/2*in2m;
    d_width=5/2*in2m;


%% %Show a Diagram of the Robot with things labeled

figure
subplot 121  
Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
axis equal
hold on

for i=1:size(Edges_All,1)
    Node1=x_all(Edges_All(i,1),:);
    Node2=x_all(Edges_All(i,2),:);
    Center=(Node1+Node2)/2
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
Support=[1 2 5]; %The Support Polygon Base
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
m_node=ones(1,24)*m_meausured/2; %Assume all other nodes have arbitrary mass
m_kin=zeros(1,6); %Assume the joints have no mass

m_tot=[m_kin,m_node];

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
%% Main Loop
count=1; %A number to count the number of iterations
tic


figure
while (1)
    %Get the GUI Status to determine which nodes to control
      
    %Determine which nodes to control, 
    
    Node_Control=0;  %For now Hardcoded (picks the node, or the center of mass)
    Cost_Function=1; %An idex for the Cost function
    Distributed_Control=0; %Distributed or centralized computation
    
    %Possibly also need to set the support index in the GUI
    
    %Get Joystick Commands (or Keyboard commands?)
    Command=[1; 0]; %Get the current state of the joystick (Two Axis)
    %% Specify the Desired Motions of the Nodes and Work Backwards
    A_com=M_Mat(1:2,:);
    % D=.15;  %A Value of .2 seems to approach instability .15 works well with
    % only the com constrained
    Max_Speed_Node=.15;
    Command_Normalized=Command*Max_Speed_Node; %Normalize for the maximum speed D
    
    %Also use the ground constraint matrix
    C_Lock=zeros(9,3*n_all);
    for i=1:length(Support)
            C_Lock(3*(i-1)+1,Support(i))=1;
            C_Lock(3*(i-1)+2,Support(i)+n_all)=1;
            C_Lock(3*(i-1)+3,Support(i)+2*n_all)=1;
    end

    if Node_Control==0  %Center of Mass Control
        A_used=A_com;
    else     %Moving a Specific Node
        Lock_Node=zeros(3,3*n_all);
        Lock_Node(1,Node_Control)=1;
        Lock_Node(2,Node_Control+n_all)=1;
        Lock_Node(3,Node_Control+2*n_all)=1;
        A_used=Lock_Node(1:2,:);
    end
    
    A=[ A_used; C_Lock];  %Note in this case I always use the same A_com
    b=[ Command_Normalized; zeros(9,1)];

    Loop_Con=[];
    Num_Circuits=4;
    for i=1:Num_Circuits
        Elements=3;
        Loop_Con=blkdiag(Loop_Con, ones(1,Elements));
    end

%% Compute the required Motion (centralized)

%Do the Bookkeeping for the state vectors.

[ x_opt, Ldot ] = Controled_Motion_Nodes( y_tot(end,:)', Edges_All, Edges_All_Tube, Loop_Con, L2th, Order, A, b );

%From the Desired Edge Motions, extract the Roller Changes
Command=L2th*Ldot;  %This can be done with local information, right?
%It looks like rollers 1 and 2 are the ground rollers.

%Can Potentially simulate unrecognized failure of the rollers
% Command(6)=0;
%Could Potentially turn off the Commands

%Play Back the Commands through the Rollers
[xdot] = Dynamics_Rollers_no_t( y_tot(end,:)', Edges_All, C, Order, B_T, Command );

%% Estimate the next position. 

%Perform the Euler Integration Update (Does this need to account for the
%real time gap?  In practice, I could measure the new state, as opposed to
%compute it through forward propogation. 
dt=toc;
time(count)=dt;
tic
y_tot=[y_tot; xdot'*dt+y_tot(end,:)];
%Could extract some closed loop position information here


%
%% Do the plotting if desired, compute the frequency every steps_plot loops
steps_plot=10;
if mod(count,steps_plot)==0
    
    y_current=y_tot(end,:)'; %(count,:);
    y_current_mat=reshape(y_current,n_all,3);
%     Duration=toc;
%     tic
    disp(steps_plot/sum(time(end-steps_plot+1:end)))
    %Do I compute the loads just when doing a display?
    
    %Plot the robot (Shape, Loads, CoM, Edge Labels)\
    %Could get the dynamic Loading Data
%     Indices_Tube=L_Load(:,i); %./Max_Load;
%%
    
%   In practice, make this update the figure axis in the GUI 
        
    Indices_Tube=ones(12,1);
    clf
    Plot_Robot_Edges(Edges_All(1:12,:),reshape(y_current',n_all,3),Indices_Tube, .05);
    hold on
%         Plot_Robot_Edges_Color_Spec( Edges_Test(1:12,:), reshape(y_tot(i,:)',n_all,3), .05, [1 0 0] )
        %Plot the Constraint Connections
    [Edge_Con, x_conPlot]=Plot_Connections( y_current', T2T_Qual );
    Plot_Robot_Edges(Edge_Con,x_conPlot,ones(size(T2T_Qual,1),1),.025);
        %Plot the Node/Node Connection Joints with Spheres? 
    Plot_Node_Boxes(y_current', T2T_Qual, d_up, d_down, d_length, d_width );
    
    %Plot the center of mass
    x_com_current=M_Mat*y_tot';
    plot3(x_com_current(1,:),x_com_current(2,:),x_com_current(3,:),'k');
    axis equal
    drawnow
    %Number the nodes
      
    for i=1:6
        text(y_current_mat(i,1),y_current_mat(i,2),y_current_mat(i,3),num2str(i),'fontsize',20)
    end
    %To control, need a way to orient the robot with respect to the world
    %frame.  Use one IMU to gather that data? Align the Plot?
    
end

count=count+1;

end

toc






















