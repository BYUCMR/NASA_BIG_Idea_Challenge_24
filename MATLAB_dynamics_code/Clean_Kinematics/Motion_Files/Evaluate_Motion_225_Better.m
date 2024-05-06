clc
clear
close all

%Integrate the Positions to achieve locomotion of the octahedron
%Determine the Initial Positions
[x_initial, x_final, Adj]=Get_Oct();
in2m=2.54/100;

%For now, I am hand-defining these circuits
Edges_Kin{1}=[6 2; 
              2 1;
              1 6];
Edges_Kin{2}=[5 1;
              1 4;
              4 5];
Edges_Kin{3}=[5 2;
              2 3;
              3 5];
Edges_Kin{4}=[6 3;
              3 4;
              4 6];
          
Edges_Kin_All=[];
for i=1:length(Edges_Kin)
    Edges_Kin_All=[Edges_Kin_All; Edges_Kin{i}] ;
end

n=size(x_initial,1);
Plot_Edges(Edges_Kin_All, x_initial,'o-')

Number_Nodes( x_initial )
axis equal


%Set the Geometry of the Structure
d_space=2.5*in2m;
d_offset=7*in2m;

%Scale the Structure to Match the Proper Size

%How does this change if the Joint is out of plane?  Somehow it must
%project back into plane? 

L_tube=132*in2m; %The physical length of an entire tube in m

L_Edge_True=(L_tube-3*d_space)/3;
L_dif=2*(d_space/sind(30)+(d_offset-d_space/tand(30))*cosd(30));
L_kin_Edge=L_Edge_True+L_dif;

x0=x_initial*L_kin_Edge; %Scale up the initial to match the inner lengths
%Need to move the mass from the true nodes to the other nodes.
m_meausured=1.5;  %870 g

N_subsections=length(Edges_Kin);
%% Need to be able to extract the Initial Configuration. Options: Take
%This relies on the initial configuration being uniform

it=1;
Edges_Tube=[];
x_all=x0;
count=n+1; %A counter to store the roller matrix vectors
for i=1:N_subsections
    Current_Edges=Edges_Kin{i}; %Isolate all of the current edges.
    Nodes= Current_Edges(:,1); %unique(reshape(Edges(Current_Edges,:)',length(Current_Edges)*2,1));%Get the Nodes that are part of the current triangle.
    count_init=count;
    for j=1:length(Nodes)  %Add A new Node
        Node_Current=Nodes(j);
        Ind=find(Node_Current==Current_Edges(:,1));
        Node_Next=Current_Edges(Ind,2);
        Ind=find(Node_Current==Current_Edges(:,2));
        Node_Before=Current_Edges(Ind,1);
        x_Current=x0(Node_Current,:);
        Node_Other=Nodes;
        Ind=find(Node_Other==Node_Current);
        Node_Other(Ind)=[];
        %This only works for a triangle
        p_opposite=(x0(Node_Before,:)+x0(Node_Next,:))/2; %Find the centerpoint of the opposite line
        Vec_1=x0(Node_Before,:)-x_Current; %The vector down one of the lines
        Vec_2=x0(Node_Next,:)-x_Current;
        dir=p_opposite-x_Current;
        xprime(it,:)=x_Current+d_offset*dir/norm(dir);
        
        dir_Normal=cross(Vec_1, Vec_2);
        rotationMatrix = rotationVectorToMatrix(dir_Normal/norm(dir_Normal)*-pi/2);
        x_all(count,:)=  xprime(it,:)'+ rotationMatrix'*d_space*dir'/norm(dir);
        x_all(count+1,:)=xprime(it,:)'+ rotationMatrix*d_space*dir'/norm(dir);
        
        Edges_Between_Rollers(it,:)=[count, count+1];
        Edges_Con_Double(count-n,:)=[Node_Current,count];
        Edges_Con_Double(count+1-n,:)=[Node_Current,count+1];
        %Add the Cooresponding row to the mega Adjacency Matrix
%         Adj_Block(Node_Current,it)=1;  %I could Directly add edges and assign them a category?
        Indices_prime(it)=i;
        
        if j==length(Nodes)
            Edges_Tube(it,:)=[count+1,count_init];
        else
            Edges_Tube(it,:)=[count+1,count+2];
        end
        
        count=count+2;
        it=it+1;        
    end

end

%%
% Edges_Test=[Edges_Tube; Edges_Link];

Edges_Test=[Edges_Tube; Edges_Between_Rollers; Edges_Con_Double];
% Indices_Test=[ones(size(Edges_Tube,1),1), ones(size(Edges_Tube,1),1)*2];
% x_all=[x0; x_roller];
% Plot_Robot(Edges_All,x_all,Indices_All,.05)
% axis equal

% Plot the Edges
figure
Plot_Edges(Edges_Test, x_all,'o-')  %All of the Required Edges
axis equal
hold on
Number_Nodes( x_all )
%%
for i=1:size(Edges_Tube,1)
    Edge_1a=Edges_Tube(i,1);
    Edge_1b=Edges_Tube(i,2);
    [row,col,v] = find(Edges_Between_Rollers==Edge_1a);
    if col==1
       Edge_2a= Edges_Between_Rollers(row,2);
    else
       Edge_2a= Edges_Between_Rollers(row,1);
    end
    
    %By the cycle convention, 
    Ind = find(Edges_Tube(:,2)==Edge_2a);
    Edge_2b=Edges_Tube(Ind,1);
    %Now find the kinematic Edge
    Ind_Kin=find(Edges_Con_Double(:,2)==Edge_1a);
    Node_Kin=Edges_Con_Double(Ind_Kin(1),1);
    T2T_Qual(i,:)=[Node_Kin, Edge_1a, Edge_1b, Edge_2a, Edge_2b];
end

%% Resort the Tube Edges into a cell array
% Edge_Con=fliplr(Edges_Link);
% 
Edge_Tube{1}=Edges_Tube(1:3,:);
Edge_Tube{2}=Edges_Tube(4:6,:);
Edge_Tube{3}=Edges_Tube(7:9,:);
Edge_Tube{4}=Edges_Tube(10:12,:);
Num_Circuits=4;  %Hard Code this in
% %From there, extract the T2TOrder
% it=1;
% for i=1:Num_Circuits
%     Edge_Temp=Edge_Tube{i};
%     for j=1:length(Edge_Temp(:,1))
%         %For each tube point, which is the kinematic connection? There will
%         %be exactly one kinematic connection.
%         Center=Edge_Temp(j,1);
%         Node2=Edge_Temp(j,2); %The next node
%         Ind=find(Edge_Temp(:,2)==Center);
%         Node1=Edge_Temp(Ind,1); %The Node Before
%         Ind=find(Edge_Con(:,1)==Center);
%         Kin=Edge_Con(Ind,2);
%         
%         %The center node is the node I am considering, the root node is the
%         %node before, and Node2 is the node afterwards
%         T2T_Angles(it,:)=[Center,Node1, Node2,Kin];
%         it=it+1;
%     end
% 
% end

% it=1;
% for i=1:n  %loop over all of the kinematic nodes
%     Inds=find(Edge_Con(:,2)==i)
%     
%     if length(Inds)>1
% %         for i=1:length(Inds)
%             Node1=Edge_Con(Inds(1),1);
%             Node2=Edge_Con(Inds(2),1);
%             N2N_Angle(it,:)=[i, Node1, Node2];  %Root First, then the other 2?
%             it=it+1
% %         end
%     end
%     
% end


Edge_Con=Edges_Between_Rollers; Edges_Con_Double;
Inds_All=[1 1 1 2 2 2 3 3 3 4 4 4];
Inds_All=[Inds_All, 5*ones(1,size(Edge_Con,1))]

Edge_All_Tube=Edges_Tube;
Edges_All=[Edges_Tube; Edges_Between_Rollers; Edges_Con_Double];
n_all=size(x_all,1);
N_L=size(Edges_Tube,1);
%%  Define the Contact with the Environment

%Define the Contact Matrix for the 3d structure
%Note that these are defined based on the kinematic nodes
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
% Rank_test=rank(R_tot)+rank(C)-rank([R_tot;C])
% Planar=rank(R_Planar)+rank(C)- rank([R_Planar; C])
% Angle=rank(R_Angle)+rank(C)- rank([R_Angle; C])
% Angle=rank(R)+rank(C)- rank([R; C])

%% Plot all of the Edges
x_tot=x_all*100
figure
r=2.54/2;
d_TopPlate2Roller=2.875;  %Top plate to sphere, and a good radius for sphere
R_Node=d_TopPlate2Roller;  
   for i=1:Num_Circuits
        Plot_Robot_d(Edge_Tube{i},x_tot,i*ones(4,1),r, R_Node)
        hold on
   end
    hold on
    N_Edge_Tube=12
    %Label Edges
for i=1:N_Edge_Tube
    Node1=x_tot(Edge_All_Tube(i,1),:);
    Node2=x_tot(Edge_All_Tube(i,2),:);
    Center=(Node1+Node2)/2
    text(Center(1),Center(2),Center(3),num2str(i))
end


view([0 90])
axis equal

%% Prepare the Mass Vectors

% x_end=reshape(y_tot(end,:)',n_all,3);
% m_meausured=.87;  %870 g
m_meausured=1.5;
%Determine the Center of Mass
m_node=ones(1,24)*m_meausured/2; %Assume all other nodes have arbitrary mass
m_kin=zeros(1,6); %Assume the joints have no mass


m_tot=[m_kin,m_node];
%Now Account for the Passive Nodes
%Assume that the start and end of the tube routings are passive

%This is one method to do this, but it is perhaps harder to integrate
%things together? 

% Passive=[1,3];
% Passive=[5,6];
Passive=[];
% m_passive=.25; %A guess at the mass of a passive node
% Edges_All=[Edge_All_Tube; Edge_Con];
for it=1:length(Passive)
   Neighbors=find(Edge_Con(:,2)==Passive(it));
   for j=1:length(Neighbors)    %Hardcoding the Fact that Each of these nodes has four neighbros
       m_tot(Edge_Con(Neighbors(j),1))=m_passive;       
   end
end

% % If the Edge order starts and ends at a passive node, loop though the edges

% for i=1:length(Edge_Tube)
%    
%     m_tot(Edge_Tube{i}(1,1))=m_passive;
%     
% end
% %Determine which nodes connect to them, and then set their contribution to
% %m_passive

%



%Build an M_Mat that captures the true behavior.

M=(m_tot/sum(m_tot));
CoM=M*x_all;
Z=zeros(1,n_all);

M_Mat=[M, Z, Z; 
       Z, M, Z;
       Z, Z, M];
figure
Plot_Robot_Edges(Edges_Test,x_all,ones(size(Edges_Test,1),1), .05);
hold on
r_s=.1
% Plot_Robot_Nodes( x_all, m_tot, r_s )
% title('Location of Passive Nodes')
axis equal
%% How to roll for the incomplete face down case
% Ldot=zeros(N_L,1);
% %Do the Integration.


Ldot=zeros(N_L,1);
Ldot([5 8])=-1;
Ldot([6 9])=1;
Ldot([10 12])=.9;
Ldot([11])=-1.8;

Return_2=zeros(N_L,1);


Ldot2=zeros(N_L,1);
Ldot2([7 8])=1;
Ldot2([9])=-2;
Ldot2([6 11])=0.95;
Ldot2([4 10])=-0.95;
% Ldot_1(8)=-.2
% Ldot_1(7)=.2
% Ldot_2([4 6])=.1;
% Ldot_2(5)=-.2;


% All_Moves=[Ldot_1, Ldot_4, Ldot_5];
% L_end=zeros(N_L,1)
% L_end([3 ])=.3;
% L_end([1 2])=-.15
Ind_Support=[1 2 5]; %The Support Polygon Base
Ind_Support_hist=[0, Ind_Support];
count_sup=1;
% All_Moves=[L_Under, 1.5*L_2, L_3 , L_aR];
% All_Moves=[L_Under, 1.5*L_2, L_3, L_aR, L_aR2];
% All_Moves=[Ldot_0, Ldot_1, L_pT1, L_pT2, L_pT15, L_p2T1, L_end]; % L_pT15, L_pT15, L_p2T1, L_p2T1];
% All_Moves=[Ldot_2, Return, Ldot_1, Return];
y_tot=reshape(x_all,3*n_all,1)';
t_tot=0;
L_init=Get_Lengths_E(Edges_Test, reshape(y_tot,n_all,3));
All_Moves=[Ldot, Return_2, Ldot2, Return_2];
% All_Moves=[Ldot_2, Return_2, L_dot_FaceUp, Return_2];
% All_Moves=[L_pT15]
N_Moves=size(All_Moves,2);
i_support=1;
for iter=1:N_Moves
%     input_total=[Ldot; zeros(6,1)];
    input_total=[All_Moves(:,iter); zeros(90-12,1)]; %use the Current Command
%     Dynamics=@(t,x) Input_Output_Dynamics( t, x, Edges, C, input_total );
%     Dynamics=@(t,x)Dynamics_True_Kin( t, x, Edges_Test, C, Angles_Con, Order, input_total );
%     Dynamics=@(t,x)Dynamics_Bisect( t, x, Edges_All, C, T2T_Angles, input_total );
    Dynamics=@(t,x)Dynamics_Double_Roller( t, x, Edges_All, C, T2T_Qual, input_total );
    %     x_start=reshape(x_initial,3*n,1);
    x_start=y_tot(end,:)';
%     [t,y]=ode45(Dynamics,[0:.01:1], x_initial);   
    tic
    time=[0:.001:1]';
%     [t,y]=ode45(Dynamics,time, x_staFrt);
%     [t,y]=Euler_Integration_No_Events( Dynamics,time,x_start);
    x_mat=reshape(x_start,n_all,3);
    Roll_Event= @(t,x) ~inpolygon(M_Mat(1,:)*x,M_Mat(2,:)*x,x_mat(Ind_Support,1),x_mat(Ind_Support,2));
    [t,y, Event_Flag]=Euler_Integration(Dynamics, time, x_start, Roll_Event)
%     t=t';
    toc
    t_tot=[t_tot; t+max(t_tot)]; 
    y_tot=[y_tot; y];  %This is double counting starting and ending configurations
    
    %If the center of mass leaves, roll over and define new contact
    %condition
    if  Event_Flag
        [x_next, Ind_Support]=Rotate_Robot(y(end,:)', Ind_Support, M_Mat, Adj);
        count_sup=count_sup+1;
        y_tot=[y_tot; x_next'];
        t_tot=[t_tot; t_tot(end)];
        Ind_Support_hist(count_sup,:)=([t_tot(end), Ind_Support]);
        C=zeros(6,3*n_all);  %6 Constraints needed in this case. 
        C(1,Ind_Support(1))=1; %Fix Everything on the First Point
        C(2,Ind_Support(1)+n_all)=1;
        C(3,Ind_Support(1)+2*n_all)=1;
        C(4,Ind_Support(2)+2*n_all)=1;
        C(5,Ind_Support(2)+n_all)=1;
        C(6,Ind_Support(3)+2*n_all)=1;
        i_support=i_support+1;
        C_hist(:,:,i_support)=C;
        
        %After Rolling, Return to the normal configuration
        Edges_Tip=Get_Lengths_E(Edges_Test,reshape(x_next,n_all,3));
        All_Moves(:,iter+1)=L_init(1:N_L)-Edges_Tip(1:N_L);
    end
    
end

OneifRoller=Event_Flag


%%
figure
r=2.54/2;
x_cur=reshape(y_tot(end,:),n_all,3)*100;
d_TopPlate2Roller=2.875;  %Top plate to sphere, and a good radius for sphere
R_Node=d_TopPlate2Roller;  
   for i=1:Num_Circuits
        Plot_Robot_d(Edge_Tube{i},x_cur,i*ones(4,1),r, R_Node)
        hold on
   end
    hold on
    N_Edge_Tube=12
    %Label Edges
for i=1:N_Edge_Tube
    Node1=x_cur(Edge_All_Tube(i,1),:);
    Node2=x_cur(Edge_All_Tube(i,2),:);
    Center=(Node1+Node2)/2
    text(Center(1),Center(2),Center(3),num2str(i))
end


view([0 90])
axis equal




%     end

%% 
figure
i_support=1; %Reset this thing
for i=1:length(t_tot)

    x_vec_temp=y_tot(i,:)';
    x_temp=reshape(x_vec_temp,n_all,3);
    L_hist(:,i)=Get_Lengths_E(Edges_Test, x_temp);
%     [Angles, Grad]=Check_Angle_Constraints( Angles_Joints, y_tot(i,:)', Num_Steps );
    [ Angle_Diff, Grad, Angle_Vals ] =Equal_Angle_Constraint_Shift(x_vec_temp, T2T_Qual);
    Angle_Vals=Angle_Vals*180/pi;
%     Angle_T2T=360-2*atan(d_space/2/d_offset)-sum(Angle_Vals,2);
    Angle_T2T=360-sum(Angle_Vals,2);
%     [Angles_N2N ]=Evaluate_Angles( N2N_Angle, x_vec_temp )*180/pi;
    T2T_hist(:,i)=Angle_Diff;
    hist_vals_1(:,i)=Angle_Vals(:,1);
    hist_vals_2(:,i)=Angle_Vals(:,2);
    H_T2T_min(i)=min(Angle_T2T);
    H_T2T_max(i)=max(Angle_T2T);
%     H_N2N_min(i)=min(Angles_N2N);
%     H_N2N_max(i)=max(Angles_N2N);
end

% subplot 211
plot(t_tot,H_T2T_min, t_tot, H_T2T_max)
ylabel('Tube to Tube Angle')
% subplot 212
% plot(t_tot,H_N2N_min, t_tot, H_N2N_max)
% ylabel('Node to Node Angle')
%% Extract the changes in the edge length

% for i=1:length(t_tot)
%     L_hist(:,i)=Get_Lengths_E(Edges_Test,reshape(y_tot(i,:)',n_all,3));
%     [Angles, Grad]=Check_Angle_Constraints( Angles_Joints, y_tot(i,:)', Num_Steps );
%     Angle_hist(:,i)=Angles; %Look at the change in the true joint angles.
% end
% % plot(L_hist')
% plot(acos(Angle_hist')*180/pi)
% xlabel('Time')
% ylabel('Angle (Degrees)')
% plot(t_tot,L_hist(4,:),t_tot,L_hist(5,:),t_tot,L_hist(6,:)) %This looks fine....
% sum(L_hist)
% plot(sum(L_hist(1:3,:)))

% Prepare the Force Vector
g=9.81;  %Gravity
Fz=m_tot*g; %Z force
F_tot=zeros(3*n_all,1); 
F_tot(2*n_all+1:end)=Fz; %The total force to apply
% 
% Torque_test=Get_Force_TrueK(y_tot(end,:)', Edges_Test, C, F_tot, Angles_Con, Order); %Build the Jacobian and get the forces for one configuration.
% % Torque_test=Get_Force(x_start, Edges, C, F_tot )
% 
% %Get the Motor Torques and the Torques at each edge.
% 
% %Ground Forces:
% Output_Force=Torque_test(end-5:end)
% Be0=sum(Output_Force)-sum(F_tot)  %The ground forces should prevent this 

%% Graph of the Ratio of the Load of Each Edge to its buckling load

in2m=.0254;
psi2kpa=6.89476;
    
% Beam Geometry Parameters
r=2.54/2*in2m*2;
L=132/3*in2m;

% Material Parameters
E=3*10^9; %The Modulus of the material, Pa
t=.01*in2m; %The thickness of the material
G=  4.1e9/1000;

%Material Parameters for LDPE
E=227e6;
poissons=.51;
G=E/(2*(1+poissons));

P= 7*psi2kpa*1000;
I=pi*r^3*t;
% F_cr_Euler=E*I*pi^2/L^2
% F_cr_Fichter=(E*I*pi^2/L^2*(P+G*pi*r*t))/(E*I*pi^2./L^2+P+G*pi*r*t)
Comp_Euler=@(r,L)  E*pi*r.^3*t*pi^2./L.^2;
Comp_Fichter=@(r,L,P)  (E*pi*r.^3*t*pi^2./L.^2.*(P+G*pi.*r*t))./(E*pi*r.^3*t*pi^2./L.^2+P+G*pi*r*t)

%Note that this is being computed with a different C matrix each time...
i_support=1;

Ind_Support_hist_Aug=[Ind_Support_hist; NaN, NaN, NaN, NaN];

for i=1:length(t_tot)
    if t_tot(i)==Ind_Support_hist_Aug(i_support+1,1) 
        i_support=i_support+1;           
    end
    C=C_hist(:,:,i_support);
    %Compute the Lengths at a Configuration
    L_hist_tube(:,i)=Get_Lengths_E(Edges_Test(1:N_Edge_Tube,:),reshape(y_tot(i,:)',n_all,3));
    %Compute the Loading in the Given Configuration
    Loads_All(:,i)=Get_Force_Double_Roller_K(y_tot(i,:)', Edges_All, C, T2T_Qual, F_tot);  %Compute the Resulting Forces and Moments
    L_Load(:,i)=Loads_All(1:size(Edge_All_Tube,1),i);
    Buckle_Load(:,i)=Comp_Fichter(r, L_hist_tube(:,i), P);
    %Given the Lengths, Compute the Buckling Loads
    Ratio(:,i)=(L_Load(:,i)./Buckle_Load(:,i));
end

N_Edges_True=size(Edges_All,1);
%At each node, Make the Color Coorespond to the 


% Plot the Nodes With Different Colors

N_Angle_Con=size(T2T_Qual,1);
clear Torques_Bisection Torques_OutofPlane


r_s=.1
Indices=ones(N_Edges_True,1);
for i=1:length(t_tot)
   Torques_Bisection(:,i)=Loads_All(N_Edges_True+1:N_Edges_True+N_Angle_Con,i);
   Torques_OutofPlane(:,i)=Loads_All(N_Edges_True+N_Angle_Con+1:N_Edges_True+2*N_Angle_Con,i);
%    Torques_All=();

end



figure
subplot 211
plot(t_tot, Torques_Bisection)
xlabel('Time (s)')
ylabel('Torque Bisection (Nm)')
subplot 212
plot(t_tot, Torques_OutofPlane)
xlabel('Time (s)')
ylabel('Torque Out of Plane (Nm)')

%What is the expected torque on this thing? 
Check=.1778*F_tot(end)    %This ground Torque seems to Check out

%% Evaulate an individual case. 


% Get_Force_Double_Roller_K(y_tot(675,:)', Edges_All, C, T2T_Qual, F_tot);  %Compute the Resulting Forces and Moments


%% Do the Combined Loading Analysis ite

d_cuff=5*in2m; %The distance from the joint to the cuff
figure
Config=1;  %Choose a Particular Configuration
%For now just pick 2D
for it=1:length(t_tot)
    for i=1:N_L
        %Extract which nodes are the nodes of interest
        Node_1=Edges_Tube(i,1);
        Node_2=Edges_Tube(i,2);
        %     T2T_Angles
        [row_1, col]=find(T2T_Qual(:,[2,4])==Node_1);
        [row_2, col]=find(T2T_Qual(:,[2,4])==Node_2);
        %How to know which bisecting torque these coorespond to?
        Torque_Node1=Torques_Bisection(row_1,it);
        Torque_Node2=Torques_Bisection(row_2,it);
        L=L_hist_tube(i,it);
        %Need to assume that half of the torque is shared? 
        
        %What do I need to do about the signs here?
        F_Lat_1=Torque_Node1/2*d_cuff;
        F_Lat_2=Torque_Node2/2*d_cuff;

        M1=0; %The moments at the different connections, due to torque/angle
        M2=0;
        V2=(F_Lat_1*d_cuff+F_Lat_2*(L-d_cuff))/L; %The Vertical Force at Support 2
        V1=F_Lat_1+F_Lat_2+V2;

        %Compute the Moments at the points of interest
        M_Cuff1=V1*(-d_cuff)+(L-2*d_cuff)*F_Lat_2+(L-d_cuff)*V2+M1+M2;
        M_Cuff2=-(L-d_cuff)*V1-F_Lat_1*(L-2*d_cuff)+d_cuff*V2;

        Moment_Max=max(abs([M_Cuff1, M_Cuff2])); %Determine the largest moment (sign doesn't matter)
        %Maximum Stress Due to Just the bending

        Stress_Bend(i,it)=Moment_Max*r/I;

        %Subtract from the applied Axial Loading
        %I think right now compressive forces are positive
        F_Axial=L_Load(i,it);
        Stress_Comp(i,it)=(P*pi*r^2-F_Axial)/(2*pi*r*t);
        Min_Stress(i,it)=Stress_Comp(i,it)-Stress_Bend(i,it);
    end

end

plot(Min_Stress')


% plot(t_tot, max(Stress_Comp), t_tot, max(Stress_Bend))
% legend('Max Computed','Required to Bend')
%I'm not sure how correct the units are in this case. 

%%
TORQUE_MOVIE=0;
if TORQUE_MOVIE==1
figure
Torques_Bisection
Max_Torque=max(max(abs(Torques_Bisection)));
for i=1:25:length(t_tot)
%     caxis([-Max_Torque Max_Torque]);
    caxis([0 Max_Torque]);
    x_temp=reshape(y_tot(i,:)',n_all,3);
    Loads=[ones(n,1); abs(Torques_Bisection(:,i))];
    Plot_Robot_Edges(Edges_Test,x_temp,ones(2*N_L,1), .05);
    Plot_Robot_Nodes( x_temp, Loads, r_s );
    axis equal
    hold off
    h=colorbar;
%     caxis([-Max_Torque Max_Torque]);
    caxis([0 Max_Torque]);
    pause(.1)
end

end

%% Plot on the robot the Different Loads?
MOVIE_LOADS=1
if MOVIE_LOADS
    Max_Load=max(max(abs(Min_Stress)));
    Min_Load=min(min(Min_Stress))
    figure
    caxis([Min_Load Max_Load]);
    count=1;
%     caxis=[0 1];
    for i=1:20:length(t_tot)
        Indices_Tube=Min_Stress(:,i); %./Max_Load;
%         Indices_Tube=Ratio(:,i);
        Indices_Con=ones(size(Edges_Test,1)-12,1);
        Indices=[Indices_Tube; Indices_Con];
%         Plot_Robot(Edges_Test,reshape(y_tot(i,:)',n_all,3),Indices, .05);
        Plot_Robot_Edges(Edges_Test,reshape(y_tot(i,:)',n_all,3),Indices, .05);
        h=colorbar;
        caxis([Min_Load Max_Load]);
        ylabel(h, 'Force (N)')
%          caxis=[0 1]
        axis equal
        drawnow
        F(count)=getframe(gcf);
        count=count+1;
        hold off
%         pause()
    end
end

%% Plot on the robot the Different Loads?
MOVIE_COMBINED_LOAD=0
if MOVIE_COMBINED_LOAD
    Max_Load=max(max(abs(L_Load)));
    figure
    caxis([-Max_Load Max_Load]);
    count=1;
%     caxis=[0 1];
    for i=1:5:length(t_tot)
        Indices_Tube=L_Load(:,i); %./Max_Load;
%         Indices_Tube=Ratio(:,i);
%         Indices_Con=ones(N_Edges,1);
        Indices_Con=ones(size(Edges_Test,1)-12,1);
        Indices=[Indices_Tube; Indices_Con ];
%         Plot_Robot(Edges_Test,reshape(y_tot(i,:)',n_all,3),Indices, .05);
        Plot_Robot_Edges(Edges_Test,reshape(y_tot(i,:)',n_all,3),Indices, .05);
        h=colorbar;
        caxis([-Max_Load Max_Load]);
        ylabel(h, 'Force (N)')
%          caxis=[0 1]
        axis equal
        drawnow
        F(count)=getframe(gcf);
        count=count+1;
        hold off
%         pause()
    end
end

%%  Are these things Correct? 

subplot 311
plot(t_tot,Buckle_Load)
xlabel('Time')
ylabel('Buckling Load')

subplot 312
plot(t_tot,L_Load)
xlabel('Time')
ylabel('Load (N)')

subplot 313
plot(t_tot,Ratio)
xlabel('Time')
ylabel('Ratio Load/Buckling Load')

%% Plot the Full Loading Conditions
figure
subplot 411
plot(t_tot,L_Load)
xlabel('Time')
ylabel('Load (N)')
subplot 412
plot(t_tot, Torques_Bisection)
xlabel('Time (s)')
ylabel('Torque Bisection (Nm)')
subplot 413
plot(t_tot, Torques_OutofPlane)
xlabel('Time (s)')
ylabel('Torque Out of Plane (Nm)')
subplot 414
plot(t_tot, Min_Stress)
xlabel('Time (s)')
ylabel('Min Stress')
%%
SAVEMOVIE=1;
if SAVEMOVIE==1
    v = VideoWriter('Rolling_Loads');
    v.Quality = 100;    % Default 75, but I think this only applies to compressed videos
    v.FrameRate = 30;
    open(v)
    writeVideo(v,F)
    close(v)
end



%% Extract the Particular Configuration
% [val, i_crit]=max(max(Loads));
% t_crit=.726;
% figure
% val=find(t_tot==.726)
% % Plot_Robot();
% x_tip=reshape(y_tot(val(1),:)', n_all, 3);
%  Plot_Robot_Edges(Edges_Test,x_tip,Indices, .05);
%  axis equal
% L_config=L_hist_tube(:,val(1))
% 
% %Rescale to Match
% Rescale_Robot=132/(3*1.1667*100/2.54)
% 
% L_True=L_config*Rescale_Robot;
%  % L_Crit=
% L_Inch=L_True*100/2.54
% %
% % Angles_Config=Angle_T2T(:,val(1))*180/pi
% H_T2T_max(val(1))
%%
% for i=1:length(t_tot)
%    x_com(:,i)=M_Mat*y_tot(i,:)' ;  %Compute the Center of Mass
%    %What should M_Mat indicate? 
%    
%    %Compute the Force on Each Edge in Each of These Configurations
% %    Torque(:,i)=Get_Force( y_tot(i,:)', Edges, C, F_tot );
%    Torque(:,i)=Get_Force_TrueK(y_tot(i,:)', Edges_Test, C, F_tot, Angles_Con, Order); %Build the Jacobian and get the forces for one configuration.
% 
% %    Get_Force
%    %From those Torques, extract the Forces. 
%    Motor_Torque(:,i)=th2L'*Torque(1:N_L,i);
%    %Compute the Torque on Each Roller in Each of these configurations
%    
%    L_hist(:,i)=Get_Lengths_E(Edges_Test,reshape(y_tot(i,:)',n_all,3));
%    [Angles, Grad]=Check_Angle_Constraints( Angles_Joints, y_tot(i,:)', Num_Steps );
%    Angle_hist(:,i)=Angles; %Look at the change in the true joint angles.
%    
%    %Check the Values of the Angles
% %    Num_Steps=1;
% %    [Angles, Grad]=Check_Angle_Constraints( Angle_Vertices, x_vec, Num_Steps );
%    
%    
% end
%%
% figure
% % plot(x_com(1,:),x_com(2,:))
% plot(x_com(1,1:length(time)),x_com(2,1:length(time)),'r')
% hold on
% plot(x_com(1,length(time):end),x_com(2,length(time):end),'b')
% % Hackgplot_Colors(Adj, x_end, Indices )
% Plot_Edges(Edges_All,x_end,Indices_All)
% % Plot_Robot(Edges_Test,reshape(y_tot(i,:)',n_all,3),Indices_Test, .05);
% %Also Plot the Center of mass trajectory over time.
% title('Center of Mass Motion')
% axis equal
%%
% figure
% plot(t_tot,Torque(1:N_L,:))
% hold on
% % plot(t_tot, Torque(1,:),'o')
% ylabel('Forces on the Edges (N)')
% %
% % 300 oz-in
% R_Roller=.5*2.54/100; %The Radius of the Rollers
% GR=23/37;
% Total_Gear=GR/R_Roller;
%%
%Recomended Torques, 15 kg-cm

% % Max_Torque=2.1;  %300 oz-in in N-m
% Max_Torque=1.4;
% %Also need the conversion from linear motion to output toque...
% % GR=; %What is the gear ratio between the motors, and the output?
% figure 
% subplot 211
% plot(t_tot, Motor_Torque./Total_Gear)
% hold on
% plot(t_tot, Max_Torque*ones(length(t_tot),1),'--',t_tot, -Max_Torque*ones(length(t_tot),1),'--')
% ylabel('Torque From the Motors (Nm)')
% 
% % From the Trajectory, Back out the Angles Throughout
% 
% subplot 212
% plot(t_tot,acos(Angle_hist')*180/pi)
% xlabel('Time')
% ylabel('Angle (Degrees)')
% 
% 
% %% Given a Minimum Angle and Extent, what else can be done? 
% th_min=min(min(acos(Angle_hist')))
% d_wide=8.16*2.54/100; %The width of one of the node boxes. 
% d_extend=d_wide/(2*tan(th_min/2)) %COmpute how far to extend in m
% d_extend_inch=d_extend*100/2.54
% %How skinny do I need to make the node? 
% 
% %Explore how sensative to width
% n_width=100;
% 
% d_wide_test=linspace(5,9,n_width); %Try 5 to 9 inches of width
% d_extend_test=5:1:10;
% n_extend=length(d_extend_test);
% for i=1:n_width
%     for j=1:n_extend
%         th_min_test(i,j)=2*atan(d_wide_test(i)/(2*d_extend_test(j)))*180/pi; %The minimum possible angle in degrees
%     end
% end
% figure
% plot(d_wide_test, th_min_test)
% xlabel('Width of Node (in)')
% ylabel('Minimum Angle (degrees)')
%  legendCell = cellstr(num2str(d_extend_test', 'd_{offset}=%.2f'));
% legend(legendCell)
