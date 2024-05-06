clc
clear
close all
Add_oct_order{1}=[]

Add_oct_order{2}=[6 5 4]  %For now only doing actahedral shapes

Add_oct_order{3}=[1 4 3;
          3 5 2;
          2 6 1];

Add_oct_order{4}=[1 4 3;
          3 5 2;
          2 6 1;
          7 9 8;
          10 12 11;
          13 15 14]

n_plots= length(Add_oct_order)

%Build a Figure with various shapes of octahedron we can make
%%

for plot_iter=1:n_plots
Nodes=Add_oct_order{plot_iter};
    
% ]; %3 5 2;
%     3 4 5;
%     1 4 3;
%     1 3 2;
%     16 18 17;
%     13 15 14]
        % 1 4 3;
        % 2 6 1;
        % 10 12 11;
        % 7 9 8;
        % 13 15 14];
in2m=2.54/100
d_space=2.5*in2m;  %The spacing between the two rollers
d_offset=10*in2m;   %The distance between the kinematic joint and tube
d_offset_normal=0*in2m; %The Offset to Move Normal to the plane of the tube
d_offset_base=2*in2m;   %Offset of how big the box is. 
L_tube=134*in2m;

% [ x_temp, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All ] = Generate_Shape_Order( Nodes, d_space, d_offset, d_offset_normal, d_offset_base, L_tube );
[ x_temp, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All, n_true, Edges_True ] = Generate_Shape_Order( Nodes, d_space, d_offset, d_offset_normal, d_offset_base, L_tube );

x_all=x_temp;
        % x_all(:,2)=x_temp(:,3);
        % x_all(:,3)=-x_temp(:,2);
        % x_all(:,3)=x_all(:,3)-min(x_all(:,3));
x_all(:,3)=x_temp(:,3)-min(x_all(:,3))
    n_all=size(x_all,1);
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


% Plot_Edges(Edges_All, x_all,'o-')  %All of the Required Edges
% hold on
% Number_Nodes( x_all(1:n_true,:))
% axis equal
% title('Nodes')

%% Generate a nice figure of this

    y_tot=reshape(x_all,3*n_all,1)';
    Margin=.5
    n=size(x_all,1)
            xmin=min(min(y_tot(:,1:n)))-Margin ;
            xmax=max(max(y_tot(:,1:n)))+Margin  ;
            ymin=min(min(y_tot(:,n+1:2*n)))-Margin;
            ymax=max(max(y_tot(:,n+1:2*n)))+Margin;
            zmin=min(min(y_tot(:,2*n+1:end)));
            zmax=max(max(y_tot(:,2*n+1:end)))+Margin;

    subplot(1,n_plots,plot_iter)

  
        y_vec_R=y_tot;
        % y_vec_R=y_tot(Frames(i_mov),:)';
        Plot_Robot_Edges_Color_Spec( Edges_Tube, reshape(y_vec_R,n_all,3), .07, [65/255  105/255 225/255] )
        [Edge_Con, x_conPlot]=Plot_Connections( y_vec_R, T2T_Qual );
        Plot_Robot_Joint_Edges(Edge_Con,x_conPlot,ones(size(T2T_Qual,1),1),.025);
        %Plot the Node/Node Connection Joints with Spheres? 
        Plot_Node_Boxes(y_vec_R, T2T_Qual, d_up, d_down, d_length, d_width );
        box off
        grid off
        axis off
        axis equal
        axis([xmin,xmax,ymin,ymax,zmin,zmax])
        Ground=[xmin ymin; xmax ymin; xmax ymax; xmin ymax];
        patch(Ground(:,1), Ground(:,2),zmin*ones(length( Ground(:,2)),1),[.9 .9 .9]);
        view([-34 28])
        lightangle(18,41)
        h.FaceLighting = 'gouraud';
        h.AmbientStrength = 0.3;
        h.DiffuseStrength = 0.8;
        h.SpecularStrength = 0.9;
        h.SpecularExponent = 25;
        h.BackFaceLighting = 'unlit';
end
%%
set(gcf,'color','w');

