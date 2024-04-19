function [ x_all, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All, n_true, Edges_True ] = Generate_Shape_Order( Nodes, d_space, d_offset, d_offset_normal, d_offset_base, L_tube )
%Generate Any line of Octahedrons from the order


%% Initialize the First Octahedron
[x_initial, x_final, Adj]=Get_Oct();
test=x_initial(3,:);
x_initial(3,:)=x_initial(5,:);
x_initial(5,:)=test;
x_oct=x_initial-[mean(x_initial(:,1)), mean(x_initial(:,2)), mean(x_initial(:,3))];
h=max(x_oct(:,3))-min(x_oct(:,3));
% Reorder=[3, 5;
%          5, 3]
%Hand Defined Edges for a single Octahedron
Edges_Kin{1}=[1 2; 
              2 3;
              3 1];
Edges_Kin{2}=[5 4;
              4 3;
              3 5];
Edges_Kin{3}=[5 2;
              2 6;
              6 5];
Edges_Kin{4}=[1 4;
              4 6;
              6 1];
Edges_Kin_All=[];
for i=1:length(Edges_Kin)
    Edges_Kin_All=[Edges_Kin_All; Edges_Kin{i}];
end
n=size(x_initial);

Indices=[1 1 1 2 2 2 3 3 3 4 4 4];
Edges_Next{1}=Edges_Kin{2};
Edges_Next{2}=Edges_Kin{3};
Edges_Next{3}=Edges_Kin{4};
Indices_Next=[1 1 1 2 2 2 3 3 3]

Edges_New_All=[];
for i=1:length(Edges_Next)
    Edges_New_All=[Edges_New_All; Edges_Next{i}];
end
n=size(x_initial);

% Plot_Edges_C( Edges_Kin_All, x_oct, Indices )
% axis equal
% %Define the set of three triangles to stack
% hold on
x_all=x_oct;
% for i=1:size(x_all,1)
%     text(x_all(i,1),x_all(i,2),x_all(i,3)+.1,num2str(i),'fontsize',24)
% end

Points_S=[1, 2, 3];
Points_D=[3, 5, 2];
%Note the Transpose
x=x_all;
P=x(Points_S,:)';
%https://math.stackexchange.com/questions/222113/given-3-points-of-a-rigid-body-in-space-how-do-i-find-the-corresponding-orienta
%This should end up being just a big linear system. I went ahead and solved
%it here. 
P(:,4)=P(:,1)+cross(P(:,2)-P(:,1), P(:,3)-P(:,1));
Q=x(Points_D,:)';
Q(:,4)=Q(:,1)+cross(Q(:,2)-Q(:,1), Q(:,3)-Q(:,1));
P_M=[P(:,2)-P(:,1), P(:,3)-P(:,1), P(:,4)-P(:,1)];
Q_M=[Q(:,2)-Q(:,1), Q(:,3)-Q(:,1), Q(:,4)-Q(:,1)];
Pinv=inv(P_M)
Result=Q_M*Pinv*P(:,1)+(Q(:,1)-Q_M*Pinv*P(:,1));
Rot=Q_M*Pinv;
Trans=(Q(:,1)-Q_M*Pinv*P(:,1));

x_new=(Rot*x_all'+Trans)'; %Now I technically know 

% Plot_Edges_C( Edges_New_All, x_new, Indices )
% Plot_Edges( Edges_New_All, x_new,'-')

%%

Stack_Height=3;

thz=60*pi/180;
Rz=[ cos(thz) -sin(thz) 0;
     sin(thz) cos(thz) 0;
     0          0   1];
Edges_All=Edges_Kin;
Indices_All=Indices;
x_all=x_oct;
x_next=x_all;

N_Tri=size(Nodes,1)*3+4
N_Nodes=N_Tri*2
for i=1:size(Nodes,1)
    
%   x_flip=Rz*x_next';        %Rotate
%   x_next=x_flip'+[0 0 h];   %Move Upward
 
    [x_all, Edges_All, Indices_All]=Stack_trans( x_all, Edges_All, Indices_All, x_next, Edges_Next, Indices_Next, Nodes(i,:));

%     [x_all, Edges_All, Indices_All]=Stack_things(x_all, Edges_All, Indices_All, x_next, Edges_Next, Indices_Next);

end

n_true=size(x_all,1);

Edges_True=Edges_All;
% %% Do the Plotting
% Edges_All_Mat_2=[];
% for j=1:length(Edges_All)
%     Edges_All_Mat_2=[Edges_All_Mat_2; Edges_All{j}];
% end
% 
% figure
% Plot_Edges_h( Edges_All_Mat_2, x_all, '-' )
% 
% axis equal
% for i=1:size(x_all,1)
% text(x_all(i,1),x_all(i,2),x_all(i,3)+.1,num2str(i),'fontsize',12)
% end

%%

% figure
% colormap('prism')
% %Mix up the Indices to generate more contrast
% Indices_Contrast=Indices_All;
% Indices_Contrast(mod(Indices_Contrast,2)==0)=max(Indices_Contrast)-Indices_Contrast(mod(Indices_Contrast,2)==0)
% 
% Plot_Robot_Edges(Edges_All_Mat_2,x_all, Indices_All, .1)
% axis equal
%%

% Nodes=[3 5 2];
% [x_all_2, Edges_All, Indices_All]=Stack_trans( x_init, Edges, Indices_Old, x_new, Edges_New, Indices_New, Nodes)
Edges_Kin=Edges_All;

Edges_Kin_All=[];
for i=1:length(Edges_All)
    Edges_Kin_All=[Edges_Kin_All; Edges_All{i}] ;
end

Inc_Tri=[1 -1 0;
         0  1 -1;
         -1 0 1];     
%Generate the Incidence th2L matrix for these guys
B_T=[];
for i=1:length(Edges_Kin)
    B_T=blkdiag(B_T,Inc_Tri);
end

%Fix this to be the new version
B_T(:,1:3:size(Edges_Kin_All,1))=[];

% B_T(:,[1,4,7,10])=[];
          



%% Add the offsets

[x_all, Edges_Tube, Edges_Con, T2T_Qual, N_subsections, Inds_All]=Add_Offsets(Edges_Kin, x_all, d_space, d_offset, d_offset_normal, d_offset_base, L_tube);



end

