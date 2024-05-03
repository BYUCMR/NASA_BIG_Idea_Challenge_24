function [ x_all, Edges_Tube, Edges_Con, T2T_Qual, B_T, N_subsections, Inds_All ] = Initialize_Octahedron(d_space, d_offset, d_offset_normal, L_tube  )
%Generate the Complete Octahedron Robot
[x_initial, x_final, Adj]=Get_Oct();
in2m=2.54/100;

th=60*pi/180;
% thx=180*pi/180;  %Flip it upside down the face down configuration.
thx=0
%Now Actuate the top triangle. 

R=[cos(th), -sin(th), 0;
   sin(th), cos(th), 0;
   0        0        1];
Rx=[1    0           0;
   0    cos(thx), sin(thx);
   0    -sin(thx), cos(thx)]
x_initial=(Rx*R*x_initial')';
z_min=-min(x_initial(:,3));
x_initial(:,3)=x_initial(:,3)+z_min;
x_initial(:,2)=x_initial(:,2)-min(x_initial(:,2));

%For now, I am hand-defining these circuits
Edges_Kin{1}=[1 2; 
              2 5;
              5 1];
Edges_Kin{2}=[3 4;
              4 5;
              5 3];
Edges_Kin{3}=[3 2;
              2 6;
              6 3];
Edges_Kin{4}=[1 4;
              4 6;
              6 1];
          
%Define the Incidence Matrix of the Triangle
Inc_Tri=[1 -1 0;
         0  1 -1;
         -1 0 1];     
%Generate the Incidence th2L matrix for these guys
B_T=[];
for i=1:length(Edges_Kin)
    B_T=blkdiag(B_T,Inc_Tri);
end
B_T(:,[1,4,7,10])=[]; 
          
Edges_Kin_All=[];
for i=1:length(Edges_Kin)
    Edges_Kin_All=[Edges_Kin_All; Edges_Kin{i}] ;
end

n=size(x_initial,1);

%Scale the Structure to Match the Proper Size

%How does this change if the Joint is out of plane?  Somehow it must
%project back into plane? 

% L_tube=132*in2m; %The physical length of an entire tube in m

L_Edge_True=(L_tube-3*d_space)/3;
L_dif=2*(d_space/sind(30)+(d_offset-d_space/tand(30))*cosd(30));
L_kin_Edge=L_Edge_True+L_dif;

x0=x_initial*L_kin_Edge; %Scale up the initial to match the inner lengths
%Need to move the mass from the true nodes to the other nodes.

N_subsections=length(Edges_Kin);
% Need to be able to extract the Initial Configuration. Options: Take
%This relies on the initial configuration being uniform

%%
it=1;
Edges_Tube=[];
x_all=x0;
count=n+1; %A counter to store the roller matrix vectors
Node_Used=zeros(n,1);
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
        if Node_Used(Node_Current)==0
            d_temp=d_offset;
             Node_Used(Node_Current)=1;
        else
            d_temp=d_offset;
        end
        xprime(it,:)=x_Current+d_temp*dir/norm(dir);
        
        dir_Normal=cross(Vec_1, Vec_2);
        rotationMatrix = rotationVectorToMatrix(dir_Normal/norm(dir_Normal)*-pi/2);
        x_all(count,:)=  xprime(it,:)'+ rotationMatrix'*d_space*dir'/norm(dir)+d_offset_normal*dir_Normal'/norm(dir_Normal);
        x_all(count+1,:)=xprime(it,:)'+ rotationMatrix*d_space*dir'/norm(dir) +d_offset_normal*dir_Normal'/norm(dir_Normal);
        
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

%
% Edges_Test=[Edges_Tube; Edges_Link];
Edge_All_Tube=Edges_Tube;
% Edges_All=[Edges_Tube; Edges_Between_Rollers; Edges_Con_Double];
Edges_All=[Edges_Tube; Edges_Between_Rollers; Edges_Con_Double];
% Edges_Tube=Edges_Tube;
Edges_Con=[Edges_Between_Rollers; Edges_Con_Double];
% Indices_Test=[ones(size(Edges_Tube,1),1), ones(size(Edges_Tube,1),1)*2];
% x_all=[x0; x_roller];
% Plot_Robot(Edges_All,x_all,Indices_All,.05)
% axis equal

%% Generate the Ordering of the Nodes
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
% Num_Circuits=4;  %Hard Code this in

Edge_Con=Edges_Between_Rollers; Edges_Con_Double;
Inds_All=[1 1 1 2 2 2 3 3 3 4 4 4];
Inds_All=[Inds_All, 5*ones(1,size(Edge_Con,1))]



% n_all=size(x_all,1);
% N_L=size(Edges_Tube,1);


end

