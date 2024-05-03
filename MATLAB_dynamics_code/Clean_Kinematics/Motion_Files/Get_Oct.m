function [ xI, xF, Adj ] = Get_Oct(  )
%Generate an Octahedron

%Line up an Octahedron Appropriately...

%% Part 1: Initialize an octahedron which stands on it's lowest point

%Start with Face 1 on the Ground...
a=1;
b=sqrt(2)/2;
x=[b -b 0; % This is x matrix containing the ordered pairs for the location of each corner in octahedron
   b b  0; % Rows 1-4 start with the square "center" of the octahedron, and Rows 5-6 are the vertical points
   -b b 0; % The origin of this coordinate system is in the geometric center
   -b -b 0;
   0  0  -a
   0  0  a];

x(:,1)=x(:,1)-b;
x=x./sqrt(2);
Adj=[0 1 0 1 1 1; % Adjacency matrix for each node in the same order as x matrix
     0 0 1 0 1 1;
     0 0 0 1 1 1;
     0 0 0 0 1 1;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
Adj=Adj+Adj'; %Make this Symetric

G=graph(Adj); % G contains two tables of information: .Edges which contains the weight/importance of each edge, and the nodes which define the end of each edge (such as edge1-2)
                % The second table is the table labeling all 6 nodes (node
                % 1-6)
N_Edges=size(G.Edges{:,1},1); % Column vector with as many rows as edges
n=size(x,1); % number of points in octahedron (ie number of rows in x) (just a scalar number, not a vector)
Edges=G.Edges{:,1}; % All of the start and end points for each edge (in G.Edges, the first "column" is really made up of a tuple, so extracting the first column leads to a matrix with two columns)

n=size(x,1);
N_L=size(Edges,1); % Number of "lengths" ie number of edges

% Hackgplot(Adj,x)

%% Part 2: Rotate the octahedron to have face 1 be on the ground

%Rotate About the Initial 
Indices=[4 1 4 1 2 4 2 3 3 2 3 1];
%Plot_Edges(Edges,x,Indices)
L=Get_Lengths_E(Edges,x);
% Get_Lengths(Edges,x) %Make sure everything starts well

th=pi-atan2(x(5,3),x(5,1))

Rot=[cos(th)  0   sin(th);
      0     1       0;
      -sin(th)  0   cos(th)]
  
%Rotate everything about this matrix
x=(Rot'*x')'
th=30*pi/180;
%Hackgplot(Adj,x)
Rotz=[cos(th)  sin(th)      0    ; 
      -sin(th) cos(th)      0;
      0         0           1];
xfinal=(Rotz'*x')';
%Plot_Edges(Edges,xfinal, Indices)
%% Get the Initial Configuration
xfinal(:,1)=xfinal(:,1)-xfinal(5,1)
xfinal(:,2)=xfinal(:,2)-xfinal(5,2)
xfinal(:,3)=xfinal(:,3)-xfinal(5,3)
xI=xfinal;
%Plot_Edges(Edges, xI, Indices)
%%
% Hackgplot(Adj,xfinal)
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')

%% Get the Final Configuration
% xI=x;
% xF=x;

%Center at Vertex 1, Rotate about Z, Rotate about Y, Rotate Back About Z
x_temp=xfinal;
xM=x_temp(1,1);
yM=x_temp(1,2);
zM=x_temp(1,3);
x_temp(:,1)=x_temp(:,1)-xM;
x_temp(:,2)=x_temp(:,2)-yM;
x_temp(:,3)=x_temp(:,3)-zM;
x_temp=(Rotz*x_temp')';
% Hackgplot(Adj,x)

th=atan2(x_temp(6,3),x_temp(6,1))
%Rotate about y
Rot=[cos(th)  0   sin(th);
      0     1       0;
      -sin(th)  0   cos(th)]
%Do the Roll Over...
x_temp=(Rot*x_temp')';
%Flip Back
% Hackgplot(Adj,x_temp)
x_temp=(Rotz'*x_temp')';
x_temp(:,1)=x_temp(:,1)+xM;
x_temp(:,2)=x_temp(:,2)+yM;
x_temp(:,3)=x_temp(:,3)+zM;
xF=x_temp;
%Do Some Basic Centering and Such
% Hackgplot(Adj,x_temp)
% axis equal
% hold on 
% Hackgplot(Adj,xI)
% 
% for i=1:n
%     Vec=[xI(i,:); x_temp(i,:)];
%     plot3(Vec(:,1),Vec(:,2),Vec(:,3))
% end


end

