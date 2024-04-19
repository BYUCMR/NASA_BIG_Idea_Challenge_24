function [ x_all, Edges_all, Indices_All ] = Stack_trans( x_init, Edges, Indices_Old, x_new, Edges_New, Indices_New, Nodes)
% Take an intitial structure, edges to match, and find the transforms

%https://math.stackexchange.com/questions/222113/given-3-points-of-a-rigid-body-in-space-how-do-i-find-the-corresponding-orienta
%This should end up being just a big linear system. I went ahead and solved
%it here. 
Points_S=[1, 2, 3];
P=x_new(Points_S,:)';
P(:,4)=P(:,1)+cross(P(:,2)-P(:,1), P(:,3)-P(:,1));
Q=x_init(Nodes,:)'; %Take three of the points from the existing structure
Q(:,4)=Q(:,1)+cross(Q(:,2)-Q(:,1), Q(:,3)-Q(:,1));
P_M=[P(:,2)-P(:,1), P(:,3)-P(:,1), P(:,4)-P(:,1)];
Q_M=[Q(:,2)-Q(:,1), Q(:,3)-Q(:,1), Q(:,4)-Q(:,1)];
Pinv=inv(P_M);
Result=Q_M*Pinv*P(:,1)+(Q(:,1)-Q_M*Pinv*P(:,1));
Rot=Q_M*Pinv;
Trans=(Q(:,1)-Q_M*Pinv*P(:,1));

x_new=(Rot*x_new'+Trans)'; %Now I technically know 

%See which nodes are the same
tol=1e-5;
count=1;
for i=1:size(x_init,1)
    for j=1:size(x_new,1)
       D=norm(x_init(i,:)-x_new(j,:));
       if abs(D)<tol
           Match(count,:)=[i,j];
           count=count+1;
       end
    end
end

  x_new(Match(:,2),:)=[]; %Delete the Rows of repeated vertices
% end

Edges_Temp=Edges_New;
%In the edge list, replace the nodes that are part of something else
for i=1:length(Edges_Temp)
    Edges_Temp{i}=Edges_Temp{i}+(size(x_init,1)-3); %Increase by the number of not deleted edges
    for j=1:size(Match,1) %For each match, switch the numbers
        Edges_Temp{i}(Edges_New{i}==Match(j,2))=Match(j,1);
    end
end

%% Combine all of the Results
x_all=[x_init; x_new];
Edges_all=[Edges, Edges_Temp];
Indices_All=[Indices_Old, Indices_New+max(Indices_Old)];
end

