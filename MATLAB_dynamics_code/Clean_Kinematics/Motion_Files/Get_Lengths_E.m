function [ L ] = Get_Lengths_E( Edges, x )
%This function receives a matrix which defines the start and end points for
%each edge, along with the spatial position of each point in the vector x,
%and returns the length of each edge

N_L=size(Edges,1);
L=zeros(N_L,1);
for i=1:N_L
L(i)=norm(x(Edges(i,1),:)-x(Edges(i,2),:)); %Edges(i,1) is the start point of edge i. Edges(i,2) is the end point. The numbering of the nodes in Edge is consistent with the index of each node in x, therefore lengths can be quickly calculated
end

end

