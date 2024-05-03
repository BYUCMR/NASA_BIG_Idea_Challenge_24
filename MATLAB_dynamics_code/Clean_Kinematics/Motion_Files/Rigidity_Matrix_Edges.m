function [ R ] = Rigidity_Matrix_Edges(Edges,x)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
n=size(x,1);
d=size(x,2);
N_L=size(Edges,1);
R=zeros(N_L,n*d);

for i=1:N_L
    %Extract the Point
    x1=x(Edges(i,1),:);
    x2=x(Edges(i,2),:);
    Norm_x1x2=norm(x1-x2);
    %Loop over the dimensions
    for j=1:d
        R(i,Edges(i,1)+n*(j-1))=(x1(j)-x2(j))/Norm_x1x2;
        R(i,Edges(i,2)+n*(j-1))=-R(i,Edges(i,1)+n*(j-1));
    end  

end



end

