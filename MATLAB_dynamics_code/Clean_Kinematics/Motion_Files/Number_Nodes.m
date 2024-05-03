function [  ] = Number_Nodes( x )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

    hold on
    for i=1:size(x,1)
        text(x(i,1),x(i,2),x(i,3),num2str(i))
    end

end

