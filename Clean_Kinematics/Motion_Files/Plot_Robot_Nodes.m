function [ output_args ] = Plot_Robot_Nodes( x, Loads, r_s )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    %Plot Spheres as the nodes
%     r_s=r*1.5;
    [x_p,y_p,z_p]=sphere();
    for i=1:size(x,1)
        surf(r_s*x_p + x(i,1),r_s*y_p + x(i,2),r_s*z_p + x(i,3), ones(size(x_p))*Loads(i),'edgecolor','none')
        hold on
    end


end

