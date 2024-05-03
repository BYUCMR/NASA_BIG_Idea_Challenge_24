function [ Edge_Con, x_conPlot ] = Plot_Connections( x_vec_temp, T2T_Qual )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    n_all=length(x_vec_temp)/3;
    x_temp=reshape(x_vec_temp,n_all,3);
    n_kin=max(T2T_Qual(:,1));
    n_Roller=size(T2T_Qual,1);
    for j=1:size(T2T_Qual,1)
        Roll_1=T2T_Qual(j,2);
        Roll_2=T2T_Qual(j,4);
        x_center(j,:)=.5*(x_temp(Roll_1,:)+x_temp(Roll_2,:));
        Edge_Con(j,:)=[j,T2T_Qual(j,1)+n_Roller];
    end
    
    x_conPlot=[x_center; x_temp(1:n_kin,:)]; %Need to take all of the nodes, not just the six
        
end

