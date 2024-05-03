function [ Cost, dx ] = Formation_Cost( x, Edges, N_Configs, L_nominal, Inc)
%Compute the Cost based on deviance of each robot from nominal lengths...

L_norm=ones(length(Edges),1)*L_nominal;
n=length(x)/(3*N_Configs);
for i=1:N_Configs
   
    x_mat=reshape(x(1:3*n),n,3); %Get the Matrix of the vector
    L=Get_Lengths_Inc(Inc,x_mat);
    %     L=Get_Lengths_E( Edges, x_mat ); %Another potential method
    
    Cost=(L-L_norm)'*(L-L_norm);
    %Get Rigidity Matrix
    %Get the Rigidity Matrix for Current Configuration
    R=Rigidity_Matrix_Inc( Inc, x_mat );
    R_current=[R(:,1:3:end),R(:,2:3:end), R(:,3:3:end)]; %Compute so it goes x,y,z stuff  
    dx=(2*(L'-L_norm')*R_current)';  %The Gradient of this thing 
    
end

%Interesting that there is no way here to maintain constant cost for this
%particular graph? 
%Many stable local equilibria for this energy function...



end

