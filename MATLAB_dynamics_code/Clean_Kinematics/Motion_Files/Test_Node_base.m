

d_offset_base=d_offset;
for i=1:length(t_tot)
% i=1;
    %For Each set of Rollers, ComputeThe Center Point, adn the external
    x_vec_temp=y_tot(i,:)';
    x_temp=reshape(x_vec_temp,n_all,3);
    for j=1:size(T2T_Qual,1)
        Roll_1=T2T_Qual(j,2);
        Roll_2=T2T_Qual(j,4);
        Roll_Other1=T2T_Qual(j,3);
        Roll_Other2=T2T_Qual(j,5);
        %Compute the Center of the Two Points
        x_center(j,:)=.5*(x_temp(Roll_1,:)+x_temp(Roll_2,:));
        % Compute an outward Normal
        Vec_Out=cross(x_temp(Roll_2,:)-x_temp(Roll_1,:), x_temp(Roll_Other1,:)-x_temp(Roll_1,:));
        Vec_Up=-cross(Vec_Out,x_temp(Roll_2,:)-x_temp(Roll_1,:));
        x_root(j,:)=d_offset_base*Vec_Up/norm(Vec_Up)+x_center(j,:);
        Tube1_Vec=x_temp(Roll_Other1,:)-x_temp(Roll_1,:);
        Tube2_Vec=x_temp(Roll_Other2,:)-x_temp(Roll_2,:);
        T2T_Angle_2(i,j)=acos(Tube1_Vec*Tube2_Vec'/(norm(Tube1_Vec)*norm(Tube2_Vec)));
    end
    
    for j=1:n
        Inds=find(T2T_Qual(:,1)==j);
        Vec1=x_root(Inds(1),:)-x_temp(j,:);
        Vec2=x_root(Inds(2),:)-x_temp(j,:);
        N2N_Angle_2(i,j)=acos(Vec1*Vec2'/(norm(Vec1)*norm(Vec2)));
    end
    
    %Now, Loop through and compute all of the node to node angles
    
end


figure
subplot 211
plot(t_tot,N2N_Angle_2*180/pi)

subplot 212
plot(t_tot,T2T_Angle_2*180/pi)
% Plot_Edges(Edges_Test, x_all,'o-')  %All of the Required Edges
% hold on
% 
% plot3(x_root(:,1),x_root(:,2),x_root(:,3),'ok')
% % plot3(x_center)
% axis equal
