function [  ] = Plot_Node_Boxes( x_vec_temp, T2T_Qual, d_up, d_down, d_length, d_width )
%Plot the Physical Extent of the Node Boxes
n_all=length(x_vec_temp)/3;
 x_temp=reshape(x_vec_temp,n_all,3);
hold on
% Number_Nodes( x_all )
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
%         x_root(j,:)=d_offset_base*Vec_Up/norm(Vec_Up)+x_center(j,:);
%         Tube1_Vec=x_temp(Roll_Other1,:)-x_temp(Roll_1,:);
%         Tube2_Vec=x_temp(Roll_Other2,:)-x_temp(Roll_2,:);
%         T2T_Angle_2(i,j)=acos(Tube1_Vec*Tube2_Vec'/(norm(Tube1_Vec)*norm(Tube2_Vec)));
    
    
    %The vertices aligned with the axis and centered at the origin
    vertices = ([-d_length, -d_width, -d_down;
    d_length, -d_width, -d_down;
    d_length, d_width, -d_down;
    -d_length, d_width, -d_down;
    -d_length, -d_width, d_up;
    d_length, -d_width, d_up;
    d_length, d_width, d_up;
    -d_length, d_width, d_up]);
    %Rotate the Vertices
    Vec_x=cross(Vec_Out,Vec_Up);
    R=[Vec_x/norm(Vec_x); Vec_Out/norm(Vec_Out); Vec_Up/norm(Vec_Up)];
    
    %Shift the Vertices
    vertices=vertices*R+x_center(j,:);
    faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 1 2 3 4; 5 6 7 8];
%     color=[1 0 0]
    color=[.1 .1 .1]
    patch('faces',faces,'vertices',vertices,'facecolor',color,'facealpha',1, 'edgecolor','k','linewidth', 1);
    hold on
    end

end

