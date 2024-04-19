function [ Output, Grad ] = Planaritu_Con_Trans( x, Order)
%A planarity constraint such that the thing is transverse to the tube
N_Simplices=size(Order,1);
Grad=zeros(length(x),N_Simplices);
Volume=zeros(N_Simplices, 1);
n=length(x)/3;
x_mat=reshape(x,n,3);

for i=1:N_Simplices
    Vec1=x_mat(Order(i,4),:)-x_mat(Order(i,1),:);
    Vec2=x_mat(Order(i,3),:)-x_mat(Order(i,1),:);
    Vec3=x_mat(Order(i,2),:)-x_mat(Order(i,1),:);
    %Compute the volume of all of the things

    %Compute the orientation
    % Volume=Vec1*cross(Vec2,Vec3);

    % Func([x1;x2;x3])=Vec1'*Vec2/(norm(Vec1)*norm(Vec2))
%     Func([x1;x2;x3;x4])=Vec1'*cross(Vec2,Vec3);
%     Volume(i)=Vec1*cross(Vec2',Vec3');
%     Grad([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n],i) = Get_Grad_Planar( x([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n]) );

    
    Vec_Norm=cross(Vec2,Vec3);
    Volume(i)=acos(Vec1*Vec_Norm'/(norm(Vec1')*norm(Vec_Norm')))-pi/2;
    %Get the gradient
    Grad([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n],i) = Get_Grad_Planar_Rad( x([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n]) );

end

Output=Volume;
%Switch the signs to be opposite those of the initial
% Output=-Volume.*sign(Initial); 

end

