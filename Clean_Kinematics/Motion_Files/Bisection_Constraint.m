function [ Angle_Diff, Grad, Angle_Vals ] = Bisection_Constraint( x, Order )
%For a given configuration and a list of angles, enforce that the vector
%between first and last node bisects the angle between 2,1,3. Need to pass
%in the coordinates of the point in 3D.

N_Con=size(Order,1); %The number of constraints
Grad=zeros(length(x),N_Con);
Angle_Diff=zeros(N_Con,1);
Angle_Vals=zeros(N_Con,2); %This returns the values of the two different angles.
n=length(x)/3;
x_mat=reshape(x,n,3);
%
for i=1:length(Order) 
    x1=x_mat(Order(i,1),:)';
    x2=x_mat(Order(i,2),:)';
    x3=x_mat(Order(i,3),:)';
    xT=x_mat(Order(i,4),:)';
    Vec1=xT-x1;
    Vec2=x2-x1;
    Vec3=x3-x1;
    Angle_Vals(i,1)=acos(Vec1'*Vec2/(norm(Vec1)*norm(Vec2)));
    Angle_Vals(i,2)=acos(Vec1'*Vec3/(norm(Vec1)*norm(Vec3)));
    %These constraints have different units?
%     Angle_Diff(i)=Vec1'*(Vec2/norm(Vec2)-Vec3./norm(Vec3));
%     Grad([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n],i)=Get_Grad_Bisect( x([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n]));
    Vec_Dif=Vec2/norm(Vec2)-Vec3./norm(Vec3);
    Angle_Diff(i)=acos(Vec1'*Vec_Dif/(norm(Vec_Dif)*norm(Vec1)))-pi/2;
    Grad([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n],i)=Get_Grad_Bisect_Rad( x([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n]));
end

% x1 is the roller location, x2 and x3 are the neighboring rollers, and xT is the true connection point

end

