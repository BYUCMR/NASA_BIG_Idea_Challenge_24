function [ Angle_Diff, Grad, Angle_Vals ] = Planar_Bisection_Constraint( x, Order )
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
%     x1=x_mat(Order(i,1),:)';
    x2=x_mat(Order(i,2),:)';
    x3=x_mat(Order(i,3),:)';
    x4=x_mat(Order(i,4),:)';
    x5=x_mat(Order(i,5),:)';
    Vec1=x4-x2;
    Vec2=x3-x2;
    Vec3=x2-x4;
    Vec4=x5-x4;
    Angle_Vals(i,1)=acos(Vec1'*Vec2/(norm(Vec1)*norm(Vec2)));
    Angle_Vals(i,2)=acos(Vec3'*Vec4/(norm(Vec3)*norm(Vec4)));
    %These constraints have different units?
%     Angle_Diff(i)=Vec1'*(Vec2/norm(Vec2)-Vec3./norm(Vec3));
%     Grad([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n],i)=Get_Grad_Bisect( x([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n]));
%     Vec_Dif=Vec2/norm(Vec2)-Vec3./norm(Vec3);
%     Angle_Diff(i)=acos(Vec1'*Vec_Dif/(norm(Vec_Dif)*norm(Vec1)))-pi/2;
    Angle_Diff(i)=Angle_Vals(i,1)-Angle_Vals(i,2);
    Grad1=zeros(3*n,1);
    Grad2=zeros(3*n,1);
    Ind1=[Order(i,[4 2 3]),Order(i,[4 2 3])+n, Order(i,[4 2 3])+2*n];
    Ind2=[Order(i,[2 4 5]),Order(i,[2 4 5])+n, Order(i,[2 4 5])+2*n];
    Grad1(Ind1)=Get_Grad_Angle_Rad( x(Ind1));
    Grad2(Ind2)=Get_Grad_Angle_Rad( x(Ind2));
    Grad(:,i)=Grad1-Grad2;  %This should work out?
end

% x1 is the roller location, x2 and x3 are the neighboring rollers, and xT is the true connection point

end

