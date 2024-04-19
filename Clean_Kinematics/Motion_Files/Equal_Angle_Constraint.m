function [ Angle_Diff, Grad, Angle_Vals ] = Equal_Angle_Constraint(x, Order)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

N_Con=size(Order,1); %The number of constraints
Grad=zeros(length(x),N_Con);

Angle_Diff=zeros(N_Con,1);
Angle_Vals=zeros(N_Con,2); %This returns the values of the two different angles.
n=length(x)/3;
x_mat=reshape(x,n,3);

for i=1:N_Con
    x1=x_mat(Order(i,1),:)';
    x2=x_mat(Order(i,2),:)';
    x3=x_mat(Order(i,3),:)';
    x4=x_mat(Order(i,4),:)';
    x5=x_mat(Order(i,5),:)';
    Vec1=x1-x2;
    Vec2=x3-x2;  
    Vec3=x1-x4;
    Vec4=x5-x4;
    Angle_Vals(i,1)=acos(Vec1'*Vec2/(norm(Vec1)*norm(Vec2)));
    Angle_Vals(i,2)=acos(Vec3'*Vec4/(norm(Vec3)*norm(Vec4)));
    
    Angle_Diff(i)=Angle_Vals(i,1)-Angle_Vals(i,2);
    %Reinitialize these things at each step
    Grad1=zeros(3*n,1);
    Grad2=zeros(3*n,1);
    Grad1([Order(i,1:3) ,Order(i,1:3)+n, Order(i,1:3)+2*n])=Get_Grad_Angle_Rad( x([Order(i,1:3) ,Order(i,1:3)+n, Order(i,1:3)+2*n]) );
    Grad2([Order(i,[1,4,5]) ,Order(i,[1,4,5])+n, Order(i,[1,4,5])+2*n])=Get_Grad_Angle_Rad( x([Order(i,[1 4 5]) ,Order(i,[1 4 5])+n, Order(i,[1 4 5])+2*n]) );
    Grad(:,i)=Grad1-Grad2; %Combine the previous gradient information
    %     Grad([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n],i)=Get_Grad_Bisect_Rad( x([Order(i,:) ,Order(i,:)+n, Order(i,:)+2*n]));
end

end

