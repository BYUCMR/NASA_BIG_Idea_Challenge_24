function [ x_next, Ind_Support ] = Rotate_Robot( x_next, Ind_Support, M_Mat, Adj )
%Take in a Robot, export the next shape

n=length(x_next)/3;
x_mat=reshape(x_next,n,3);
CoM=M_Mat*x_next; %Compute starting center of mass location
P=x_mat(Ind_Support,:);
Rolls=0;
x_init=x_mat;
while ~inpolygon(CoM(1),CoM(2),P(:,1),P(:,2))

P=x_mat(Ind_Support,:);
%%Determine which edge the robot will roll over
if ispolycw(P(:,1), P(:,2))
    P(:,1)=flipud(P(:,1));
    P(:,2)=flipud(P(:,2));
    Ind_Support=fliplr(Ind_Support); %Also flip the labeling
end

%Make sure that the polygon is counter clockwise...
Angle_Ind=[1 2 3;
           2 1 3;
           3 1 2];

Dist=10;
% plot(P(:,1),P(:,2),'*--')
hold on
for i=1:3  %Loop over each of the vertices and find the bisection point  
    
   %Find a Point in the angle Bisector
   Vec1=P(Angle_Ind(i,2),:)-P(Angle_Ind(i,1),:);
   Vec2=P(Angle_Ind(i,3),:)-P(Angle_Ind(i,1),:);
   Bi=Vec1/norm(Vec1)+Vec2/norm(Vec2);
   Bi=Bi/norm(Bi);
   Point(i,:)=P(Angle_Ind(i,1),:)-Bi*Dist;
%    plot([P(Angle_Ind(i,1),1), Point(i,1)],[P(Angle_Ind(i,1),2), Point(i,2)],'o-')
end

axis equal

%The Polygon that cooresponds to Edge 1
Poly(:,:,1)=[P(1,:); P(2,:); Point(2,:); Point(1,:)];
Poly(:,:,2)=[P(2,:); P(3,:); Point(3,:); Point(2,:)];
Poly(:,:,3)=[P(3,:); P(1,:); Point(1,:); Point(3,:)];

% for i=1:3
%      plot(Poly(:,1,i), Poly(:,2,i),'*-')
%      hold on
% end
% plot(Poly2(:,1), Poly2(:,2),'*-')
% plot(Poly3(:,1), Poly3(:,2),'*-')
% CoM=[15 0]
% plot(CoM(1),CoM(2),'o')
%Determine in Which polygon the CoM Is
for i=1:4  %This is set to 4 so that it will throw an error if it gets stuck
    if inpolygon(CoM(1),CoM(2),Poly(:,1,i),Poly(:,2,i))
        disp('Worked')
        break
    end
end

Roll_Edge=i;
Roll_Next=Roll_Edge+1;
if Roll_Next>3
   Roll_Next=1; 
end
%% Perform the Actual Roll

New_O=Ind_Support(Roll_Edge);
Other=Ind_Support(Roll_Next);

x_new=[x_mat(:,1)-x_mat(New_O,1),x_mat(:,2)-x_mat(New_O,2),x_mat(:,3)-x_mat(New_O,3)];

%Rotate all of those points in the ground plane
theta1=atan2(x_new(Other,2),x_new(Other,1));
Rz=[cos(theta1)    sin(theta1) 0;
    -sin(theta1)    cos(theta1) 0;
    0               0          1];

x_rot2=Rz*x_new';
x_rot=x_rot2';

% Hackgplot(Adj,x_rot)
% xlabel('x')

for j=1:6  %n    % Hardcoding 6 nodes!
   disp('Only checking first 6 nodes')
   if any(j==Ind_Support)
       theta(j)=pi; %Make it be some large value     
   else
       theta(j)=atan2(x_rot(j,3),-(x_rot(j,2))); %I think this abs is wrong, switch to - sign
   end
end
theta(theta<-160*pi/180)=pi; %Reset the angles that are too small
[theta2,New_Support]=min(theta);

if (theta2+pi<1e-4)
    theta(New_Support)=pi; %Fix this error if it shows up
    [theta2,New_Support]=min(theta); %
end


if theta2<0
   if abs(theta2)<1e-4
       theta2=0;
   else
%        error('Negative Angle')
   end
end

    %Assign a New Support index
Ind_Support=[New_Support,New_O,Other]; %The support indices for the next level

Rx=[1           0       0; 
        0    cos(theta2)    sin(theta2); 
        0    -sin(theta2)    cos(theta2)];
Newx=(Rx'*x_rot')'; %This is where it actually tips over

x_F=(Rz'*Newx')';
xFinal=[x_F(:,1)+x_mat(New_O,1),x_F(:,2)+x_mat(New_O,2),x_F(:,3)+x_mat(New_O,3)];

x_mat=xFinal;
x_next=reshape(xFinal,3*n,1);
CoM=M_Mat*x_next; %Compute starting center of mass location
%Also Need to Reassign the Vector P
P=x_mat(Ind_Support,:);
Rolls=Rolls+1;
if Rolls>1.5
   disp(Rolls) 
end

% close all 
% Hackgplot(Adj,x_init)
% hold on
% Hackgplot(Adj, x_mat)
% 
% figure
% Hackgplot(Adj,x_mat)
% hold on
% plot(P(:,1),P(:,2),CoM(1),CoM(2),'o')
end

%%Do The Plotting to make sure things worked
% 
% close all 
% Hackgplot(Adj,x_init)
% hold on
% Hackgplot(Adj, x_mat)



% x_next=x_next;
end

