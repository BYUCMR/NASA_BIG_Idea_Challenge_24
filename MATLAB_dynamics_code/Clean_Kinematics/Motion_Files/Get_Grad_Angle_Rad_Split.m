function [ Out ] = Get_Grad_Angle_Rad_Split( x )
%The gradient of an angle expressed in radians as a function of the 3 input
% vertices, where the vertex of the angle is the second point

%Gradients are computed in Compute_Gradient_Angle_Rad

x1x= x(1);
xT1x= x(2);
xT2x=  x(3);
x3x= x(4);
x1y= x(5);
xT1y= x(6);
xT2y= x(7);
x3y= x(8);
x1z= x(9);
xT1z= x(10);
xT2z= x(11);
x3z= x(12);

x2x=.5*(xT1x+xT2x);
x2y=.5*(xT1y+xT2y);
x2z=.5*(xT1z+xT2z);

dx1x= ((x2x - x3x)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x1x - x2x)*sign(x1x - x2x)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(3/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);
dx2x= ((x1x - 2*x2x + x3x)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) + (abs(x1x - x2x)*sign(x1x - x2x)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(3/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x2x - x3x)*sign(x2x - x3x)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(3/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);
dx3x=-((x1x - x2x)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x2x - x3x)*sign(x2x - x3x)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(3/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);

dx1y= ((x2y - x3y)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x1y - x2y)*sign(x1y - x2y)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(3/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);
dx2y= ((x1y - 2*x2y + x3y)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) + (abs(x1y - x2y)*sign(x1y - x2y)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(3/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x2y - x3y)*sign(x2y - x3y)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(3/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);
dx3y=-((x1y - x2y)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x2y - x3y)*sign(x2y - x3y)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(3/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);

dx1z= ((x2z - x3z)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x1z - x2z)*sign(x1z - x2z)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(3/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);
dx2z= ((x1z - 2*x2z + x3z)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) + (abs(x1z - x2z)*sign(x1z - x2z)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(3/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x2z - x3z)*sign(x2z - x3z)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(3/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);
dx3z=-((x1z - x2z)/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(1/2)) - (abs(x2z - x3z)*sign(x2z - x3z)*((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z)))/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)^(1/2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)^(3/2)))/(1 - ((x1x - x2x)*(x2x - x3x) + (x1y - x2y)*(x2y - x3y) + (x1z - x2z)*(x2z - x3z))^2/((abs(x1x - x2x)^2 + abs(x1y - x2y)^2 + abs(x1z - x2z)^2)*(abs(x2x - x3x)^2 + abs(x2y - x3y)^2 + abs(x2z - x3z)^2)))^(1/2);

% Out=[dx1x; dx2x; dx3x; dx1y; dx2y; dx3y; dx1z; dx2z; dx3z];
Out=[dx1x; .5*dx2x; .5*dx2x; dx3x; dx1y; .5*dx2y; .5*dx2y; dx3y; dx1z; .5*dx2z; .5*dx2z; dx3z];

end