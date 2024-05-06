function [tout, yout, Event_Flag] = Euler_Integration( ydot,time,x0, event)
%Accept a function, and perform Euler Integration
t_tot=length(time);
dt=time(2)-time(1);
n=length(x0);
Event_Flag=0;

%Preallocate the Time
y=zeros(t_tot,n);  %Each row will be one time instant
y(1,:)=x0;
for i=1:length(time)-1
    y(i+1,:)=y(i,:)'+dt*ydot(time(i),y(i,:)');
    if event(time(i+1), y(i+1,:)')>0
       i_final=i+1;
       Event_Flag=1;
       break 
    end
    time(i);
end

if (i==length(time)-1)
   i_final= length(time)-1;
end


tout=time(1:i_final);
yout=y(1:i_final,:);
% Check the event to see if I should end





end

