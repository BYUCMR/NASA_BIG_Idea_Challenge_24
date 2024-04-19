function [  ] = Hackgplot( A,x )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

   [i, j] = find(triu(A));
    X = permute(cat(3, x(i, :),x(j, :)), [3 1 2]);
    plot3(X(:, :, 1), X(:, :, 2), X(:, :, 3),'-k','Linewidth',1)

   
    %Do some nice coloration of the internal and external links.
   
   
   
    %Should I fill all of the faces in somehow?

end