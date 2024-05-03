function [  ] = Plot_Edges(Edges, x, Style)
%Plots a graph displaying the edges and the nodes of the octahedron

d=size(x,2);
    if d==2
       x=[x, zeros(size(x,1),1)]; 
    end
   i=Edges(:,1);
   j=Edges(:,2);
    X = permute(cat(3, x(i, :),x(j, :)), [3 1 2]);
    plot3(X(:, :, 1), X(:, :, 2), X(:, :, 3), Style)
   

end

