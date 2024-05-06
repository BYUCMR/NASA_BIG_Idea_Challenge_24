function [ Roller_Pos ] = Pos_Tube_Oct( Edges, x )
%Determine the Position along the tube for each roller using the full
%dynamics

Lengths_All=Get_Lengths_E(Edges,reshape(x, length(x), 3));
Mat=[1 0 0;
     1 1 0];

Sum_Mat=[]; 
for i=1:4
   Sum_Mat=blkdiag(Sum_Mat,Mat); 
end

Roller_Pos=Sum_Mat*Lengths_All(1:12);

end

