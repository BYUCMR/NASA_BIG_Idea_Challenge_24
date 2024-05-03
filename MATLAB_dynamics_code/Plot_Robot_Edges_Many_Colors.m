function [  ] = Plot_Robot_Edges_Many_Colors( Edges, x, r, colormap, Loads_map, Loads )
%Plot the Edges in a Given Order
%     Color={'r','g','b','k'};
%     Color=[1 0 0;
%            0 1 0;
%            0 0 1;
%            1 0 1];
%     r=.05;
%     r=.075;
    %How to use the different sizes.
    for i=1:size(Edges,1)
        curve=[x(Edges(i,1),:)', x(Edges(i,2),:)'];
        % colormap(Color(Index(i),:));
        [x_plot,y_plot,z_plot]=tubeplot_C(curve,r);
%         surf(x_plot,y_plot,z_plot,Index(i)*ones(size(x_plot)),'edgecolor','k')
        color= interp1(Loads_map,colormap,Loads(i)); %Find the color for the load
        surf(x_plot,y_plot,z_plot,'edgecolor','none','FaceColor',color);
        %I could generate many lines and use tubeplot? 
%         plot3(x(Edges(i,:),1),x(Edges(i,:),2),x(Edges(i,:),3),Color{Index(i)})  %I could one day replace this with tube plot or something like that
        hold on
    end
    

%     hold off
end

