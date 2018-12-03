function draw_sphere_sector(xc,r,azim_min,azim_max,elev_min,elev_max,color,alpha)
     % this function draw a circle sector 
     % xc : center of sphere
     % azim_max,azim_min,elev_min,elev_max : range of angle 
     % r : radius 
     % color : vector 
     % alpha : double
     
     N = 10 ;
    
     azim_d = (azim_max - azim_min)/N;   
     elev_d = (elev_max - elev_min)/N;
     
     xs = xc(1);
     ys = xc(2);
     zs = xc(3);
     
     edge_pnt = []; % N x 3
     
     for azim = azim_min : azim_d : azim_max
         for elev = elev_min : elev_d : elev_max
             X_cur = xc + r * [cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)];
             xs = [xs; X_cur(1)];
             ys = [ys;X_cur(2)];   
             zs = [zs; X_cur(3)];
             
             if ((elev == elev_min) || (elev == elev_max)) && ((azim == azim_min) || (azim == azim_max))
                edge_pnt = [edge_pnt ; X_cur];
             end
             
         end
     end
     
%      patch(xs,ys,color,'FaceAlpha',alpha,'EdgeColor','k');
    shp = alphaShape(xs,ys,zs,20);
    plot(shp,'FaceAlpha',alpha,'EdgeColor','none','FaceColor',color)
    hold on 
    % plot the edge
    for i = 1:4
        pnt1 = xc;
        pnt2 = edge_pnt(i,:);       
        plot3([pnt1(1) pnt2(1)],[pnt1(2) pnt2(2)],[pnt1(3) pnt2(3)],'k');
    end
    
    hold off 

end