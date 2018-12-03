    %% small test for rayInsert3D 
    
    target_pnt = [ target1_xs(1) target1_ys(1) 0];
    ray_length = 4; 
    N_azim = 10; 
    N_elev = 5;
    azim_set = linspace(0,2*pi,N_azim +1);
    azim_set = azim_set(1:end-1);
    elev_set = linspace(0,pi/2,N_elev);
    figure
    show(map3)
    hold on
    for azim = azim_set
        for elev = elev_set
            if ~rayInsert3D(map3,target_pnt,azim,elev,ray_length,1/res)
                pnt1 = target_pnt;
                pnt2 = target_pnt + ray_length*[cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)] ;
                plot3([pnt1(1) pnt2(1)],[pnt1(2) pnt2(2)],[pnt1(3) pnt2(3)],'-','LineWidth',2,'Color',[1 0 0 0.4])                
            end
            
        end
    end
    
    
    
    