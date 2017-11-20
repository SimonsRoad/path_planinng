function frontier_idx=get_frontier(occu_map,center,rad)
    Nray=500;
    angles=linspace(0,2*pi,Nray);
    p_free=0.2;
    p_unknown_upper=0.8;
    p_unknown_lower=0.4;
    Nx=occu_map.GridSize(1);
    Ny=occu_map.GridSize(2);
    
    for i=1:Nray
        [~,midpoints]=occu_map.raycast([center 0],rad,angles(i));
        for j=1:length(midpoints)
            
            is_frontier=false;
            ix=midpoints(j,1); iy=midpoints(j,2);
            if occu_map.getOccupancy([ix iy],'grid')<p_free % if it is free space , let's check if it is frontier actually 
                
                if ix-1>=0
                    is_frontier=is_frontier || (p_unknown_lower<occu_map.getOccupancy([ix-1 iy],'grid') && p_unknown_upper>occu_map.getOccupancy([ix-1 iy],'grid'));
                end
            
                
                if iy-1>=0
                    is_frontier=is_frontier || (p_unknown_lower<occu_map.getOccupancy([ix iy-1],'grid') && p_unknown_upper>occu_map.getOccupancy([ix-1 iy],'grid'));
                end
                
                
                if ix+1<=Nx
                    is_frontier=is_frontier || (p_unknown_lower<occu_map.getOccupancy([ix-1 iy],'grid') && p_unknown_upper>occu_map.getOccupancy([ix-1 iy],'grid'));
                end
                               
                
                if iy+1<=Ny
                    is_frontier=is_frontier || (p_unknown_lower<occu_map.getOccupancy([ix-1 iy],'grid') && p_unknown_upper>occu_map.getOccupancy([ix-1 iy],'grid'));
                end
                
            
            
            end
        end
        
    end



end