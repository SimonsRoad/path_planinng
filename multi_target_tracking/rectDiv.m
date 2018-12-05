function rects=rectDiv(DT,N_rect,r_max_stride,c_max_stride,stride_res)
    % This function outputs some preferrable box region which has high
    % visibility score 
    
    % r_stride / c_stride : expansion stride 
    % DT = distance field 
    % N_rect 
    % rect : rectangle info (two corners / mean vis_score)
    % field of rect : lower/upper/score 
    
    n = 0;
    
    % inflate the null matrix
    DT(DT <= 1) =0;
    
    
    null_matrix = DT <1;
        
      % boundary detection
    [boundary_idx]=bwboundaries(null_matrix);
    
    boundary_rc = [];    
    for idx=1:length(boundary_idx)
            boundary_rc = [boundary_rc; boundary_idx{idx}];
    end
       
    while n < N_rect && (max(max(DT)) ~= 0)
        
        % current local max index
        [r_lmx,c_lmx]=find(DT == max(max(DT))); 
        % if there are multiple local maximum, we just sample a random one 
        if length(r_lmx )> 1
            rand_idx =  randi(length(r_lmx ));
            r_lmx = r_lmx(rand_idx);
            c_lmx = c_lmx(rand_idx);    
        end
        
        % default rectangle region 
        r_lower = r_lmx;
        r_upper= r_lmx;
        c_lower = c_lmx;
        c_upper = c_lmx;
                
        % 1. rectangle row_expansion         
        for r_exp = 1:r_max_stride
            % box region 
            
            r_lower_prev = r_lower;
            r_upper_prev = r_upper;
            
            r_lower = max(r_lmx - stride_res *  r_exp,1);
            r_upper = min(r_lmx + stride_res * r_exp,size(DT,1));
                        
            
            
        x1 = r_lower;
        y1 =c_lower;
        x2 = r_upper;
        y2 = c_upper;
                
            
            % plot 
            plot(r_lmx,c_lmx,'g*','MarkerSize',8,'MarkerFaceColor','g')
            
                        
            % does it contain null region ?
            boundary_edge_r = boundary_rc(:,1); boundary_edge_c = boundary_rc(:,2);
            included_null=(r_lower <= boundary_edge_r) ...
                & (boundary_edge_r <= r_upper) ...
                & (boundary_edge_c <= c_upper) ...
                & (boundary_edge_c >= c_lower);
            
            if sum(included_null) 
                r_lower = r_lower_prev;
                r_upper = r_upper_prev;
                break % break the loop 
            end
            
          patch([x1 x1 x2 x2],[y1 y2 y2 y1],ones(1,3),'FaceAlpha',0.1,'EdgeColor','g')

              
        end
        
        
         % 2. rectangle  col_expansion                 
           for c_exp = 1:c_max_stride
            % expansion 
            c_lower_prev = c_lower;
            c_upper_prev  = c_upper;
            
            c_lower = max(c_lmx - stride_res * c_exp,1);
            c_upper = min(c_lmx + stride_res *c_exp,size(DT,2));
            
            
            % does it contain null region ?
            boundary_edge_r = boundary_rc(:,1); boundary_edge_c = boundary_rc(:,2);
            included_null=(r_lower <= boundary_edge_r) ...
                & (boundary_edge_r <= r_upper) ...
                & (boundary_edge_c <= c_upper) ...
                & (boundary_edge_c >= c_lower);
                        
        x1 = r_lower;
        y1 =c_lower;
        x2 = r_upper;
        y2 = c_upper;
                

            
            if sum(included_null) 
                c_lower = c_lower_prev;
                c_upper = c_upper_prev;
                break % break the loop 
            end
            
         patch([x1 x1 x2 x2],[y1 y2 y2 y1],ones(1,3),'FaceAlpha',0.1,'EdgeColor','g')
         
            
           end
                  
           
           DT(r_lower:r_upper,c_lower:c_upper) = 0;      

           if r_lower < r_upper && c_lower<c_upper
            sub_blk = DT(r_lower:r_upper,c_lower:c_upper) ;                   
            rect.lower = [r_lower c_lower];
            rect.upper = [r_upper c_upper];
            rect.score = mean(mean(sub_blk));
            
            

            x1 = r_lower;
            y1 =c_lower;
            x2 = r_upper;
            y2 = c_upper;
                
            
            patch([x1 x1 x2 x2],[y1 y2 y2 y1],ones(1,3),'FaceAlpha',0.1,'EdgeColor','b','LineWidth',3)
         
  
            


            % boundary detection
            % null matrix update  but not update 
            null_matrix = DT < 1;
            [boundary_idx]=bwboundaries(null_matrix);

            boundary_rc = [];    
            for idx=1:length(boundary_idx)
                    boundary_rc = [boundary_rc; boundary_idx{idx}];
            end
            
            
            n = n+1;            
            rects{n} = rect;
           end
            
    end
    
   

end