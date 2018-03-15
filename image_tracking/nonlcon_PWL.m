function [c,ceq,cgrad,ceqgrad]=nonlcon_PWL(X)
    % param: [azim_min azim_max N_azim elev_min elev_max N_elev]
    % X = [r azim elev] (3 x 1)
    ceq=[]; ceqgrad=[];
    global param;
    % result matrix where (i,j) element corresponds to (i th elev,j th azim) of each set 
    global castRayResult
    azim_min=param(1);
    azim_max=param(2);
    N_azim=param(3);  D_azim=(azim_max-azim_min)/(N_azim-1);
    azim_set=linspace(azim_min,azim_max,N_azim);
    
    elev_min=param(4); 
    elev_max=param(5); 
    N_elev=param(6); D_elev=(elev_max-elev_min)/(N_elev-1);
    elev_set=linspace(elev_min,elev_max,N_elev);
    
    azim=X(2); elev=X(3);
    
    azim_lower_idx=floor(azim/D_azim)+1;
    azim_upper_idx=azim_lower_idx+1;
    
    elev_lower_idx=floor((elev-elev_min)/D_elev)+1;
    elev_upper_idx=elev_lower_idx+1;
    
    azim_lower=azim_set(azim_lower_idx); azim_upper=azim_set(azim_upper_idx);
    elev_lower=elev_set(elev_lower_idx); elev_upper=elev_set(elev_upper_idx);
    
    if elev <elev_lower+(azim-azim_lower)*D_elev/D_azim % lower triangle 
        
        b=[castRayResult(elev_lower_idx,azim_lower_idx)...
            castRayResult(elev_lower_idx,azim_upper_idx)...
            castRayResult(elev_upper_idx,azim_upper_idx)]';
        A=[azim_lower elev_lower 1 ; azim_upper elev_lower 1 ; azim_upper elev_upper 1];
        % z=ax + by + c
        plane_coeff=A\b;
        c=X(1)-plane_coeff(1)*X(2)-plane_coeff(2)*X(3)-plane_coeff(3);
        cgrad=[1 -plane_coeff']';
    else
        b=[castRayResult(elev_lower_idx,azim_lower_idx)...
            castRayResult(elev_upper_idx,azim_lower_idx)...
            castRayResult(elev_upper_idx,azim_upper_idx)]';
        A=[azim_lower elev_lower 1 ; azim_lower elev_upper 1 ; azim_upper elev_upper 1];
        % z=ax + by + c
        plane_coeff=A\b;
        c=X(1)-plane_coeff(1)*X(2)-plane_coeff(2)*X(3)-plane_coeff(3);
        cgrad=[1 -plane_coeff']';        
    end
        
        
    
    
    
    
    
    
    
    



end