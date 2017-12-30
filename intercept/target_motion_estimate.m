function [px,py,pz]=target_motion_estimate(t,p)
%% Arguments
    %% input : observation
    % t = [t0 t1 t2 .. tN] ( 1 x Nobs)
    % p = [ xs; ys; zs] (3 x Nobs)
   %% output : polynomial coeffi.

%% time scaling 
    global t_horizon_prediction n
    t_cur=t(end);  % last observation is current time 
    t=(t-t_cur)/t_horizon_prediction; % to make the future time lie in [0,1]
    
%% fitting with order n poly
   
    px=fliplr(polyfit(t,p(1,:),n))';
    py=fliplr(polyfit(t,p(2,:),n))';
    pz=fliplr(polyfit(t,p(3,:),n))';
 
end