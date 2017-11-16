function dist=traj_length(xs,ys)
    % compute total length of trajectory
    dist=0;
    for i=1: length(xs)-1
        dist=dist+norm([xs(i+1) ys(i+1)]-[xs(i) ys(i)]);
    end

end