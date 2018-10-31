function draw_circle_sector(xc,theta_min,theta_max,r,color,alpha)
     % this function draw a circle sector 
     % xc : center of circle 
     % theta_min, theta_max : range of angle 
     % r : radius 
     % color : string 
     N = 10 ;
     theta_d = (theta_max - theta_min)/N;     
     xs = xc(1);
     ys = xc(2);
     
     for theta = theta_min : theta_d : theta_max
         X_cur = xc + r * [cos(theta) sin(theta)];
         xs = [xs X_cur(1)];
         ys = [ys X_cur(2)];         
     end
     
     patch(xs,ys,color,'FaceAlpha',alpha,'EdgeColor','none');

end