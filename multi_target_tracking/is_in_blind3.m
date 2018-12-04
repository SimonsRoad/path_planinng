function [check,shp_obj]=is_in_blind3(target1,target2,FOV,x_pnts,plot_flag)
    % 3D polyhydra version of blind checking 
    % FOV constriant evalutation 
    % x_pnts : set evalutation pnt  (3 x N)      
    % this is not relaxed evaluation 
    
    % output 
    % check : N x 1 boolean 
    % shp_obj : alphaShape object for plotting 
    
    target1 = reshape(target1,[],1);
    target2 = reshape(target2,[],1);
%     x_pnts = reshape(x_pnts,[],3);
    
    l = norm(target1 - target2);
    r = l / sqrt(2*(1-cos(FOV)));
    
    % generate point clouds => will wrap with alphaShape 
    
    % resolution of sampling 
    N = 6;
    M = 16;
        
    % sampling on the plane 
    theta =( pi - FOV )/2;
    
    theta_i = linspace(theta,theta+FOV,N);
    theta_j = linspace(0,2*pi,M);
    
    pcl = [];
    for i = 1:N              
        l_i = r*(sin(theta_i(i)) - sin(theta));
        x = r*cos(theta_i(i));        
        for j = 1:M
            y = l_i*cos(theta_j(j)) ;
            z = l_i*sin(theta_j(j));
            pcl = [pcl  ; [x y z]];
        end
    end
    
    
%     figure(1)
%     subplot(2,1,1)
%     plot(alphaShape(pcl(:,1),pcl(:,2),pcl(:,3),20))
%     hold on 
%     plot3(r*cos(theta),r*sin(theta),0,'ro')
%     plot3(-r*cos(theta),r*sin(theta),0,'ro')
%     xlabel('x'); ylabel('y'); zlabel('z');
%     
    
    
    %% Doing transformation 
    m1 = (target1 + target2)/2;    
    t01 = m1 ;
           
    x1 = (target2 - target1)/norm(target2-target1);  % x axis of 1th frame 
    a = rand(1); b = rand(1);
%     y1 = [a b (-a*x1(1)-b*x1(2))/(x1(3))]'; y1 = y1/norm(y1);
    y1 = [0 0 1]'; 
    z1 = cross(x1,y1);
    
    R01 = [x1 y1 z1]; % rotation matrix from 0 to 1             
    T01 = [R01 t01 ; 0 0 0 1];    
    
    pcl1 = T01 * [pcl' ; ones(1,size(pcl,1))];
    pcl1 = pcl1(1:3,:);
    
    shp_obj = alphaShape(pcl1(1,:)',pcl1(2,:)',pcl1(3,:)',20);

    if ~isempty(x_pnts)
        x_pnts = reshape(x_pnts,3,[]);
        check = shp_obj.inShape(x_pnts(1,:),x_pnts(2,:),x_pnts(3,:));
    end
    
    if plot_flag
        plot(shp_obj)    
    end
    %%  Check the result     
%     figure(1)
%     hold on
%     plot3(target1(1),target1(2),target1(3),'ro');
%     plot3(target2(1),target2(2),target2(3),'ro');
%     scatter3(pcl1(1,:),pcl1(2,:),pcl1(3,:),'ko');
% if plot_flag
%     plot(shp_obj)    
% end
%     xlabel('x'); ylabel('y'); zlabel('z');
%     axis equal

    
    
   
    
    


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

     
end