function g=RRT_self(gridmap,g,N,xf)
% prob = structure that contains graph 
% g=graph
% N=number of needed nodes 


while g.n<N
    
    if rand<0.03
        x_rand=[xf,1];
    else
        [x_rand,p_rand,~]=gridmap.sample;
    end
%     plot3(x_rand(1),x_rand(2),x_rand(3),'b*')

    [v_nearest]=g.closest(x_rand,'distance');
    x_nearest=g.vertexlist(:,v_nearest);
    x_new=steer(x_nearest,x_rand);
    % check the probability 
    [isobs,p]=collision(gridmap,x_new(1:2));
    x_new(3)=p;
    
%     plot3(x_new(1),x_new(2),x_new(3),'bo')
    % obstacle free?
    if ~isobs
        v_new=g.add_node(x_new);  
        g.add_edge(v_nearest,v_new);
    end
    
 
end

end
