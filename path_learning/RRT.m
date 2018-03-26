function g=RRT(prob,g,N,xf)
% prob = structure that contains graph 
% g=graph
% N=number of needed nodes 

while g.n<N
    
    if rand<0.03
        x_rand=xf;
    else
        x_rand=prob.sample;
    end
%     plot3(x_rand(1),x_rand(2),x_rand(3),'b*')
    [v_nearest]=g.closest(x_rand);
    x_nearest=g.vertexlist(:,v_nearest);
    x_new=steer(x_nearest,x_rand);
%     plot3(x_new(1),x_new(2),x_new(3),'bo')
    % obstacle free?
    if ~prob.isobs2(x_new,x_nearest)
        v_new=g.add_node(x_new);  
        g.add_edge(v_nearest,v_new);
                

    end
    
    
end

end
