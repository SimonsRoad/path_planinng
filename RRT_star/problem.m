classdef problem
    properties
    dim  % dimension of problem 
    range % workspace /size: D x 2 
    obs
    end
    
    methods
        function obj=problem(dim,range,obs)
        obj.dim=dim;
        obj.range=range;
        obj.obs=obs;
        end
        
        function mapplot(problem)
            figure()
            axis(reshape(problem.range.',1,[]))
            hold on
            nobs=length(problem.obs);
            for i=1:nobs
                cur_obs=problem.obs{i};
                if problem.dim==2
                    draw_rect([cur_obs(1,1) cur_obs(2,1)],[cur_obs(1,2) cur_obs(2,2)])
                else
                    draw_box([cur_obs(1,1) cur_obs(2,1) cur_obs(3,1)],[cur_obs(1,2) cur_obs(2,2) cur_obs(3,2)])
                end
                
            end
            hold off
            
        end
         
        function isobs=isobs(problem,x1,x2)
            testnb=10;
            nobs=length(problem.obs);            
            testlist=pnt_gen(x1,x2,testnb);
            isobs=0;
            for i=1:nobs
                cur_obs=problem.obs{i};
                for j=1:testnb
                    isobs=isobs || iswithin(testlist(:,j),cur_obs);
                    if isobs==1
                        break
                    end
                end
                if isobs==1
                    break
                end
            end
            
            
            
        end
        
        
    
    end
    
end