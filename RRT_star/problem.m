classdef problem
    properties
    dim  % dimension of problem 
    range % workspace /size: D x 2 
    obs
    N
    end
    
    methods
        function obj=problem(dim,range,obs,N)
        obj.dim=dim;
        obj.range=range;
        obj.obs=obs;
        obj.N=N;
        end
        
        function mapplot(problem)
            figure()
            axis(reshape(problem.range(1:3,:).',1,[]))
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
         
        function isobs=isobs1(problem,x)
            nobs=length(problem.obs);            
            isobs=0;
            Npnts=Npoint_gen(x,1.5,problem.N);
            for i=1:nobs
                cur_obs=problem.obs{i};
                isobs=isobs || iswithin(Npnts,cur_obs);
                if isobs==1
                    break
                end
            end
        end
        
        function isobs=isobs2(problem,x1,x2)
            testnb=5;
            testlist=pnt_gen(x1,x2,testnb);
            isobs=0;

            for j=1:testnb
                isobs=isobs || problem.isobs1(testlist(:,j));
                if isobs==1
                    break
                end
            end
            
        end
        
    end
    
end