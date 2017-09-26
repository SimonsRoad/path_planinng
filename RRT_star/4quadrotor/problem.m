classdef problem
    % In this problem, the state is ..... 
    
    
    properties
    dim  % dimension of problem 
    range % workspace /size: D x 2 
    obslist
    N  % N point of each disc
    rq  % radius of quadrotors
    rl   % radius of load
    l % length of string 
    end
    
    methods
        function obj=problem(dim,range,obslist,N)
        obj.dim=dim;
        obj.range=range;
        obj.obslist=obslist;
        obj.N=N;
        end
        
        function mapplot(problem)
            
            axis(reshape(problem.range(1:3,:).',1,[]))
            hold on
            nobs=length(problem.obslist);
            for i=1:nobs
                cur_obs=problem.obslist{i};
                cur_obs.plot();
            end
            hold off
            
        end
         
        function isobs=isobs1(problem,x)
            % is a state x in obs?
            nobs=length(problem.obslist);            
            isobs=0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This section could be changed depending on problem 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % State x= [ q_load q1 q2 q3 q4]
            Npnts=Npoint_gen(x(1:6),problem.rl,problem.N); 
            
            corners=Npoint_gen(x(1:6),problem.rl,4); 
            
            q1=corners(:,1)+[problem.l*sin(x(7))]
            
            
            
            for i=1:nobs % for all the obstacles 
                cur_obs=problem.obs{i};
                isobs=isobs || cur_obs.isobs(Npnts); % for all Npoints generated 
                if isobs==1
                    return % exit if any 
                end                
            end
        end
        
        function isobs=isobs2(problem,x1,x2)
            % are the states along path x1-x2 in obs?
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