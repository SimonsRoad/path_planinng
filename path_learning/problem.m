classdef problem
    % In this problem, the state is .....     
    properties
    dim  % dimension of problem 
    range % workspace /size: D x 2 
    obslist
    end
    
    methods
        function obj=problem(dim,range,obslist)
        obj.dim=dim;
        obj.range=range;
        obj.obslist=obslist;
        end
        
        function mapplot(problem)
            if problem.dim==3
            axis(reshape(problem.range(1:3,:).',1,[]))
            hold on
            nobs=length(problem.obslist);
            for i=1:nobs
                cur_obs=problem.obslist{i};
                cur_obs.plot();
            end
            hold off
            
            
            else 
            axis(reshape(problem.range(1:2,:).',1,[]))
            hold on
            nobs=length(problem.obslist);
            for i=1:nobs
                cur_obs=problem.obslist{i};
                cur_obs.plot();
            end
            hold off
                
                
            end    
        end

                 
        function isobs=isobs1(problem,x)
            % is a state x in obs?
            nobs=length(problem.obslist);            
            isobs=0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This section could be changed depending on problem 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Npnts=[];
            r=0.05; % collision radius
            N=8; % # of inspect pnts per state
            dtheta=2*pi/N;
            for n=1:N
                Npnts=[Npnts [x(1)+r*cos(dtheta*(n-1)) x(2)+r*sin(dtheta*(n-1))]'];            
            end
            
                   
            for i=1:nobs % for all the obstacles 
                cur_obs=problem.obslist{i};
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
        
        function x=sample(problem)
            % sample within range 
            again=1;
            while again
            rand_mat=rand(1,problem.dim);
            rand_mat=[1-rand_mat;rand_mat];
            sampled=problem.range*rand_mat;
            x=diag(sampled);
                if problem.isobs1(x)
                    again=1;
                else 
                    again=0;
                end   
            
            end
            
            
        end
    end % /method
    
end