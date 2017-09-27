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
        function obj=problem(dim,range,obslist,N,rq,rl,l)
        obj.dim=dim;
        obj.range=range;
        obj.obslist=obslist;
        obj.N=N;   % Npoint generation number 
        obj.rq=rq;
        obj.rl=rl;
        obj.l=l;
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
        
        function PlotState(problem,x)
            % This draws the 4quads and the load slung to it. 
            Npnts=Npoint_gen(x(1:6),problem.rl,20); 
            Npnts=[Npnts(:,2:end) Npnts(:,2)];
            
            plot3(Npnts(1,:),Npnts(2,:),Npnts(3,:),'k-','LineWidth',3);
            hold on
            T=SE3(rotx(x(4))*roty(x(5))*rotz(x(6)),x(1:3)); 
            DrawAxis(T,problem.rl*0.75);
                        
            corners=Npoint_gen(x(1:6),problem.rl,4); 
            corners=corners(:,2:end);
            
            q1=corners(:,1)+[problem.l*sin(x(7))*cos(x(8)) problem.l*sin(x(7))*sin(x(8)) problem.l*cos(x(7))]'; % xyz of quadrotor1 
            q2=corners(:,2)+[problem.l*sin(x(9))*cos(x(10)) problem.l*sin(x(9))*sin(x(10)) problem.l*cos(x(9))]'; % xyz of quadrotor2
            q3=corners(:,3)+[problem.l*sin(x(11))*cos(x(12)) problem.l*sin(x(11))*sin(x(12)) problem.l*cos(x(11))]'; % xyz of quadrotor3
            q4=corners(:,4)+[problem.l*sin(x(13))*cos(x(14)) problem.l*sin(x(13))*sin(x(14)) problem.l*cos(x(13))]'; % xyz of quadrotor4
            
            qs=[q1 q2 q3 q4];
            
            for i=1:4
                plot3([qs(1,i) corners(1,i)],[qs(2,i) corners(2,i)],[qs(3,i) corners(3,i)],'r','LineWidth',1);        
%                 draw_drone2(eye(3),qs(:,i),problem.rq,0.5,2);
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
            
            % State x= [ q_load alpha1 beta1 alpha2 beta2 alpha3 beta3 alpha4 beta4]
            Npnts=Npoint_gen(x(1:6),problem.rl,problem.N); 
            
            corners=Npoint_gen(x(1:6),problem.rl,4); 
            corners=corners(:,2:end);
            q1=corners(:,1)+[problem.l*sin(x(7))*cos(x(8)) problem.l*sin(x(7))*sin(x(8)) problem.l*cos(x(7))]'; % xyz of quadrotor1 
            q1=[q1; zeros(3,1)];
            
            
            Npnts=[Npnts Npoint_gen(q1,problem.rq,problem.N)]; 
            
            q2=corners(:,2)+[problem.l*sin(x(9))*cos(x(10)) problem.l*sin(x(9))*sin(x(10)) problem.l*cos(x(9))]'; % xyz of quadrotor2
            q2=[q2; zeros(3,1)];
            
            Npnts=[Npnts Npoint_gen(q2,problem.rq,problem.N)]; 

            
            q3=corners(:,3)+[problem.l*sin(x(11))*cos(x(12)) problem.l*sin(x(11))*sin(x(12)) problem.l*cos(x(11))]'; % xyz of quadrotor3
            q3=[q3 ; zeros(3,1)];

            Npnts=[Npnts Npoint_gen(q3,problem.rq,problem.N)]; 
            
            q4=corners(:,4)+[problem.l*sin(x(13))*cos(x(14)) problem.l*sin(x(13))*sin(x(14)) problem.l*cos(x(13))]'; % xyz of quadrotor4
            q4=[q4; zeros(3,1)];
                              
            Npnts=[Npnts Npoint_gen(q4,problem.rq,problem.N)]; 
            % we need to consider 
            T=SE3(rotx(x(4))*roty(x(5))*rotz(x(6)),x(1:3));
            load_obs=obstacle(T,[problem.rl,problem.rl,0.5]);
            
             
            
            isobs=load_obs.isobs(Npnts(:,problem.N+2:end));  

            if isobs==1
                return % exit if any 
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