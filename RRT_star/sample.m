function x_rand=sample(problem)
    d=problem.dim;
    nobs=length(problem.obs);
    isobs=1;
    N=3;
    isout=1;
%    only xyz planning  
%     while isobs
%         x_rand=zeros(d,1);
%         
%         for i=1:d
%             ith_range=problem.range(i,:);
%             x_rand(i)=(ith_range(2)-ith_range(1))*rand+ith_range(1);
%         end
%         % obstacle check 
%         isobs_sub=0;
%         for i=1:nobs
%         isobs_sub=isobs_sub || iswithin(x_rand,problem.obs{i});
%         end
%         isobs=isobs_sub;
%         
%     end
%     
    
    
     while isobs || isout
        x_rand=zeros(d+3,1);
        
        
        
        % sampling
        for i=4:6
            ith_range=problem.range(i,:);
            x_rand(i)=(ith_range(2)-ith_range(1))*rand+ith_range(1);
        end
        
        % difficulty sampling
        is_ds=rand;
        diff_region=[-4 4; -3 3;-1 1];
        if is_ds<0.4  
            for i=1:3
            x_rand(i)=(diff_region(i,2)-diff_region(i,1))*rand+diff_region(i,1);
            end
        else
            for i=1:3
            ith_range=problem.range(i,:);
            x_rand(i)=(ith_range(2)-ith_range(1))*rand+ith_range(1);
            end
        end
        
        
        
        Npnts=Npoint_gen(x_rand,1.5,N);

        % range check
        isout= ~iswithin(Npnts,problem.range(1:3,:));
        
     
                
        % obstacle check 
        isobs_sub=0;
        for i=1:nobs
        isobs_sub=isobs_sub || iswithin(x_rand(1:3),problem.obs{i});
        for j=1:length(Npnts)
            cur_pnt=Npnts(1:3,j);
            isobs_sub=isobs_sub || iswithin(cur_pnt,problem.obs{i});
        end
        isobs=isobs_sub;
        
        end
     end
    
end
