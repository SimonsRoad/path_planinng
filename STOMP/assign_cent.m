function [c1,c2]=assign_cent(x1,x2,centroids,frontiers,IDX,fil_IDX)
    n_cent=size(centroids,1);
    assign_combi=[1 1];
    cost_min=999;
    
    if n_cent>1

        for n1=1:n_cent
            for n2=n1+1:n_cent
                cost=norm(x1-centroids(n1,:))+norm(x2-centroids(n2,:));
                if cost<cost_min
                    assign_combi=[n1 n2];       
                    cost=cost_min
                end
            end
        end
        
        c1=centroids(assign_combi(1),:);
        c2=centroids(assign_combi(2),:);

    else
        
        frontiers=frontiers(find(IDX==fil_IDX),:);
        
        % determine c1
       d1=frontiers-x1;
       dist1=zeros(length(d1),1);
       for i=1:length(d1)
           dist1(i,:)=norm(d1(i,:));           
       end
       
       [sorted_d1,idx1]=sort(dist1);
       
       
       n_frontiers=length(frontiers);
       
       c1=sum(frontiers(idx1(1:round(n_frontiers/2)),:))/round(n_frontiers/2);
       
       
        % determine c2
       d2=frontiers-x2;
       dist2=zeros(length(d2),1);
       for i=1:length(d2)
           dist2(i,:)=norm(d2(i,:));           
       end
       
       [sorted_d2,idx2]=sort(dist2);
       
       c2=sum(frontiers(idx2(1:round(n_frontiers/2)),:))/round(n_frontiers/2);
      
       
    end

end