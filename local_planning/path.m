function path_t=path(p,t_query)
   % refer p 66 연구노트
   
   global m n ts 
   
   p=reshape(p,4*m,(n+1));
   if ts(1)<=t_query && ts(2)>t_query
        segment_idx =1;
   elseif ts(2)<=t_query && ts(3)>t_query
        segment_idx=2;
      elseif ts(3)<=t_query && ts(4)>t_query
        segment_idx=3;
   else
       segment_idx=4;
   end
   t_vector=[];
   for i=1:n+1
    t_vector=[t_vector; t_query^(i-1)];    
   end
    path_t=p(4*segment_idx-3:4*segment_idx,:)*t_vector;
end