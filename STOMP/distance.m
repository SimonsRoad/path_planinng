function res=distance(x_ref,obs_list)

% x_ref=[x_ref y_ref]
% obs_list=[obs_x,obs_y]

distance_vec=bsxfun(@minus,x_ref,obs_list);
res=zeros(1,size(distance_vec,2));

for i=1:size(distance_vec,2)
    res(:,i)=norm(distance_vec(:,i));
end


end