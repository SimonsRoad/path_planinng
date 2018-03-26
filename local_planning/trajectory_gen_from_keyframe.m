function path_out=trajectory_gen_from_keyframe(keyframe,nb)
% this function interpolate 
% path = ndim x timestep
% nb points insertion btw each keyframe

[ndim,len]=size(keyframe);

path_out=zeros(ndim,nb*(len-1)+len);

for i=1:len-1
    inter_pnts=pnt_gen(keyframe(:,i),keyframe(:,i+1),nb+2);
    path_out(:,(i-1)*(nb+1)+1:i*(nb+1))=inter_pnts(:,1:nb+1);    
end
path_out(:,end)=keyframe(:,end);
end