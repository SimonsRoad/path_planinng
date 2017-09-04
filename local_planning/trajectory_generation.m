function path_out=trajectory_generation(keyframe,nb)
% this function interpolate 
% path = ndim x timestep
% nb points insertion btw each keyframe
for i=1:length(keyframe)-1
    middle_pnt=pnt_gen(keyframe(:,i),keyframe(:,i+1),nb);
    
end


end