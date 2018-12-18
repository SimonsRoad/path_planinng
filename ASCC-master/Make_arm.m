function [arms,arms_flat]=Make_arm(LinkLength,base_pos)
% base_pos = SE3 type 
global dof


for idx=1
    arms{idx}.mid=SerialLink([0 0 LinkLength(idx)/2 pi/2],'base',base_pos);
    arms{idx}.end=arms{idx}.mid*Link([0 0 LinkLength(idx)/2 0]);
    arms_flat{2*idx-1}=arms{idx}.mid;
    arms_flat{2*idx}=arms{idx}.end;
end

for idx=2:3
    arms{idx}.mid=arms{idx-1}.end*Link([0 0 LinkLength(idx)/2 pi/2]);
    arms{idx}.end=arms{idx}.mid*Link([0 0 LinkLength(idx)/2 0]);
    arms_flat{2*idx-1}=arms{idx}.mid;
    arms_flat{2*idx}=arms{idx}.end;
end
       
for idx=4:dof
    arms{idx}.mid=arms{idx-1}.end*Link([0 0 LinkLength(idx)/2 0]);
    arms{idx}.end=arms{idx}.mid*Link([0 0 LinkLength(idx)/2 0]);
    arms_flat{2*idx-1}=arms{idx}.mid;
    arms_flat{2*idx}=arms{idx}.end;    
end


end