function D=signed_distance_transform(castRayResultBinary)

edgecastRayResultBinary=zeros(size(castRayResultBinary,1),size(castRayResultBinary,2));
% boundary detection
[boundary_idx]=bwboundaries(castRayResultBinary);

for cluster_idx=1:length(boundary_idx)
    cur_cluster_boundary_idx=boundary_idx{cluster_idx};
    for idx=1:length(cur_cluster_boundary_idx)
        edgecastRayResultBinary(cur_cluster_boundary_idx(idx,1),cur_cluster_boundary_idx(idx,2))=1;
    end
end

% figure(1)
% imshow(castRayResultBinary,'InitialMagnification',2000)
% figure(2)
% imshow(edgecastRayResultBinary,'InitialMagnification',2000)

M = imfill(castRayResultBinary,'holes');
D=bwdist(edgecastRayResultBinary);
D(M)=-D(M);
    
end