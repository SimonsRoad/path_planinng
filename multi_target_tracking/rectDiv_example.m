%% draw original SDT 
% load('DT_example.mat')
DT(DT<0) =0;
c_set = size(DT,2);
r_set = size(DT,1);
max_DT = max(max(DT));

[ys,xs] = meshgrid(1:c_set,1:r_set);
figure
hold on 
xlabel('row')
ylabel('col')

for r = 1:r_set
    for c = 1: c_set
        
        x1 = xs(r,c)-0.5;
        y1 = ys(r,c)-0.5;
        x2 = xs(r,c) + 0.5;
        y2 = ys(r,c) + 0.5;
        
        intensity = DT(r,c) / max_DT;        
        patch([x1 x1 x2 x2],[y1 y2 y2 y1],intensity*ones(1,3))
    end
    
end


null_matrix = DT <= 1;

% boundary detection
[boundary_idx]=bwboundaries(null_matrix);
boundary_rc = [];    
for idx=1:length(boundary_idx)
    boundary_rc = [boundary_rc; boundary_idx{idx}];
end


for i = 1:length(boundary_rc)
    plot(boundary_rc(i,1),boundary_rc(i,2),'rs','MarkerSize',4);    
end


%% subdivision 

N_rect = 6;
r_max_stride = 5;
c_max_stride = 5;
stride_res = 1;
rects=rectDiv(DT,N_rect,r_max_stride,c_max_stride,stride_res);


r_set =size(DT,1);
c_set= size(DT,2);


%% division box 
hold on 
for i = 1:length(rects)
    
    
        x1 = rects{i}.lower(1);
        y1 = rects{i}.lower(2);
        x2 = rects{i}.upper(1);
        y2 = rects{i}.upper(2);
                
        patch([x1 x1 x2 x2],[y1 y2 y2 y1],ones(1,3),'FaceAlpha',0.1,'EdgeColor','r','LineWidth',3)
    
end