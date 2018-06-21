addpath('../')
load('castRayResult.txt')
SEDT=signed_distance_transform(castRayResult);

%% DOG + color histogram 
% DOG generation 
filt1=imgaussfilt(SEDT,1.6);
filt2=imgaussfilt(SEDT,1.5);
DOG=filt2-filt1;

figure()
subplot(2,1,1)
surf(SEDT)
title('SEDT')
colorbar

subplot(2,1,2)
surf(DOG)
title('DOG')
colorbar

% histogram 
[N,edges,bin]=histcounts(DOG(:));
negative_edges=find(edges<0);
negative_bins=max(negative_edges)-1;
bin_selected=ceil(negative_bins*0.5); % we select nodes until this cluster
bin=reshape(bin,size(DOG));
[r,c]=find(bin<=bin_selected);

%% plotting
figure
title('local extrema')
surf(SEDT)
hold on 
scatter3(c,r,10*ones(length(c),1),'r*')



%% sobel filter 
h_sobel=fspecial('sobel');
A_sobel_x=imfilter(SEDT,h_sobel','replicate'); % horizontal
A_sobel_y=imfilter(SEDT,h_sobel,'replicate'); % vertical 
figure 
title('local extrema')
subplot(2,1,1)
title('Gx')
surf(A_sobel_x)
view(0,90)
colorbar
subplot(2,1,2)
title('Gy')
surf(A_sobel_y)
view(0,90)
colorbar
%% finding extremas
eps=3;
[rx,cx]=find(abs(A_sobel_x)<eps);
[ry,cy]=find(abs(A_sobel_y)<eps);

extrema_row=[]; extrema_col=[];

for i=1:length(rx)
    for j = 1:length(ry)
        if rx(i)==ry(j) && cx(i)==cy(j)
            extrema_row=[extrema_row rx(i)]; 
            extrema_col=[extrema_col cx(i)];            
        end
    end
end

%% plotting
figure
title('local extrema')
surf(SEDT)
hold on 
scatter3(extrema_col,extrema_row,10*ones(length(extrema_col),1),'r*')











