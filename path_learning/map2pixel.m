function pixel=map2pixel(ws_range,obstacle_list,Nx,Ny)

dx=(ws_range(1,2)-ws_range(1,1))/Nx;
dy=(ws_range(2,2)-ws_range(2,1))/Ny;

Xpnt=ws_range(1,1)+dx/2:dx:ws_range(1,2)-dx/2;
Ypnt=ws_range(2,1)+dy/2:dy:ws_range(2,2)-dy/2;

[x,y]=meshgrid(Xpnt,Ypnt);


% is the cell in obstacle? 

pixel=zeros(Nx,Ny);

for i=1:Nx
    for j=1:Nx

        isobs=false;

        for k=1:length(obstacle_list)
            cur_obs=obstacle_list{k};
            isobs=isobs || cur_obs.isobs([x(i,j) y(i,j)]');
        end
        
        if isobs
            pixel(i,j)=1;
        end
        
    end
end

    
end
