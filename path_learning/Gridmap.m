function [x,y]=Gridmap(ws_range,Nx,Ny)

dx=(ws_range(1,2)-ws_range(1,1))/Nx;
dy=(ws_range(2,2)-ws_range(2,1))/Ny;

key_idx=1:Nx*Ny;

Xpnt=ws_range(1,1)+dx/2:dx:ws_range(1,2)-dx/2;
Ypnt=ws_range(2,1)+dy/2:dy:ws_range(2,2)-dy/2;

[x,y]=meshgrid(Xpnt,Ypnt);

x=reshape(x,1,[]);
y=reshape(y,1,[]);



end


