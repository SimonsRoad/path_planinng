%% data generation 
global castRayResult param

load example_raycast_result.mat mat
castRayResult=mat;
elev_min=pi/8; elev_max=pi/3;
param=[0 2*pi size(castRayResult,2) elev_min elev_max size(castRayResult,1) ];
[x,y]=meshgrid(linspace(0,2*pi,size(castRayResult,2)),linspace(elev_min,elev_max,size(castRayResult,1)));
z=castRayResult;
x=reshape(x,[],1);
y=reshape(y,[],1);
z=reshape(z,[],1);

% x=[x ; x+2*pi];
% y=[y;y];
% z=[z;z];
% 
% x=[x ; x-2*pi];
% y=[y;y];
% z=[z;z];
% 
% x=x/(2*pi);
% y=(y-elev_min)/(elev_max-elev_min);

global d_track nx ny
d_track=4;
nx=5; ny=4;

%% cost4: gaussian based cost function 


% query_point=[0.5,0.8];
x_train=[];
y_train=[];
pert=1e-5;
for i=1:length(x)
   
    if z(i)<4
        for translate_=[-1 0 1]
        for rep=1:(4-z(i))
        x_train=[x_train ; (x(i)/(2*pi) + 2*pert*rand() -pert + translate_)]  ;       
        y_train=[y_train ;((y(i)-elev_min)/(elev_max-elev_min)+2*pert*rand() -pert)];
        end
        end
    end
    
end

options = statset('Display','final');
obj = fitgmdist([x_train y_train],6,'Options',options);
ezcontour(@(x,y) 10*pdf(obj,[x y]),[0 1],[0 1]);
%ezsurfc(@(x,y) pdf(obj,[x y]));
axis equal
plot(x_train,y_train,"ro")


