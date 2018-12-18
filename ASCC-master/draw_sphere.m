function draw_sphere(pos,rad)

[x,y,z]=sphere;
x=rad*x; y=rad*y; z=rad*z; 

hSurface = surf(x+pos(1),y+pos(2),z+pos(3));
set(hSurface,'FaceColor',[1 0 0],'FaceAlpha',0.5,'edgecolor','none');


end