function draw_rect(p1,p2)

x=[0 1 1 0]*(p2(1)-p1(1))+p1(1);
y=[0 0 1 1]*(p2(2)-p1(2))+p1(2);

 h=patch(x,y,'k','FaceAlpha',0.2);
 set(h,'edgecolor','w')
end