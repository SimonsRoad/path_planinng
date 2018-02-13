function Rplot(p,R,d,linewidth)
% this function draw body axis to defined position p with each length d 

plot3([p(1) p(1)+d*R(1,1)],[p(2) p(2)+d*R(2,1)],[p(3) p(3)+d*R(3,1)],'r','LineWidth',linewidth)
plot3([p(1) p(1)+d*R(1,2)],[p(2) p(2)+d*R(2,2)],[p(3) p(3)+d*R(3,2)],'g','LineWidth',linewidth)
plot3([p(1) p(1)+d*R(1,3)],[p(2) p(2)+d*R(2,3)],[p(3) p(3)+d*R(3,3)],'b','LineWidth',linewidth)

