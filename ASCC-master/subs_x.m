function x = subs_x(t1,t2,t3,L1,L2,L3,posx,posy)

x(1) = posx + L1.*cos(t1) + L2.*cos(t2)+L3.*cos(t3);
x(2) = posy + L1.*sin(t1) + L2.*sin(t2)+L3.*sin(t3);
