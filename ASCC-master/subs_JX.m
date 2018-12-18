function JX = subs_JX(t1,t2,L1,L2)

JX(1,1) = cos(t2)./(L1.*cos(t1).*sin(t2) - L1.*cos(t2).*sin(t1));
JX(1,2) = sin(t2)./(L1.*cos(t1).*sin(t2) - L1.*cos(t2).*sin(t1));

JX(2,1) = -cos(t1)./(L2.*cos(t1).*sin(t2) - L2.*cos(t2).*sin(t1));
JX(2,2) = -sin(t1)./(L2.*cos(t1).*sin(t2) - L2.*cos(t2).*sin(t1));

