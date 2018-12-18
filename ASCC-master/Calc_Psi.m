function out=Calc_Psi(in1,in2)
global obs po eta beta lambda

%po = potential effect distance 



x=in1(1);
y=in1(2);
z=in1(3);



px=sqrt((x-obs(1))^2+(y-obs(2))^2+(z-obs(3))^2);  %o = obstacle position 

% dynamic field => nargin 
if nargin>1
    vx=in2(1);
    vy=in2(2);
    vz=in2(3);
    normv=sqrt(vx^2+vy^2+vz^2);
    % (vx*(x-o(1))+vy*(y-o(2))+vz*(z-o(3)))/normv/px;
    theta=acos((vx*(x-obs(1))+vy*(y-obs(2))+vz*(z-obs(3)))/normv/px);
    if pi>theta && theta>pi/2
        psi1=lambda*((-cos(theta))^(beta-1))*normv/px*(beta*(vx-(vx*(x-obs(1))+vy*(y-obs(2))+vz*(z-obs(3)))*(x-obs(1))/px/px)/px-cos(theta)/px/px*(x-obs(1)));
        psi2=lambda*((-cos(theta))^(beta-1))*normv/px*(beta*(vy-(vx*(x-obs(1))+vy*(y-obs(2))+vz*(z-obs(3)))*(y-obs(2))/px/px)/px-cos(theta)/px/px*(y-obs(2)));
        psi3=lambda*((-cos(theta))^(beta-1))*normv/px*(beta*(vz-(vx*(x-obs(1))+vy*(y-obs(2))+vz*(z-obs(3)))*(z-obs(3))/px/px)/px-cos(theta)/px/px*(z-obs(3)));
    %     plot3(x,y,z,'r.','MarkerSize',6); hold on;
    else
        psi1=0;
        psi2=0;
        psi3=0;
    end
else
    if (px<po)
        psi1=eta*(1/px-1/po)*(x-obs(1))/(px^3);
        psi2=eta*(1/px-1/po)*(y-obs(2))/(px^3);
        psi3=eta*(1/px-1/po)*(z-obs(3))/(px^3);
        out=[psi1;psi2;psi3];
%         plot3(x,y,z,'r.','MarkerSize',6); hold on;
        
    else
        psi1=0;
        psi2=0;
        psi3=0;
    end
end
out=[psi1;psi2;psi3];
end
