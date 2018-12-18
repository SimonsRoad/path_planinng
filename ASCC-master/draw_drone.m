function draw_drone(T,l,r,w)
% T = SE3 

R=double(T);
xl=[-l 0 0 ; l 0 0];
yl=[0 l 0; 0 -l 0];

xyl=[xl;yl];


angle=0:pi/16:2*pi;

propellers=cell(4);
for j=1:4
    circle=zeros(length(angle),3);

    for i=1:length(angle)
    circle(i,:)=xyl(j,1:3)+r*[cos(angle(i)) sin(angle(i)) 0];
    end

    propellers{j}=[circle ones(length(angle),1)];
end


xyl=(R*[xyl ones(4,1)]')';


for i=1:4
    propellers{i}=(R*propellers{i}')';
end


plot3(xyl(1:2,1),xyl(1:2,2),xyl(1:2,3),'LineWidth',w,'Color','k')
plot3(xyl(3:4,1),xyl(3:4,2),xyl(3:4,3),'LineWidth',w,'Color','k')

for i=1:4
    plot3(propellers{i}(:,1),propellers{i}(:,2),propellers{i}(:,3),'r')
end


