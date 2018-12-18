close all

figure(1)

axis([-3 4 -3 4 0 2])
hold on
grid on
    
plot3(0,0,0,'gs','linewidth',3)   % initial point
plot3(output(length(output),1), output(length(output),2), output(length(output),3),'rd','linewidth',3)   % final point

for i = 1 : length(output)    

    plot3(output(i,1), output(i,2), output(i,3),'ko','MarkerSize',3)
    
    line([output(i,1) output(i,1)+0.5*cos(output(i,4))], [output(i,2) output(i,2)+0.5*sin(output(i,4))], [output(i,3) output(i,3)]);

    % pause(0.05)
end

xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
legend('Initial Point', 'Goal Point', 'Quadrotor', 'Heading')