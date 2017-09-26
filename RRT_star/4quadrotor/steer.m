function x_new=steer(x_near,x_rand)
e=0.1;
x_new=x_near+e*(x_rand-x_near);
end