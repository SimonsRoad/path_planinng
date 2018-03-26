% refet p71-P73
n=7; % order of polynomial

C=100*rand*ones(3*(n+1));
C=zeros(3*(n+1));
t0=0; tf=10; tm=10;


for i=4:n
    for j=4:n
        if i==4 && j==4
        C(3*i+1:3*(i+1),3*j+1:3*(j+1))=(factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4)*(tf-t0))*eye(3);
        else
        C(3*i+1:3*(i+1),3*j+1:3*(j+1))=(factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4)/(i+j-8)*(tf^(i+j-7)-t0^(i+j-7)))*eye(3);
        end
    end
end

Aeq=[]; beq=[];
A=[]; b=[];

% POSTION CONTRAINT 
% ===============
x0=zeros(3,1); xf=[10 5 -2]';
Aeq=[Aeq ;[Tmat(t0,0,n) ; Tmat(tf,0,n)]];
beq=[beq ;x0;xf];
% VELOCITY CONSTRAINT
% ================
dx0=[1 1 -1]'; 
Aeq=[Aeq ; Tmat(t0,1,n)];
beq=[beq;dx0];
% ACCELERATION CONSTRAINT (direction only)
% ====================

% (1) equality condition

ad=[1 1 1]';
Aeq=[Aeq ; [ad(2) -ad(1) 0;0 ad(3) -ad(2)]*Tmat(tm,2,n)];
beq=[beq; zeros(2,1)];
% (2) inequailty condition = same direction 

A=[A ;[-ad(1) 0 0;0 0 0; 0 0 0]*Tmat(tm,2,n)]; 
b=[b; zeros(3,1)];

% CORRIDER CONSTRAINT
% ================
n_seg=5; tlist=linspace(t0,tf,n_seg); delta=1;
t=xf-x0;
r=x0;
for i=2:n_seg-1
   T=Tmat(tlist(i),0,n);
   for j=1:3
       A=[A;(T(j,:)-t(j)*t'*T)]; b=[b;delta-(t'*r)*t(j)-r(j)];
       A=[A;-(T(j,:)-t(j)*t'*T)]; b=[b;delta+(t'*r)*t(j)-r(j)];
   end    
end
    

% OPIMIZATION
% =========

T=[Tx;a_eq];
bnd_cond=[x0; xf; [0 0]'];

p=quadprog(C,[],A,b,Aeq,beq);

% PLOTTING
x=[]; d2x=[]; t=[];
for time=t0:0.05:tf
    t=[t  time];
    traj_res=traj(p,time,n);
    x=[x traj_res.x];
    d2x=[d2x traj_res.d2x];
end

figure

tmpAspect=daspect();
daspect(tmpAspect([1 2 2]))
hold on 

plot3(x0(1),x0(2),x(3),'bo')
plot3(xf(1),xf(2),xf(3),'ro')
plot3(x(1,:), x(2,:), x(3,:))

for time=t0:0.5:tf

traj_res=traj(p,time,n);
quiver3(traj_res.x(1),traj_res.x(2),traj_res.x(3),traj_res.d2x(1),traj_res.d2x(2),traj_res.d2x(3),'Color','r','LineWidth',2,'MaxHeadSize',2);

end

time=tf;
traj_res=traj(p,time,n);
quiver3(traj_res.x(1),traj_res.x(2),traj_res.x(3),traj_res.d2x(1),traj_res.d2x(2),traj_res.d2x(3),'Color','r','LineWidth',2,'MaxHeadSize',2);




