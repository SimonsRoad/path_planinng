%% new trial % refer p71-P73
g=9.81;
n=8; % order of polynomial
% for better condition number of C matrix 
C=10*rand(3*(n+1),1);

% IN REAL TIME DOMAIN 
%%%%%%%%%%%%%%

t0_real=0; tf_real=30; tm_real=[30]; t_cond_real=[t0_real tf_real tm_real];

% BOUNDARY CONDITION
%%%%%%%%%%%%%%%
x0=zeros(3,1); xf=10*ones(3,1); 
dx0_real=zeros(3,1);



% in non-dimensionalized domain:
% mapping btw two domain
t_cond=(t_cond_real-t0_real)/(tf_real-t0_real);
t0=t_cond(1); tf=t_cond(2); tm=t_cond(3:end);

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
Aeq=[Aeq ;[Tmat(t0,0,n) ; Tmat(tf,0,n)]];
beq=[beq ;x0;xf];
% VELOCITY CONSTRAINT
% ================
dx0=dx0_real*(tf-t0);
Aeq=[Aeq ; Tmat(t0,1,n)];
beq=[beq;dx0];

%ACCELERATION CONSTRAINT (direction only)
% ====================
ad=[0.1 0.5 0.5]';

for i=1:length(tm)
% (1) equality condition

Aeq=[Aeq ; [ad(2) -ad(1) 0;0 ad(3) -ad(2)]*Tmat(tm(i),2,n)];
beq=[beq; [0 ad(2)*g]'];
% beq=[beq; [0 0]'];


% (2) inequailty condition = same direction (very improtant)
A=[A ;[-ad(1) 0 0;0 0 0; 0 0 0]*Tmat(tm(i),2,n)]; 
b=[b; zeros(3,1)];

end

%ACCELERATION CONSTRAINT (initial condition)
% ====================
% d2x0=[0 0 0]';
% Aeq=[Aeq ; Tmat(t0,2,n)];
% beq=[beq; d2x0];


% CORRIDER CONSTRAINT
% % ================
n_seg=5; tlist=linspace(t0,tf,n_seg); delta=1.5;
t=(xf-x0)/norm(xf-x0);
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
% options = optimoptions('quadprog','Display','off',);

p=quadprog(C,[],A,b,Aeq,beq);
%p=fmincon(@(x) x'*C*x,rand(3*(n+1),1),A,b,Aeq,beq);


%% PLOTTING
% x=[]; d2x=[]; tset=[]; dx=[];
% for time=t0_real:tf_real
%     tset=[tset  time];
%     traj_res=traj(p,time,n,t0_real,tf_real);
%     x=[x traj_res.x];
%     dx=[dx traj_res.dx];
%     d2x=[d2x traj_res.d2x];
% end
% 
% 
% figure
% 
% % tmpAspect=daspect();
% % daspect(tmpAspect([1 2 2]))
% hold on 
% 
% plot3(x0(1),x0(2),x0(3),'bo')
% plot3(xf(1),xf(2),xf(3),'ro')
% plot3(x(1,:), x(2,:), x(3,:))
% 
% 
% for time=linspace(t0_real,tf_real,20)
% 
% traj_res=traj(p,time,n,t0_real,tf_real);
% 
% %quiver3(traj_res.x(1),traj_res.x(2),traj_res.x(3),traj_res.d2x(1)/norm(traj_res.d2x),traj_res.d2x(2)/norm(traj_res.d2x),traj_res.d2x(3)/norm(traj_res.d2x),'Color','r','LineWidth',1,'MaxHeadSize',2);
% quiver3(traj_res.x(1),traj_res.x(2),traj_res.x(3),traj_res.d2x(1),traj_res.d2x(2),traj_res.d2x(3),'Color','r','LineWidth',1,'MaxHeadSize',2);
% end
% 
% figure
% title('ad')
% for i=1:3
%     subplot(3,1,i)
%     plot(tset,d2x(i,:))
% end
% 
% figure
% for i=1:3
%     subplot(3,1,i)
% title('vd')
%     plot(tset,dx(i,:))
% end

%% PLOTTING 
% the original problem
% x=[]; d2x=[]; tset=[];
% for time=t0_real:tf_real
%     tset=[tset  time];
%     traj_res=get_flats(time);
%     x=[x traj_res.x];
%     d2x=[d2x traj_res.d2x];
% end
% 
% 
% figure
% 
% % tmpAspect=daspect();
% % daspect(tmpAspect([1 2 2]))
% hold on 
% 
% plot3(x(1,:), x(2,:), x(3,:))
% 
% 
% for time=linspace(t0_real,tf_real,20)
% 
% traj_res=traj(p,time,n,t0,tf);
% 
% %quiver3(traj_res.x(1),traj_res.x(2),traj_res.x(3),traj_res.d2x(1)/norm(traj_res.d2x),traj_res.d2x(2)/norm(traj_res.d2x),traj_res.d2x(3)/norm(traj_res.d2x),'Color','r','LineWidth',1,'MaxHeadSize',2);
% quiver3(traj_res.x(1),traj_res.x(2),traj_res.x(3),traj_res.d2x(1),traj_res.d2x(2),traj_res.d2x(3),'Color','r','LineWidth',1,'MaxHeadSize',2);
% end
% 
% figure
% 
% for i=1:3
%     subplot(3,1,i)
%     plot(tset,x(i,:))
% end
% 

