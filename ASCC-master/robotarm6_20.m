
%% 6/19
%% robot arm trajectory following 
% Link define
nbData=50;
LinkLength=[4 4 4];
global arms dof arms_flat 
global alpha0_max eta po beta lambda 
global obs

dof=3;


arms=cell(dof,1); arms_flat=cell(2*dof,1);
eta=200; % need to be tuned
alpha0_max=0.8;
po=1;
beta=1.2;
lambda=1.2;


% robot arm(serial Link) defined

for idx=1:dof
    
    if idx==1    
    arms{idx}.mid=SerialLink([0 0 LinkLength(idx)/2 0]);
    arms{idx}.end=arms{idx}.mid*Link([0 0 LinkLength(idx)/2 0]);
    arms_flat{2*idx-1}=arms{idx}.mid;
    arms_flat{2*idx}=arms{idx}.end;
    else
    
    arms{idx}.mid=arms{idx-1}.end*Link([0 0 LinkLength(idx)/2 0]);
    arms{idx}.end=arms{idx}.mid*Link([0 0 LinkLength(idx)/2 0]);
    arms_flat{2*idx-1}=arms{idx}.mid;
    arms_flat{2*idx}=arms{idx}.end;    
    end
    
    
end


%% end effector trajectory avoidance
%% learning and reproduction 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parameter 
nbVar = 3; %Number of variables (Trajectory in a plane)
nbStates = 8; %Number of states (or primitives) 
nbRepros=1;
K =300; %Initial stiffness gain
D = 20; %Damping gain
dt = .01; %Time step
%Decay factor
alpha = 1.0;

%Centers equally distributed
Mu_d = linspace(nbData,1,nbStates);
Mu_s=zeros(1,nbStates);
% how wide should i spread each function 
Sigma_d = 400;
%Estimate Mu_s and Sigma_s to match Mu_d and Sigma_d
Mu_s(1,:) = exp(-alpha*Mu_d*dt);
for i=1:nbStates
  std_s = Mu_s(1,i) - exp(-alpha*(Mu_d(i)+Sigma_d^.5)*dt);
  Sigma_s(:,:,i) = std_s^2;
end

% path generation 
q0=[1e-4 0 pi/6 0 pi/4 0];
[T0,all]=arms{dof}.end.fkine(q0);
figure()
title('initial pos')
axis([-16 12 -12 12])
arms{dof}.end.plot(q0,'top','tilesize',6)

dispx=-15;
dispy=0;


x0=T0.t;
xg=x0+dispx*[1 0 0]'+dispy*[0 1 0]';
Tg=T0; Tg.t=xg;
Ts=(ctraj(T0.T,Tg.T,nbData)); % nominal trajectory 

xs=reshape(Ts(1:3,4,:),3,nbData);
obs=xs(:,nbData/2)+[-1.5 0.1 0]';

xdots=computeDerivative(xs,dt);
xddots=computeDerivative(xdots,dt);


figure()
hold on
title('obstacle avoidance')
axis([-16 12 -12 12])
plot(obs(1),obs(2),'r*')
draw_circle(obs(1),obs(2),po)
plot(xs(1,:),xs(2,:),'k--')
plot(xg(1),xg(2),'ko')
hold off



%% path learning 
%WLS learning: least norm solution to find weight (Y=Mu_x*H')
s = 1; %Decay term
for n=1:nbData
  s = s + (-alpha*s)*dt;
  sList(n) = s;
  for i=1:nbStates
    h(i) = gaussPDF(s,Mu_s(:,i),Sigma_s(:,:,i));
  end
  %Compute weights
  H(n,:) = h./sum(h)*s;
end

f = inv(K)*(xddots - ((repmat(xg,1,nbData)-xs)*K - xdots*D-K*repmat((xg-x0),1,nbData).*repmat(sList,3,1)));
w = [inv(H'*H)*H'*f']';


%% reproduction  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nb=1;
rWLS(nb).currPos = xs(:,1); %Initial position
x0=rWLS(nb).currPos; 

rWLS(nb).currVel = zeros(nbVar,1); %Initial velocity
rWLS(nb).currAcc = zeros(nbVar,1); %Initial acceleration
s = 1; %Decay term
for n=1:nbData
%Log data
rWLS(nb).Data(:,n) = [rWLS(nb).currPos; rWLS(nb).currVel; rWLS(nb).currAcc];
%Update s (ds=-alpha*s)
s = s + (-alpha*s)*dt;
%Activation weights with WLS
for i=1:nbStates
  rWLS(nb).H(i,n) = gaussPDF(s,Mu_s(:,i),Sigma_s(:,:,i));
end
rWLS(nb).H(:,n) = rWLS(nb).H(:,n) / sum(rWLS(nb).H(:,n));

%Evaluate acceleration with WLS
currF0 = (xg-rWLS(nb).currPos)*K - rWLS(nb).currVel*D-K*(xg-x0)*s;
rWLS(nb).F(:,n) = w * rWLS(nb).H(:,n)*s;
rWLS(nb).currAcc = currF0 + K*rWLS(nb).F(:,n) +Calc_Psi(rWLS(nb).currPos,rWLS(nb).currVel);
rWLS(nb).currVel = rWLS(nb).currVel + rWLS(nb).currAcc * dt;
%Update position 

rWLS(nb).currPos = rWLS(nb).currPos + rWLS(nb).currVel * dt;

end

figure()
hold on
title('obstacle avoidance')
axis([-16 12 -12 12])
plot(obs(1),obs(2),'r*')
draw_circle(obs(1),obs(2),po)
plot(rWLS.Data(1,:),rWLS.Data(2,:))
plot(xg(1),xg(2),'ko')
hold off


%%
% solving inverse kinematics - nominal path 
xdots_nominal=rWLS.Data(4:6,:);


qs=zeros(dof,nbData);
qs2=zeros(2*dof,nbData);
figure()
axis([-16 12 -12 12])
for t=1:nbData
   
   if t==1
        qs(:,t)=q0(1:2:2*dof);
        
   else
       J=(arms{dof}.end.jacob0(prev_q2));
       J=J(1:2,1:2:2*dof);
       [x0,J0,x0dot,do]=MK4(prev_q');
       J0=J0(1:2,:);
   
       null_vel=alphah(do)*pinv(J0*(eye(dof)-pinv(J)*J),0.0001)*(alphao(do)*x0dot(1:2)-J0*pinv(J)*xdots_nominal(1:2,t));    
%        disp(J0*(eye(dof)-pinv(J)*J))
       
       DMP_vel=(pinv(J)*xdots_nominal(1:2,t));
       

%    
%         fprintf('nominal angular velocity/   nullspace veclocity\n')
%        for i=1:dof
%        fprintf('%4.4f          %4.4f\n',DMP_vel(i),null_vel(i)) 
%        end
%        
 
       thetadot=DMP_vel;
            
       
       qs(:,t)=prev_q+thetadot*dt;
       
       
   end    
   hold on
   
   prev_q=qs(:,t);

   prev_q2=reshape([prev_q'; zeros(1,dof)],1,2*dof);
   qs2(:,t)=prev_q2;
   
   plot(obs(1),obs(2),'r*')
   draw_circle(obs(1),obs(2),po/3)
   
   plot(rWLS.Data(1,t:end),rWLS.Data(2,t:end))
   plot(xg(1),xg(2),'ko')
   
   arms{dof}.end.plot(prev_q2,'top','tilesize',5,'nowrist')
   axis([-16 12 -12 12])

   if t~=1
%    quiver(x0(1),x0(2),x0dot(1),x0dot(2),'b')
   end
   hold off

end

%% avoidance movie
   hold on

   plot(obs(1),obs(2),'r*')
   draw_circle(obs(1),obs(2),po)
   
   plot(rWLS.Data(1,:),rWLS.Data(2,:))

   plot(xg(1),xg(2),'ko')
   
   arms{dof}.end.plot(qs2','top','tilesize',6,'nowrist')

   hold off


%%

% %%
% q0=[1e-4 pi/7 pi/4]';
% dispx=-17;
% %define robot arm - in this case 
% 
% global arm arm1 arm2 
% global obs po eta
% global alpha0_max
% eta=1000; % need to be tuned
% alpha0_max=10;
% po=3;
% obs=[3 7 0]';
% 
% 
% 
% 
% arm=SerialLink(L,'name','three link');
% arm1=SerialLink(L(1),'name','1st_link');
% arm2=SerialLink(L(1:2),'name','2nd_link');
% 
% % define path= linear 
% T0=arm.fkine(q0); x0=T0(1:3,4);
% xg=x0+dispx*[1 0 0]';
% Tg=[T0(1:3,1:3) xg ; zeros(1,3) 1];
% 
% % trajectory generatation 
% Ts=ctraj(T0,Tg,nbData); % nominal trajectory 
% Qs=zeros(nbData,3);
% dt=0.01;
% xs=reshape(Ts(1:3,4,:),3,nbData);
% xs_tmp=xs(:,2:end); xs_tmp=[xs_tmp xs_tmp(:,end)];
% xdots=(xs_tmp-xs)/dt;
% xdots=xdots(1:2,:);
% % solving inverse kinematics - nominal path 
% qs=zeros(3,nbData);
% figure()
% axis([-16 12 -12 12])
% for t=1:nbData
%    
%    if t==1
%         qs(:,t)=q0;
%    else
%        J=(arm.jacob0(prev_q));
%        J=J(1:2,:);
%        [x0,J0,x0dot,alpha0]=MK(prev_q); 
%        null_vel=pinv(J0*(eye(3)-pinv(J)*J))*(x0dot-J0*pinv(J)*xdots(:,t));
%         if norm(null_vel)>1e+4
%         alpha0=0;
%         disp('singular')
%         end
%        thetadot=(pinv(J)*xdots(:,t))+alpha0*(eye(3)-pinv(J)*J)*null_vel;
%        qs(:,t)=prev_q+thetadot*dt;
%    end
%    hold on
% 
%    prev_q=qs(:,t);
% 
%    % plotting 
%    plot(obs(1),obs(2),'r*')
%    draw_circle(obs(1),obs(2),po)
%    
%    arm.plot(prev_q','top')
%    
%    
%    hold off
% end
% % % plot again
% arm.plot(qs','top')
% 
% % 
% % % solving inverse kinematics - avoidance
% % for t=1:nbData
% %    
% %    if t==1
% %         qs(:,t)=q0;
% %    else
% %        J=(arm.jacob0(prev_q));
% %        J=J(1:3,:);
% %        qs(:,t)=prev_q+(pinv(J)*xdots(:,t))*dt;
% %        
% %    end
% % 
% %    prev_q=qs(:,t);
% % end
% % 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 









