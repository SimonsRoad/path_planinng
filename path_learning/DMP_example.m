%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 200; %Length of each trajectory
nbVar = 2; %Number of variables (Trajectory in a plane)
nbStates = 12; %Number of states (or primitives) 
nbRepros = 5;
K =40; %Initial stiffness gain
D = 10; %Damping gain
dt = .01; %Time step

%Decay factor
alpha = 1.0;
%Centers equally distributed
Mu_d = linspace(nbData,1,nbStates);
% how wide should i spread each function 
Sigma_d = 800;
%Estimate Mu_s and Sigma_s to match Mu_d and Sigma_d
Mu_s(1,:) = exp(-alpha*Mu_d*dt);
for i=1:nbStates
  std_s = Mu_s(1,i) - exp(-alpha*(Mu_d(i)+Sigma_d^.5)*dt);
  Sigma_s(:,:,i) = std_s^2;
end


%% obstacle avoidance 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% obstacle position 
global o eta po lambda beta
o=[5 7]';
lambda=4;
beta=2; % paper set
eta=1000;
po=4;


%% Drawing arm and trajectory
figure()
plotArm(theta_init,LinkLength,base)
hold on
plot(path_demo(1,:),path_demo(2,:))
axis([-12 12 -12 12])
box on; axis square;

%% solving inverse kinematics 
theta=theta_init; x=startPos;
cur_theta=theta_init; cur_x=startPos;
for n=1:nbData-1
dx=path_demo(:,n+1)-path_demo(:,n);

dtheta=subs_JX(cur_theta(1),cur_theta(2),LinkLength(1),LinkLength(2))*dx;
    
cur_theta=cur_theta+dtheta;

theta=[theta cur_theta];
end

% check the inverse kinematics 
Pos=zeros(2,nbData);
for n=1:nbData
Pos(:,n)=subs_x(theta(1,n),theta(2,n),LinkLength(1),LinkLength(2),base(1),base(2));
end







%% Load dataset 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%'Data' is a 2D trajectory of 200 points.
load('data/Data.mat');
%Data(posId,:), Data(velId,:) and Data(accId,:) correspond to position, velocity and acceleration variables.
posId=[1:nbVar]; velId=[nbVar+1:2*nbVar]; accId=[2*nbVar+1:3*nbVar]; 
%Define end-point as target
xT = Data(posId,end);
x0=Data(posId,1);

%Estimate of derivatives with polynomial fitting 
Data(velId,:) = computeDerivative(Data(posId,:), dt); %Velocity
Data(accId,:) = computeDerivative(Data(velId,:), dt); %Acceleration

%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

f = inv(K)*(Data(accId,:) - ((repmat(xT,1,nbData)-Data(posId,:))*K - Data(velId,:)*D-K*repmat((xT-x0),1,nbData).*repmat(sList,2,1)));
w = [inv(H'*H)*H'*f']';


%% Reproductions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
in=[1]; out=[2:3];
for nb=1:nbRepros
  if nb==1
    rWLS(nb).currPos = Data(posId,1); %Initial position
  else
    rWLS(nb).currPos = Data(posId,1) + (rand(2,1)-0.5).*5; %Initial position (with noise)
  end
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
    currF0 = (xT-rWLS(nb).currPos)*K - rWLS(nb).currVel*D-K*(xT-x0)*s;
    rWLS(nb).F(:,n) = w * rWLS(nb).H(:,n)*s;
    rWLS(nb).currAcc = currF0 + K*rWLS(nb).F(:,n);

    rWLS(nb).currVel = rWLS(nb).currVel + rWLS(nb).currAcc * dt;
    %Update position 
    rWLS(nb).currPos = rWLS(nb).currPos + rWLS(nb).currVel * dt;
  end
end %nb

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,50,700,700]);
%Create colormap to obtain a different color for each state 
clrmap = colormap('Jet');
xx = round(linspace(1,64,nbStates));
clrmap = clrmap(xx,:);

%WLS (reproductions)
subplot(6,1,[1,2,3]); hold on; box on;
plot(Data(1,:),Data(2,:),'-','linewidth',5,'color',[.75 .75 .75]);
for nb=1:nbRepros
  plot(rWLS(nb).Data(1,:),rWLS(nb).Data(2,:),'-','linewidth',5,'color',[1 1 1]);
  for n=2:1:nbData
    colTmp = rWLS(nb).H(:,n)' * clrmap;
    plot(rWLS(nb).Data(1,[n-1:n]),rWLS(nb).Data(2,[n-1:n]),'-','linewidth',3,'color',colTmp);
  end
end
title('DMP with WLS','fontsize',16);
axis equal; axis([-10 11 -10 11]); xlabel('x_1'); ylabel('x_2');
% set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%WLS (non-linear force)
subplot(6,1,4); hold on; box on;
plot(f(1,:),'-','linewidth',5,'color',[.75 .75 .75]);
for nb=1:nbRepros
  plot(rWLS(nb).F(1,:),'-','linewidth',2,'color',[0 0 0]);
end
xlabel('t'); ylabel('F_1');
set(gca,'Xtick',[]); set(gca,'Ytick',[]);


subplot(6,1,5); hold on; box on;
plot(f(2,:),'-','linewidth',5,'color',[.75 .75 .75]);
for nb=1:nbRepros
  plot(rWLS(nb).F(2,:),'-','linewidth',2,'color',[0 0 0]);
end
xlabel('t'); ylabel('F_2');
set(gca,'Xtick',[]); set(gca,'Ytick',[]);

%WLS (activation weights)


subplot(6,1,6); hold on; box on;

for i=1:nbStates
  plot(rWLS(1).H(i,:),'-','linewidth',2,'color',clrmap(i,:));
end
axis([1 nbData 0 1.1]); xlabel('t'); ylabel('h_i');
set(gca,'Xtick',[]); set(gca,'Ytick',[]);



