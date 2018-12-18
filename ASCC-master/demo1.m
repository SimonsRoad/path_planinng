%
% Estimation of the parameters of a DMP (dynamic movement primitives) through GMR (Gaussian mixture regression). 
% A DMP is composed of a virtual spring-damper system modulated by a non-linear force. 
% The standard method to train a DMP is to predefine a set of activations functions and estimate a set of force 
% components through a weighted least-squares (WLS) approach. 
% The weighted sum of force components form a non-linear force perturbing the system,
% by moving it away from the point-to-point linear motion while following a desired trajectory.
% GMR is used here to learn the joint distribution between the decay term s (determined by a canonical dynamical
% system) and the non-linear force variable to estimate.
% Replacing WLS with GMR has the following advantages:
% 1) It provides a probabilistic formulation of DMP (e.g., to allow the exploitation of correlation and variation
% information, and to make the DMP approach compatible with other statistical machine learning tools).
% 2) It simultaneously learns the non-linear force together with the activation functions. Namely, the Gaussian 
% kernels do not need to be equally spaced in time (or at predefined values of the decay term 's'), and the 
% bandwidths (variance of the Gaussians) are automatically estimated from the data instead of being hand-tuned.
% 3) It provides a more accurate approximation of the non-linear perturbing force with local linear models of 
% degree 1 instead of degree 0 (by exploiting the conditional probability properties of Gaussian distributions).  
%
% Author:	Sylvain Calinon, 2012
%         http://programming-by-demonstration.org/SylvainCalinon
%
% This source code is given for free! In exchange, I would be grateful if you cite  
% the following reference in any academic publication that uses this code or part of it: 
%
% @inproceedings{Calinon12Hum,
%   author="Calinon, S. and Li, Z. and Alizadeh, T. and Tsagarakis, N. G. and Caldwell, D. G.",
%   title="Statistical dynamical systems for skills acquisition in humanoids",
%   booktitle="Proc. {IEEE} Intl Conf. on Humanoid Robots ({H}umanoids)",
%   year="2012",
%   address="Osaka, Japan"
% }

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 200; %Length of each trajectory
nbVar = 2; %Number of variables (Trajectory in a plane)
nbStates = 7; %Number of states (or primitives) 
nbRepros = 5;
K = 100; %Initial stiffness gain
D = 20; %Damping gain
dt = .01; %Time step

%Decay factor
alpha = 1.0;
%Centers equally distributed
Mu_d = linspace(nbData,1,nbStates);
Sigma_d = 2000;
%Estimate Mu_s and Sigma_s to match Mu_d and Sigma_d
Mu_s(1,:) = exp(-alpha*Mu_d*dt);
for i=1:nbStates
  std_s = Mu_s(1,i) - exp(-alpha*(Mu_d(i)+Sigma_d^.5)*dt);
  Sigma_s(:,:,i) = std_s^2;
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
%WLS learning: least norm solution to find Mu_X (Y=Mu_x*H')
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
f = Data(accId,:) - [(repmat(xT,1,nbData)-Data(posId,:))*kP - Data(velId,:)*kV-K*()];
Mu_F = [inv(H'*H)*H'*f']';

%GMR EM learning: estiamte the joint distribution of s and Y.
DataEM = [sList; f];
[Priors, Mu, Sigma] = EM_init_regularTiming(DataEM, nbStates);
[Priors, Mu, Sigma] = EM_boundingCov(DataEM, Priors, Mu, Sigma);

%% Reproductions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
in=[1]; out=[2:3];
for nb=1:nbRepros
  if nb==1
    rWLS(nb).currPos = Data(posId,1); %Initial position
  else
    rWLS(nb).currPos = Data(posId,1) + (rand(2,1)-0.5).*5; %Initial position (with noise)
  end
  rGMR(nb).currPos = rWLS(nb).currPos; %Initial position
  rWLS(nb).currVel = zeros(nbVar,1); %Initial velocity
  rGMR(nb).currVel = zeros(nbVar,1); %Initial velocity
  rWLS(nb).currAcc = zeros(nbVar,1); %Initial acceleration
  rGMR(nb).currAcc = zeros(nbVar,1); %Initial acceleration
  s = 1; %Decay term
  for n=1:nbData
    %Log data
    rWLS(nb).Data(:,n) = [rWLS(nb).currPos; rWLS(nb).currVel; rWLS(nb).currAcc];
    rGMR(nb).Data(:,n) = [rGMR(nb).currPos; rGMR(nb).currVel; rGMR(nb).currAcc];
    %Update s (ds=-alpha*s)
    s = s + (-alpha*s)*dt;
    %Activation weights with WLS
    for i=1:nbStates
      rWLS(nb).H(i,n) = gaussPDF(s,Mu_s(:,i),Sigma_s(:,:,i));
    end
    rWLS(nb).H(:,n) = rWLS(nb).H(:,n) / sum(rWLS(nb).H(:,n));
    %Activation weights with GMR
    for i=1:nbStates
      rGMR(nb).H(i,n) = Priors(i) * gaussPDF(s,Mu(in,i),Sigma(in,in,i));
    end
    rGMR(nb).H(:,n) = rGMR(nb).H(:,n) / sum(rGMR(nb).H(:,n));
    %Evaluate acceleration with WLS
    currF0 = (xT-rWLS(nb).currPos)*K - rWLS(nb).currVel*D;
    rWLS(nb).F(:,n) = Mu_F * rWLS(nb).H(:,n)*s;
    rWLS(nb).currAcc = currF0 + rWLS(nb).F(:,n);
    %Evaluate acceleration with GMR
    currF0 = (xT-rGMR(nb).currPos)*K - rGMR(nb).currVel*D;
    rGMR(nb).F(:,n) = zeros(nbVar,1);
    for i=1:nbStates
      currFTmp = Mu(out,i) + Sigma(out,in,i)*inv(Sigma(in,in,i)) * (s-Mu(in,i));
      rGMR(nb).F(:,n) = rGMR(nb).F(:,n) + rGMR(nb).H(i,n) * currFTmp;
    end
    rGMR(nb).currAcc = currF0 + rGMR(nb).F(:,n); 
    %Update velocity 
    rWLS(nb).currVel = rWLS(nb).currVel + rWLS(nb).currAcc * dt;
    rGMR(nb).currVel = rGMR(nb).currVel + rGMR(nb).currAcc * dt;
    %Update position 
    rWLS(nb).currPos = rWLS(nb).currPos + rWLS(nb).currVel * dt;
    rGMR(nb).currPos = rGMR(nb).currPos + rGMR(nb).currVel * dt;
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
subplot(5,2,[1,3,5]); hold on; box on;
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
set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%WLS (non-linear force)
subplot(5,2,7); hold on; box on;
plot(f(1,:),'-','linewidth',5,'color',[.75 .75 .75]);
for nb=1:nbRepros
  plot(rWLS(nb).F(1,:),'-','linewidth',2,'color',[0 0 0]);
end
xlabel('t'); ylabel('F_1');
set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%WLS (activation weights)
subplot(5,2,9); hold on; box on;
for i=1:nbStates
  plot(rWLS(1).H(i,:),'-','linewidth',2,'color',clrmap(i,:));
end
axis([1 nbData 0 1.1]); xlabel('t'); ylabel('h_i');
set(gca,'Xtick',[]); set(gca,'Ytick',[]);

%GMR (reproductions)
subplot(5,2,[2,4,6]); hold on; box on;
plot(Data(1,:),Data(2,:),'-','linewidth',5,'color',[.75 .75 .75]);
for nb=1:nbRepros
  plot(rGMR(nb).Data(1,:),rGMR(nb).Data(2,:),'-','linewidth',5,'color',[1 1 1]);
  for n=2:1:nbData
    colTmp = rGMR(nb).H(:,n)' * clrmap;
    plot(rGMR(nb).Data(1,[n-1:n]),rGMR(nb).Data(2,[n-1:n]),'-','linewidth',3,'color',colTmp);
  end
end
title('DMP with GMR','fontsize',16);
axis equal; axis([-10 11 -10 11]); xlabel('x_1'); ylabel('x_2');
set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%WLS (non-linear force)
subplot(5,2,8); hold on; box on;
plot(f(1,:),'-','linewidth',5,'color',[.75 .75 .75]);
for nb=1:nbRepros
  plot(rGMR(nb).F(1,:),'-','linewidth',2,'color',[0 0 0]);
end
xlabel('t'); ylabel('F_1');
set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%GMR (activation weights)
subplot(5,2,10); hold on; box on;
for i=1:nbStates
  plot(rGMR(1).H(i,:),'-','linewidth',2,'color',clrmap(i,:));
end
axis([1 nbData 0 1.1]); xlabel('t'); ylabel('h_i');
set(gca,'Xtick',[]); set(gca,'Ytick',[]);

pause;
close all;



