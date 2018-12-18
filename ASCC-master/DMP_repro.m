function [x_repro,xdot_repro]=DMP_repro(nbData,nbVar,alpha,dt,nbStates,K,D ,w,Mu_s,Sigma_s,start_step,start_pos,x0_repro,xg,ispotential)
global obs1_path obs2

nb=1;

rWLS(nb).currPos = start_pos; %Initial position

rWLS(nb).currVel = zeros(nbVar,1); %Initial velocity
rWLS(nb).currAcc = zeros(nbVar,1); %Initial acceleration

s = 1; %Decay term
for n=1:nbData
  s = s + (-alpha*s)*dt;
  sList(n) = s;
end

for n=start_step:nbData
%Log data
rWLS(nb).Data(:,n) = [rWLS(nb).currPos; rWLS(nb).currVel; rWLS(nb).currAcc];
%Update s (ds=-alpha*s)
s=sList(n);
%Activation weights with WLS
for i=1:nbStates
  rWLS(nb).H(i,n) = gaussPDF(s,Mu_s(:,i),Sigma_s(:,:,i));
end
rWLS(nb).H(:,n) = rWLS(nb).H(:,n) / sum(rWLS(nb).H(:,n));

%Evaluate acceleration with WLS
currF0 = (xg-rWLS(nb).currPos)*K - rWLS(nb).currVel*D-K*(xg-x0_repro)*s;
rWLS(nb).F(:,n) = w * rWLS(nb).H(:,n)*s;


rWLS(nb).currAcc = currF0 + K*rWLS(nb).F(:,n);

if ispotential
rWLS(nb).currAcc=rWLS(nb).currAcc+Calc_Psi_obsinput(obs1_path(n,:),rWLS(nb).currPos,rWLS(nb).currVel)+Calc_Psi_obsinput(obs2,rWLS(nb).currPos,rWLS(nb).currVel); 
end


rWLS(nb).currVel = rWLS(nb).currVel + rWLS(nb).currAcc * dt;
%Update position 

rWLS(nb).currPos = rWLS(nb).currPos + rWLS(nb).currVel * dt;
end

x_repro=rWLS(nb).Data(1:nbVar,:);
xdot_repro=rWLS(nb).Data(nbVar+1:2*nbVar,:);



end

