function [w,Mu_s,Sigma_s]=DMP_learning(nbData,nbVar,alpha,dt,nbStates,K,D,xs)
% demopath(xs) = D x nbdata 
% nbStates= Number of states (or primitives) 

x0=xs(:,1);
xg=xs(:,end);

xdots=computeDerivative(xs,dt);
xddots=computeDerivative(xdots,dt);


%Number of states (or primitives) 


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

f = inv(K)*(xddots - ((repmat(xg,1,nbData)-xs)*K - xdots*D-K*repmat((xg-x0),1,nbData).*repmat(sList,nbVar,1)));
w = [inv(H'*H)*H'*f']';









