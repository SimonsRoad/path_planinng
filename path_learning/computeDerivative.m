function rderivdata = computeDerivative(rdata,dt)

%Euler approximation
rderivdata = [[rdata(:,2:end) rdata(:,end)] - rdata]/dt;