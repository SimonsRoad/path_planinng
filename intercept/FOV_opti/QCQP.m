function x_opt=QCQP(Q,f,c,H,k,d,A,b,Aeq,beq,x0)
%% Example 
% https://kr.mathworks.com/help/optim/ug/linear-or-quadratic-problem-with-quadratic-constraints.html
% Q = [3,2,1;
%      2,4,0;
%      1,0,5];
% f = [-24;-48;-130];
% c = -2;
% 
% rng default % for reproducibility
% % Two sets of random quadratic constraints:
% H{1} = gallery('randcorr',3); % random positive definite matrix
% H{2} = gallery('randcorr',3);
% k{1} = randn(3,1);
% k{2} = randn(3,1);
% d{1} = randn;
% d{2} = randn;

% 
% x0 = [0;0;0]; % column vector

%% 
options = optimoptions(@fmincon,'Algorithm','interior-point',...
    'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
    'HessianFcn',@(x,lambda)quadhess(x,lambda,Q,H));

[x_opt,fval,eflag,output,lambda] = fmincon(@(x) quadobj(x,Q,f,c),x0,...
    A,b,Aeq,beq,[],[],@(x) quadconstr(x,H,k,d),options);