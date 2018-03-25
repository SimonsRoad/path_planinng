function H=RegMatrix3D(N)
%% ====================================================================
% Author: Mohammad Rouhani, Morpheo Team, INRIA Rhone Alpes, (2013)
% Email: mohammad.rouhani@inria.fr
% Title: convolutions between two B-Spline basis functions
% Place of publication: Grenoble, France
% Available from: URL
% http://www.iis.ee.ic.ac.uk/~rouhani/mycodes/IBS.rar
%====================================================================
% When using this software, PLEASE ACKNOWLEDGE the effort that went
% into development BY REFERRING THE PAPER:
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
% Rouhani M. and Sappa A.D., Implicit B-spline fitting using the 3L
% algorithm, IEEE Conference on on Image Processing (ICIP'11), 2011.
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%% ====================================================================
%About this code:
%Input: N: the size of IBS.
%Output: H: IBS Regularization matrix (p'Hp: regularization term).

%This function makes the regularization matrix needed for the L3.
%The output R is a matrix showing the coefficients to minimze the bending
%energy.
%we start moving on the whole net, and making the term for the
%corresponding control parameters.
%H=zeros(N^3,N^3);
tic;
%H=sparse(N^3,N^3); %THIS IS MY MISTAKE!!! SHOULDN'T BE SPARSE
H=zeros(N^3,N^3);
[Pi0 Pi1 Pi2]=BConV(N);

% We must multiply these values based on (33) in TIP'14
% Repeat these matrices properly to construct the N^3xN^3 Regularization
%For the first index (k_2,l_2):
%Pi0_1=repmat(Pi0,N^2,N^2); Pi1_1=repmat(Pi1,N^2,N^2); Pi2_1=repmat(Pi2,N^2,N^2);

%2nd version: FASTEST::::::::::::::::::::::::::::::::::::::::::
w1=waitbar(0,'Constructing the regularization matrix');
Pi0_1=repmat(Pi0,N,N); Pi1_1=repmat(Pi1,N,N); Pi2_1=repmat(Pi2,N,N);
Pi0_1=repmat(Pi0_1,N,N); Pi1_1=repmat(Pi1_1,N,N); Pi2_1=repmat(Pi2_1,N,N);
%For the second index (k_2,l_2): build the blocks first:
Pi0_2=zeros(N^2,N^2); Pi1_2=Pi0_2; Pi2_2=Pi0_2;
for k2=1:N
    waitbar(k2/(2*N),w1);
    for l2=1:N
        ind1=(k2-1)*N+(1:N); ind2=(l2-1)*N+(1:N);
        Pi0_2(ind1,ind2)=Pi0(k2,l2); Pi1_2(ind1,ind2)=Pi1(k2,l2); Pi2_2(ind1,ind2)=Pi2(k2,l2);
    end
end
Pi0_2=repmat(Pi0_2,N,N); Pi1_2=repmat(Pi1_2,N,N); Pi2_2=repmat(Pi2_2,N,N);
for k3=1:N
    waitbar((N+k3)/(2*N),w1);
    for l3=1:N
        ind1=(k3-1)*N^2+(1:N^2); ind2=(l3-1)*N^2+(1:N^2);
        Pi0_3(ind1,ind2)=Pi0(k3,l3); Pi1_3(ind1,ind2)=Pi1(k3,l3); Pi2_3(ind1,ind2)=Pi2(k3,l3);
    end
end


%Export the regularization matrix:
H=Pi2_1.*Pi0_2.*Pi0_3+ 2*Pi1_1.*Pi1_2.*Pi0_3+Pi0_1.*Pi2_2.*Pi0_3...
    +2*Pi1_1.*Pi0_2.*Pi1_3+2*Pi0_1.*Pi1_2.*Pi1_3+Pi0_1.*Pi0_2.*Pi2_3;
H=reshape(H,N^3,N^3);
toc

function [Pi0, Pi1, Pi2]=BConV(N)
%% It computes the convolutions between two B-Splines their derrivatives.
% These terms is used to construct the regularization matrix.
% Output: Pi^(k)_{m,n}=\int B^(k)_m(x) B^(k)_n(x) dx
B=[-1 3 -3 1; 3 -6 3 0; -3 0 3 0; 1 4 1 0]/6;
%[t^3 t^2 t 1] will be multiplied from the left hand side!
%so its derivaive will be as follows:
dB=[0 0 0 0; -3 9 -9 3; 6 -12 6 0; -3 0 3 0]/6;
d2B=[0 0 0 0; 0 0 0 0; -6 18 -18 6; 6 -12 6 0]/6;
% constructing 4x4 matrices made out of the blending functions.
% the blending functions are multiplied and integrated over [0,1]
pi0=INTofMULT(B,B);
pi1=INTofMULT(dB,dB);
pi2=INTofMULT(d2B,d2B);

Pi0=zeros(N); Pi1=zeros(N); Pi2=zeros(N);
%We start moving on the active region (the interval [0 1]).
stepX=1/(N-3);
for i=1:N-3
    % in each subsection there will be 4 active basis in one side
    % to be convolved with 4 other in the other side.
    for r=0:3
        for s=0:3
            %b_r is associated to B(i+r) & b_s is associated to B(i+s).
            Pi0(i+r,i+s)=Pi0(i+r,i+s)+pi0(r+1,s+1)*stepX;
            Pi1(i+r,i+s)=Pi1(i+r,i+s)+pi1(r+1,s+1)*stepX;
            Pi2(i+r,i+s)=Pi2(i+r,i+s)+pi2(r+1,s+1)*stepX;
        end
    end
end

function C=INTofMULT(A,B)
%% C_{ij}=\int [t^3 t^2 t 1]*A(:,i) \times [t^3 t^2 t 1]*B(:,j) dt
C=zeros(4);
for i=1:4
    for j=1:4
        %consider two different columns (showing two basised):
        W=MULT(A(:,i),B(:,j)); % [t^6 t^5 t^4 t^3 t^2 t 1]*W
        sum=0;
        for k=1:7
            sum=sum+W(k)/(8-k); %\int [t^6 t^5 t^4 t^3 t^2 t 1]*W dt
        end
        C(i,j)=sum;
    end
end

function W=MULT(U,V)
%% algebraic product of two column vectors U & V:
% [t^3 t^2 t 1]*U \times [t^3 t^2 t 1]*V
% ===========[t^6 t^5 t^4 t^3 t^2 t 1]*W
W=zeros(7,1);
for i=1:4
    for j=1:4
        %consider the term U(i) & V(j) which is coef. of t^(4-i) & t^(4-j)
        %so the multiply has the power (8-i-j) and must be saved in C(i+j-1)
        W(i+j-1)=W(i+j-1)+U(i)*V(j);
    end
end