function H=RegMatrix3D_T(N,IND)
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
%Input: N: the size of IBS; IND: the indices of active parameters.
%Output: H: IBS Regularization matrix (p'Hp: regularization term).

%This function makes the regularization matrix needed for the L3.
%The output R is a matrix showing the coefficients to minimze the bending
%energy.
%we start moving on the whole net, and making the term for the
%corresponding control parameters.
%H=zeros(N^3,N^3);

%This function makes the regularization matrix needed for the L3.
%The output R is a matrix showing the coefficients to minimze the bending
%energy.
%we start moving on the whole net, and making the term for the
%corresponding control parameters.
whichVersion='1st'; '2nd'; '1st';

DIM=max(IND(:));
H=zeros(DIM,DIM);
[Pi0 Pi1 Pi2]=BConV(N);

%disp(['The chosen version is the ',whichVersion,'-one.'])
if whichVersion=='2nd'
    %2nd version: HIGHLY FAST!!!!::::::::::::::::::::::::::::::::::::::::::
    vk=find(IND(:)>0); %ROWS
    vk=reshape(repmat(vk',DIM,1),DIM^2,1);
    k=IND(vk);  %(k1,k2,k3) %k=(k3-1)*N^2+(k2-1)*N+k1;
    k3=floor((vk-1)/N^2)+1; k2=floor((vk-1)/N)-(k3-1)*N+1; k1=vk-(k2-1)*N-(k3-1)*N^2;
        
    vl=find(IND(:)>0); %COLUMNS
    vl=reshape(repmat(vl,1,DIM),DIM^2,1);
    l=IND(vl); %(il,jl,kl) %l=(il-1)*N^2+(jl-1)*N+kl;
    l3=floor((vl-1)/N^2)+1; l2=floor((vl-1)/N)-(l3-1)*N+1; l1=vl-(l2-1)*N-(l3-1)*N^2;

    %Here it is the regularization matrix:
    ind=(l-1)*DIM+k; %H(k,l)
    ind1=(l1-1)*N+k1; ind2=(l2-1)*N+k2; ind3=(l3-1)*N+k3;
    nonZero=find(Pi0(ind1)& Pi0(ind2)&Pi0(ind3));

    ind=ind(nonZero); ind1=ind1(nonZero); ind2=ind2(nonZero); ind3=ind3(nonZero);
    H(ind)=Pi2(ind1).*Pi0(ind2).*Pi0(ind3)...
        +2*Pi1(ind1).*Pi1(ind2).*Pi0(ind3)...
        +Pi0(ind1).*Pi2(ind2).*Pi0(ind3)...
        +2*Pi1(ind1).*Pi0(ind2).*Pi1(ind3)...
        +2*Pi0(ind1).*Pi1(ind2).*Pi1(ind3)...
        +Pi0(ind1).*Pi0(ind2).*Pi2(ind3);    
end

if whichVersion=='1st'
    % 1st version: very slow because of the loops::::::::::::::::::::::
    w1=waitbar(0,'The Regularization matrix is being constructed...');
    for vk=find(IND(:)>0)'
        %(k1,k2,k3)
        %k=(k3-1)*N^2+(k2-1)*N+k1;
        k=IND(vk); k3=floor((vk-1)/N^2)+1; k2=floor((vk-1)/N)-(k3-1)*N+1; k1=vk-(k2-1)*N-(k3-1)*N^2;
        if ~mod(k,100); waitbar(k/DIM,w1); end
        %for vl=find(IND(:)>0)'
        vl=find(IND(:)>0);
        %(il,jl,kl)
        %l=(il-1)*N^2+(jl-1)*N+kl;
        l=IND(vl); l3=floor((vl-1)/N^2)+1; l2=floor((vl-1)/N)-(l3-1)*N+1; l1=vl-(l2-1)*N-(l3-1)*N^2;
        %Here it is the regularization matrix:
        H(k,l)=Pi2(k1,l1).*Pi0(k2,l2).*Pi0(k3,l3)...
            +2*Pi1(k1,l1).*Pi1(k2,l2).*Pi0(k3,l3)...
            +Pi0(k1,l1).*Pi2(k2,l2).*Pi0(k3,l3)...
            +2*Pi1(k1,l1).*Pi0(k2,l2).*Pi1(k3,l3)...
            +2*Pi0(k1,l1).*Pi1(k2,l2).*Pi1(k3,l3)...
            +Pi0(k1,l1).*Pi0(k2,l2).*Pi2(k3,l3);
    end
    close(w1); 
end

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
