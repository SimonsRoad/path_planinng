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
%% ==IBS Reconstruction=================================================
%1. Load the data points:
load('88.mat'); %load('Duck.mat'); %or any other data set
%Note: the data points must be normalized (in unit cube).

%2. Call the 3L algorithm to compute IBS control parameters:
Truncated=input('Enter "1" Truncated IBS (Faster)/ "0" for normal IBS (0/1?) ');
L=10; %regularization parameter; increase it for a coarser surface.
if Truncated
    Size=100; P=IBSL3_3D_T(.01,Size,L,S, 0,N); %IBS size can be increased to 100.
else
    Size=30; P=IBSL3_3D(.01,Size,L,S, 0,N); %IBS size can be increased to 30.
end

%3. Represent the IBS surface (zero leve set): 
IBSLevelSurf(P,S,100); %the visulaization step can be decreased;
%if the surface is not completly reconstruced, change "box" parameters.
