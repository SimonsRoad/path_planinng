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

This code present a surface reconstruction technique based on implicit B-Splines (IBSs).
This Matlab code can be simply executed by calling the DEMO.m file provided in this folder.
For feeding the code, one may need a 3D mesh (points+triangulations); however, some examples
are already provided in the folder: 
(Armadillo, Bunny, Cow, Duck, Hand, Homer, Kitten, Knot and Mannequin). 
The mothod use an algebraic approach that is quite fast and contributes local geometry.
The smoothness of the surface can be easily controlled through the regularization parameter L
in function IBSL3_3DTRI.m: the higher L results in a smoother surface and vice versa.
The regularization matrices are provided for size N=20, 30; for other sizes the RegMatrix3D.m
will be automatically called to reconstruct the matrix anatically.
Please refer to the abovementioned reference and find the extended works in "Blended IBS" page. 
 
For running the code of IBS and Truncated IBS please call DEMO.m
