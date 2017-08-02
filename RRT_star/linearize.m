function varargout=linearize(x,u,varargin)
% state= [x y z roll pitch yaw  u v w p q r]
global l
f1=-sin(x(4))*sin(x(5))*cos(x(6))+cos(x(4))*sin(x(6));
f2=cos(x(4))*cos(x(5))*cos(x(6));
f3=-cos(x(4))*sin(x(5))*sin(x(6))+sin(x(4))*cos(x(6));

f4=-sin(x(4))*sin(x(5))*sin(x(6))-cos(x(4))*cos(x(6));
f5=cos(x(4))*cos(x(5))*sin(x(6));
f6=cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6));

f7=-sin(x(4))*cos(x(5));
f8=-cos(x(4))*sin(x(5));
f9=0;

f10=cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6));
f11=cos(x(4))*sin(x(5))*sin(x(6))-sin(x(4))*cos(x(6));
f12=cos(x(4))*cos(x(5));

A=[zeros(6) eye(6);[zeros(3) u(1)*[f1 f2 f3; f4 f5 f6;f7 f8 f9];zeros(3,6)] zeros(6)];
B=[zeros(6,4); [f10 f11 f12]' zeros(3) ;zeros(3,1) diag([l l 1])];
varargout{1}=A; varargout{2}=B;

% query?
if ~isempty(varargin)
        xq=varargin{1}; uq=varargin{2};
    xdot=dynamics(x,u);
    varargout{3}=xdot+A*(xq-x)+B*(uq-u);
end    
        
end

