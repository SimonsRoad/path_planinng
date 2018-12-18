function out=alphah(do)
global alpha0_max po

if do<po
   out=1/2*alpha0_max;
else
   out=1/2*alpha0_max*exp(-(do-po));
end


end
