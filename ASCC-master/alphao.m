function out=alphao(do)
global alpha0_max po

if do<0.1*po
out=alpha0_max; 
else
out=alpha0_max*(po/20)/do;
end

end
