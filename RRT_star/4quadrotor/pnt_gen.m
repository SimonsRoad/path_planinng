function pnts=pnt_gen(x1,x2,n)
d=length(x1);
pnts=[];
for i=1:d
    pnts=[pnts;linspace(x1(i),x2(i),n)];
 
end

end