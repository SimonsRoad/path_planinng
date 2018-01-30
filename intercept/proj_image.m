function proj=proj_image(f,p,R,pt)
    xc=R'*(pt-p);
    
    proj=f/xc(1)*[xc(2); xc(3)];
   end