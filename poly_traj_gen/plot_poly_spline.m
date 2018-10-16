% This function plot the path generated from piecewise polynomial 

function plot_poly_spline(Ts,Xs,Ys,Zs,pxs,pys,pzs)
    n_seg=length(Xs)-1;
    n_poly=length(pxs)/n_seg-1;
    N=5;
    xs=[]; ys=[]; zs=[];
    ts=[];
    % per segement 
    for n=1:n_seg
        t1=Ts(n); 
        t2=Ts(n+1);
                
        px=pxs((n_poly+1)*(n-1)+1:(n_poly+1)*n);
        py=pys((n_poly+1)*(n-1)+1:(n_poly+1)*n);
        pz=pzs((n_poly+1)*(n-1)+1:(n_poly+1)*n);
   
        
        for t = linspace(t1,t2,N)
            t_eval=t-t1;
            xs=[xs polyval(flipud(px),t_eval)];
            ys=[ys polyval(flipud(py),t_eval)];
            zs=[zs polyval(flipud(pz),t_eval)];
            ts=[ts t];
        end
       
    end

    figure()
    % 3d path plot
    subplot(3,2,[1,3,5])
    plot3(Xs,Ys,Zs,'rs');
    hold on 
    plot3(xs,ys,zs);
    
    subplot(3,2,2)
    plot(ts,xs)
    
    subplot(3,2,4)
    plot(ts,ys)
    
    subplot(3,2,6)
    plot(ts,zs)
    
    
    


end