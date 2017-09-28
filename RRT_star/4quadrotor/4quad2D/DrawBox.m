function DrawBox(T,dim)
            % for points of rect before applying T
            p1=-dim; p2=dim;
            x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0])*(p2(1)-p1(1))+p1(1);
            y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1])*(p2(2)-p1(2))+p1(2);
            z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1])*(p2(3)-p1(3))+p1(3);
            x=reshape(x,1,[]);
            y=reshape(y,1,[]);
            z=reshape(z,1,[]);
            
            pnts=[x;y;z];
            % location of pnts after applying T
            pnts=T*pnts;
            
            x=reshape(pnts(1,:),4,6);            
            y=reshape(pnts(2,:),4,6);
            z=reshape(pnts(3,:),4,6);
            % plotting using patch
           for i=1:6
                h=patch(x(:,i),y(:,i),z(:,i),'k','FaceAlpha',0.2);
                set(h,'edgecolor','w')
           end
 end
        
