% following trajectory 
figure
isRepro=0;
for t=1:nbData
   obs=obs_path(t,:);
    
   if t==1
        qs(t,:)=q0+q_start;
   else
       J=arms{dof}.end.jacob0(qin(prev_q));
       J=J(1:3,[1 5 7 9]);
       
       [x0,J0,x0dot,do]=MK2(prev_q);
       
       J0=J0(:,[1 3 4 5]);
       
       if do<po/2
          disp('!!!!!!!!!! colision !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
       end              
       null_vel=alphah(do)*pinv(J0*(eye(dof-1)-pinv(J)*J*0.98),0.01)*(alphao(do)*x0dot-J0*pinv(J)*xdots(t,:)');
       
       if do>po+1
          null_vel=zeros(dof-1,1); 
       else
           start_step=t;
           start_pos=arms{dof}.end.fkine(qin(prev_q)).t;
           [x_repro,xdot_repro]=DMP_repro(nbData,nbVar,alpha,dt,nbStates,K,D,w,Mu_s,Sigma_s,start_step,start_pos,x0_repro,xg);
           
           
           xdots(t+1:end,:)=xdot_repro(:,t+1:end)';
           isRepro=1;        
      end
       
       DMP_vel=(pinv(J,0.001)*xdots(t,:)');
       
       thetadot=DMP_vel+null_vel;
       
       fprintf('nominal angular velocity/   nullspace veclocity\n')
       for i=1:dof-1
       fprintf('%4.4f          %4.4f\n',DMP_vel(i),null_vel(i)) 
       end
                 
%        thetadot(2)=0;     

       qs(t,:)=prev_q+[thetadot(1)';0;thetadot(2:end)]'*dt;
   end

   
   prev_q=qs(t,:);
   axis([-15 15 -15 15 -15 15])
   arms_flat{end}.plot(qin(prev_q),'jointdiam',0.25,'jointcolor',[0.5 0.5 1],'tilesize',5,'nobase','noname',...
       'workspace',[-15 15 -15 15 -15 15],'nowrist','perspective')

   if t==nbData
    hold on
   else
       hold off
   end
  
   hSurface = surf(x+obs(1),y+obs(2),z+obs(3));
   set(hSurface,'FaceColor',[1 0 0],'FaceAlpha',0.5);
   hold on 
   
   
   if isRepro==1
   quiver3(x0(1),x0(2),x0(3),100*x0dot(1),100*x0dot(2),100*x0dot(3),'b','LineWidth',4)
   plot3(x_repro(1,t:end),x_repro(2,t:end),x_repro(3,t:end),'r')
   
   end
   
   plot3(xs_nominal(:,1),xs_nominal(:,2),xs_nominal(:,3),'k-')
   plot3(xg(1),xg(2),xg(3),'r*')
   
%    
%    subplot(1,3,2)
%    arms_flat{end}.plot(qin(prev_q),'jointdiam',0.25,'jointcolor',[0.5 0.5 1],'tilesize',5,'nobase','noname','view','x')
%    subplot(1,3,3)
%    arms_flat{end}.plot(qin(prev_q),'jointdiam',0.25,'jointcolor',[0.5 0.5 1],'tilesize',5,'nobase','noname','view','y')

 
   
end


hold on
plot3(xs(:,1),xs(:,2),xs(:,3))