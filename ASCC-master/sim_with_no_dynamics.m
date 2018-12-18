% flight test 
%% Flight Simulation 

% initialization 
[xl_repro,xdotl_repro]=DMP_repro(nbDatal,3,alphal,dt,nbStatesl,Kl,Dl ,wl,Mul,Sigmal,1,xl_start.t,xl_start.t,xl_end.t,0);
[q_DMP,qdot_DMP]=DMP_repro(nbDatal,nbVarl,alphal,dt,nbStatesl,Kl,Dl ,wq,Muq,Sigmaq,1,qs(1,:)',qs(1,:)',qs(end,:)',0);

q_sim=zeros(nbDatab,dof);

q_null1=zeros(nbDatab,dof);
x0s=zeros(3,nbDatab);
x0dots=zeros(3,nbDatab);
figure()

isaway1=0;
iscol1=0;
isrepro1=0;

for t=1:nbDatab
    
    obs1=obs1_path(t,:)';
    
    [arms,arms_flat]=Make_arm(LinkLength,Tb(:,:,t));
    
    if t==1
        q_sim(t,:)=q0+q_start;
    else
            
        J=arms{dof}.end.jacob0(qin(prev_q));
        xe=arms{dof}.end.fkine(qin(prev_q));
        xe=xe.t;
        J=J(1:3,[1 5 7 9]);
        [x01,J01,x0dot1,do1,nrst]=MK5(obs1,prev_q);
        J01=J01(:,[1 3 4 5]);
        
            if do1<po
                iscol1=1;
            end
                 
            if do1<po/3
                 disp('colision-1')
            end

                                   
        qdot_null1=alphah(do1)*pinv(J01*(eye(dof-1)-pinv(J)*J),0.0001)*(alphao(do1)*x0dot1-J01*pinv(J)*xdotl_repro(:,t));
           
        
        if ~iscol1
        
        qdot=qdot_DMP(:,t);    
        
        
        elseif iscol1 && ~isrepro1
            if norm(pinv(J01*(eye(dof-1)-pinv(J)*J),0.0001))<1e-6
                qdot=pinv(J,0.001)*x0dot1;
            else
                qdot=pinv(J,0.001)*xdotl_repro(:,t)+qdot_null1;
            end
                
            
                qdot=[qdot(1) ;0 ;qdot(2:end)];
        
        else 
        qdot=qdot_DMP(:,t);    
        
        end
        
        isaway1=do1>po+0.5 && iscol1;
                
        q_sim(t,:)=prev_q+qdot'*dt; 
       
       
       
       
        if isaway1 && ~isrepro1
           start_step=t;
           start_pos=q_sim(t,:)';
           [q_repro,qdot_repro]=DMP_repro(nbDatal,nbVarl,alphal,dt,nbStatesl,Kl,Dl ,wq,Muq,Sigmaq,start_step,start_pos,qs(1,:)',qs(end,:)',0);
           qdot_DMP(:,t+1:end)=qdot_repro(:,t+1:end);
           disp('----------------repro1-----------------------------')
           isrepro1=1;
        end
        
              
    end
    prev_q=q_sim(t,:);


    arms_flat{end}.plot(qin(q_sim(t,:)),'nojoints','tilesize',10,'nobase','noname',...
   'workspace',axs,'nowrist','fps',1000,'linkcolor','k')    
    hold on
    



    plot3(xb_repro(1,:),xb_repro(2,:),xb_repro(3,:))
    plot3(xb0(1),xb0(2),xb0(3),'ko')
    plot3(xbg(1),xbg(2),xbg(3),'ro')
   
    

    
    if t~=1
    quiver3(x01(1),x01(2),x01(3),x0dot1(1),x0dot1(2),x0dot1(3),'b')
    end

    draw_sphere(obs1,po)
end

%% this is solution 

xyz_yaw=[xb_repro ; yaw+q_sim(:,1)'];

figure
for i=1:4
subplot(4,1,i)

plot(xyz_yaw(i,:))
end

figure
for i=1:3

subplot(3,1,i)
plot(q_sim(:,i+2))
end

%%
figure()

for t=1:nbDatab
    plot3(xb_repro(1,:),xb_repro(2,:),xb_repro(3,:))
    hold on
    obs1=obs1_path(t,:)';
%     
%     R=double(SE3(eye(3),xyz(t,:)))*rpy2tr(rpy(t,1),rpy(t,2),rpy(t,3));
%     
    draw_drone(SE3(rotz(xyz_yaw(end,t)),xyz_yaw(1:3,t)),4,2,3)
    

%     x0=x0s(:,t);
%     x0dot=x0dots(:,t);
    axis([-50 50 -50 50 -25 25])

%     quiver3(x0(1),x0(2),x0(3),x0dot(1),x0dot(2),x0dot(3),'b')
%     
    draw_sphere(obs1,po/2)


    [arms,arms_flat]=Make_arm(LinkLength,Tb(:,:,t));
    

    arms_flat{end}.plot(qin(q_sim(t,:)),'nojoints','tilesize',10,'nobase','noname',...
   'workspace',axs,'nowrist','fps',100,'linkcolor','k','view',[-70 40],'workspace',[-50 50 -50 50 -25 25])    
    


    hold off
end



