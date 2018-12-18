function out=link_dynamics(in)
% input = xyz / rpy / q_cur / obs / qdot_DMP / xdotl_repro
% output = qdot / x0 / x0dot 

% robot parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global arms dof arms_flat
LinkLength=[1 1 4 4 5];
dof=5;

% potential field parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global  po

% boolean
global iscol1 isrepro1 isaway1 
global qdot_DMP q_init q_fin wq Muq Sigmaq
global yaw
nbDatal=200; nbVarl=dof; alphal=1;  nbStatesl=10; Kl=500; Dl=40; 
dt=0.01;




xyz=in(1:3);
rpy=in(4:6);
obs=in(7:9);
obs=reshape(obs,3,1);

cur_q=in(10:14);
cur_q=reshape(cur_q,1,length(cur_q));



qdot_DMP_local=in(15:19);
xdotl_repro=in(20:22)';
xdotl_repro=reshape(xdotl_repro,3,1);
t_cur=in(end);






% cur_q(1)=0;

R=double(SE3(eye(3),xyz))*rpy2tr(rpy(1),rpy(2),interp1((0:200)'/10,[yaw(:,1) yaw]',t_cur));

[arms,arms_flat]=Make_arm(LinkLength,R);



J=arms{dof}.end.jacob0(qin(cur_q));
J=J(1:3,[1 5 7 9]);

q2=qin(cur_q);

[~,T_all]=arms{dof}.end.fkine(q2);
xs=zeros(3,length(T_all));
nbCand=length(T_all);

for i=1:nbCand
xs(:,i)=T_all(i).t;
end

% calculating relative distance
ds_tmp=repmat(obs,1,nbCand)-xs(1:3,:);
for i=1:nbCand
ds(i)=norm(ds_tmp(:,i),2);
end

% truncate points attached to end effector link
% ds=ds(1:end-3);
[do1,nrst]=min(ds);

D=3;

% finding Jacobian 
J0=arms_flat{nrst}.jacob0(q2(1:nrst));
J0=J0(1:D,1:2:nrst); 
J01=[J0  zeros(D,dof-size(J0,2))];


% potential field 
x0=xs(:,nrst);
x0dot1=Calc_Psi_obsinput(obs,x0);


J01=J01(:,[1 3 4 5]);     
    % colision detected
    if do1<po
        iscol1=1;
    end
    % this is real colision. safe limit is violated
    if do1<po/3
         disp('colision-1')
    end           
    
    if ~iscol1
%         disp('following plain qdot_DMP')
        qdot=qdot_DMP_local;    
        
    elseif iscol1 && ~isrepro1
        disp('following nullspace projection') 
        % if nearest point is end effector=>
        if norm(pinv(J01*(eye(dof-1)-pinv(J)*J),0.0001))<1e-6
            qdot=pinv(J,0.001)*x0dot1;
        else
            mask=eye(dof-1);
            mask(1)=0.5;
            qdot_null=mask*alphah(do1)*pinv(J01*(eye(dof-1)-pinv(J)*J),0.0001)*(alphao(do1)*x0dot1-J01*pinv(J)*xdotl_repro);
            qdot=pinv(J,0.001)*xdotl_repro+qdot_null;
        end
        qdot=[qdot(1) ;0 ;qdot(2:end)];

    else 
%         disp('following reproduced trajectory') 
        qdot=qdot_DMP_local;    
    end
    
     isaway1=do1>po+0.5 && iscol1;
    
     
     
     

    if isaway1 && ~isrepro1
        
       start_step=floor(t_cur*10);
       start_pos=cur_q';
       
       [~,qdot_repro]=DMP_repro(nbDatal,nbVarl,alphal,dt,nbStatesl,Kl,Dl ,wq,Muq,Sigmaq,start_step,start_pos,q_init',q_fin',0);
       qdot_DMP(:,start_step+1:end)=qdot_repro(:,start_step+1:end);
       disp('----------------repro1-----------------------------')
       isrepro1=1;
    end



    
    
    
    
    
out=[reshape(qdot,1,length(qdot)) reshape(x0,1,3) reshape(x0dot1,1,3)];
end





    