%% parameters 

% robot parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global arms dof arms_flat

% potential field parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global alpha0_max eta po beta lambda 
dof=5;

% DMP learning parameter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbDatal=200; nbVarl=dof; alphal=1;  nbStatesl=10; Kl=500; Dl=40; 
dt=0.01;
nbDatab=200; nbVarb=3; alphab=1;  nbStatesb=12; Kb=200; Db=20; 

% simulink parameters  
% 200 step=> 20s
global t_scale
t_scale=20/200;




%% Link defined

LinkLength=[1 1 4 4 5];

arms=cell(dof,1); arms_flat=cell(2*dof,1);
eta=800; % need to be tuned
alpha0_max=3;
po=3;
beta=3;
lambda=40;

[arms,arms_flat]=Make_arm(LinkLength,SE3(eye(4)));
[arms0,arms_flat0]=Make_arm(LinkLength,SE3(eye(4)));
%% Body path & obstacle defined 
% straiht line 

xb(1,:)=linspace(25,-25,nbDatab);
xb(2,:)=zeros(1,nbDatab);
xb(3,:)=5*ones(1,nbDatab);

% C -curve

% th=linspace(-pi,0,nbDatab);
% for i=1:nbDatab
% xb(1,i)=30*cos(th(i));
% xb(2,i)=30*sin(th(i));
% end
% 
% xb(3,:)=5*ones(1,nbDatab);
% 
% % ¿Ï¸¸ÇÑ °î¼± 
% xb(2,:)=xb(2,:)/3;

%% obstacle generation 
global obs1_path 

%straight line - moving case 
waypoint1=xb(:,nbDatab/2)'+[0 0 -3];

obs1_path=[repmat(waypoint1(1),200,1)  [linspace(-10,0,125)  linspace(0.03,10,75)]'  repmat(waypoint1(1),200,1)]; 

%C curve - static case 
% waypoint1=xb(:,nbDatab/2)'+[0 1 -3];
% obs1_path=repmat(waypoint1,200,1);


%%


xb0=xb(:,1);
xbg=xb(:,end);

[wb,Mub,Sigmab]=DMP_learning(nbDatab,nbVarb,alphab,dt,nbStatesb,Kb,Db,xb);
[xb_repro,xdotb_repro]=DMP_repro(nbDatab,3,alphab,dt,nbStatesb,Kb,Db,wb,Mub,Sigmab,1,xb0,xb0,xbg,0);


global yaw
% yaw calculation 
yaw=zeros(1,nbDatab);
yaw(1:end-1)=atan2(xdotb_repro(2,2:end),xdotb_repro(1,2:end)); yaw(end)=yaw(end-1);
yaw(yaw<0)=yaw(yaw<0)+2*pi;
for check=1:nbDatab-1
    if abs(yaw(check+1)-yaw(check))>pi
        yaw(check+1)=yaw(check+1)+2*pi;
    end
end
yaw=yaw-pi;


for i=1:nbDatab
Tb(:,:,i)=[rotz(yaw(i)) xb_repro(:,i); zeros(1,3) 1];
end

figure()
title('body trajectory')
axs=[-50 50 -50 50 -50 50];


for t=1:nbDatab
obs1=obs1_path(t,:)';
axis(axs)
hold on
plot3(xb_repro(1,:),xb_repro(2,:),xb_repro(3,:))
plot3(xb0(1),xb0(2),xb0(3),'ko')
plot3(xbg(1),xbg(2),xbg(3),'ro')
grid on
draw_sphere(obs1,po/2)
hold off
end




%% Link path defined and learned in joint space
global q_DMP qdot_DMP xl_repro xdotl_repro q_init q_fin
global wq Muq Sigmaq


q0=[0 -pi/2 0 0 0];
q_start=[1e-4 0 1e-4 pi/30 pi/30];
q_end=[1e-4 0 1e-4 pi/4 pi/3];

% still arms_flat is SE3(eye(4))
xl_start=arms_flat0{end}.fkine(qin(q0+q_start));
xl_end=arms_flat0{end}.fkine(qin(q0+q_end));
xl=mtraj(@lspb,xl_start.t',xl_end.t',nbDatal);
xl_nominal=xl;
xdotl=computeDerivative(xl',dt);
xddotl=computeDerivative(xdotl,dt);

[wl,Mul,Sigmal]=DMP_learning(nbDatal,3,alphal,dt, nbStatesl,Kl,Dl,xl');
[xl_repro,xdotl_repro]=DMP_repro(nbDatal,3,alphal,dt,nbStatesl,Kl,Dl ,wl,Mul,Sigmal,1,xl_start.t,xl_start.t,xl_end.t,0);

% link motion animation-did with not special reason

qs=zeros(nbDatal,dof);
for t=1:nbDatal     
    
    if t==1
        qs(t,:)=q0+q_start;
    else
        if t<=nbDatal % activating link 
            J=arms{dof}.end.jacob0(qin(prev_q));
            J=J(1:3,[1 5 7 9]);
            DMP_vel=(pinv(J,0.0001)*xdotl(:,t));
            thetadot=DMP_vel;
            qs(t,:)=prev_q+[thetadot(1)';0;thetadot(2:end)]'*dt;
        else % no - activation link
            qs(t,:)=qs(t-1,:);
        end
    end
    prev_q=qs(t,:);
end


q_init=qs(1,:);
q_fin=qs(end,:);



[wq,Muq,Sigmaq]=DMP_learning(nbDatal,nbVarl,alphal,dt, nbStatesl,Kl,Dl,qs');

[q_DMP,qdot_DMP]=DMP_repro(nbDatal,nbVarl,alphal,dt,nbStatesl,Kl,Dl ,wq,Muq,Sigmaq,1,qs(1,:)',qs(1,:)',qs(end,:)',0);

%%
figure()
title('link motion')
axis([-10 10 -10 10 -10 10])
plot3(xl_repro(1,:),xl_repro(2,:),xl_repro(3,:),'k')
hold on
arms_flat0{end}.plot(qin(qs),'notiles','nojoints','nobase','noname','nowrist','workspace',[-10 10 -10 10 -10 10])
%%
sim_with_no_dynamics

%% slimulink here 
% data export
global xd yd zd 
xd=[ (1:200)' xb_repro(1,:)'];
yd=[(1:200)' xb_repro(2,:)' ];
zd=[ (1:200)' xb_repro(3,:)'];

% initializing 
global iscol1 isrepro1 isaway1
iscol1=0; isrepro1=0; isaway1=0;
% original qdot 
[wq,Muq,Sigmaq]=DMP_learning(nbDatal,nbVarl,alphal,dt, nbStatesl,Kl,Dl,qs');
[q_DMP,qdot_DMP]=DMP_repro(nbDatal,nbVarl,alphal,dt,nbStatesl,Kl,Dl ,wq,Muq,Sigmaq,1,qs(1,:)',qs(1,:)',qs(end,:)',0);

run('simulink_pd.mdl')


%% data import from simulink 

rpy=interp1(output1(:,end)/t_scale,output1(:,4:6),1:nbDatab);
xyz=interp1(output1(:,end)/t_scale,output1(:,1:3),1:nbDatab);
q=interp1(output1(:,end)/t_scale,output1(:,13:17),1:nbDatab);

x0s=interp1(output1(:,end)/t_scale,x0_x0dot.Data(:,1:3),1:nbDatab);
x0dots=interp1(output1(:,end)/t_scale,x0_x0dot.Data(:,4:6),1:nbDatab);



%%
figure()
t_last=140;
pot_length=5;
axs=[-50 50 -50 50 -25 25]*0.55;
for t=1:nbDatab
%     plot3(xb_repro(1,:),xb_repro(2,:),xb_repro(3,:))
    plot3(xyz(1:t,1),xyz(1:t,2),xyz(1:t,3),'k')

    
    obs1=obs1_path(t,:)';
    R=double(SE3(eye(3),xyz(t,:)))*rpy2tr(rpy(t,1),rpy(t,2),rpy(t,3));
    hold on
%     draw_drone(R,4,2,3)
       
    x0=x0s(t,:);
    x0dot=x0dots(t,:);
    
    
    
    if t>90 && t<t_last
        quiver3(x0(1),x0(2),x0(3),x0dot(1)/norm(x0dot)*pot_length,...
            x0dot(2)/norm(x0dot)*pot_length,x0dot(3)/norm(x0dot)*pot_length,'r',...
            'LineWidth',3,'MaxHeadSize',3);
        plot3(x0s(90:t,1),x0s(90:t,2),x0s(90:t,3),'b*')
    elseif t>=t_last
         plot3(x0s(90:t_last,1),x0s(90:t_last,2),x0s(90:t_last,3),'b*')
    end
    
    
    

    axis(axs)

    
     
    draw_sphere(obs1,po/1.6)

    
    
% 
%     [arms,arms_flat]=Make_arm(LinkLength,R);

%     
%     arms_flat{end}.plot(qin([0 q(t,2:5)]),'nojoints','notiles','nobase','noname',...
%    'workspace',axs,'nowrist','fps',100,'linkcolor','k','view',[110 40],'workspace',axs)    
    


    hold off

end






%%





%%

% 
% 
%             % trajectory reproduction 
%             
%             if do>po+1
%                 qdot_null=zeros(dof-1,1); 
%             else
%                start_step=t;
%                start_pos=arms{dof}.end.fkine(qin(prev_q)).t-xb(:,t);
%                [x_repro,xdot_repro]=DMP_repro(nbDatal,3,alphal,dt,nbStatesl,Kl,Dl,wl,Mul,Sigmal,start_step,start_pos,xl_start.t,xl_end.t);
%                xdotl(:,t+1:end)=xdot_repro(:,t+1:end);
%                isRepro=1;        
%             end
%        
        



















