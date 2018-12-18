% DMP angle learning 


%% parameters 

% robot parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global arms dof arms_flat

% potential field parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global alpha0_max eta po beta lambda 
global obs

% DMP learning parameter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbDatal=50; nbVarl=5; alphal=1;  nbStatesl=5; Kl=500; Dl=40; 
dt=0.01;
nbDatab=200; nbVarb=3; alphab=1;  nbStatesb=12; Kb=200; Db=20; 

axs=[-15 15 -15 15 -15 15];


%% Link defined

LinkLength=[1 1 2 4 5];
dof=5;
arms=cell(dof,1); arms_flat=cell(2*dof,1);
eta=300; % need to be tuned
alpha0_max=1;
po=2;
beta=2;
lambda=20;

[arms,arms_flat]=Make_arm(LinkLength,SE3(eye(4)));

%%

q0=[0 -pi/2 0 0 0];
q_start=[1e-4 0 1e-4 pi/30 pi/30];
q_end=[1e-4 0 pi/6 pi/3 pi/4];
arms_flat{end}.plot(qin(q0+q_end),'jointdiam',0.25,'jointcolor',[0.5 0.5 1],'tilesize',5,'nobase','noname')

xl_start=arms_flat{end}.fkine(qin(q0+q_start));
xl_end=arms_flat{end}.fkine(qin(q0+q_end));
xl=mtraj(@lspb,xl_start.t',xl_end.t',nbDatal);
xl_nominal=xl;



xdotl=computeDerivative(xl',dt);
xddotl=computeDerivative(xdotl,dt);


qs=zeros(nbDatal,dof);

plot3(xl(:,1),xl(:,2),xl(:,3),'k')
hold on
for t=1:nbDatal     
    
    if t==1
        qs(t,:)=q0+q_start;
    else
        if t<=nbDatal % activating link 
            J=arms{dof}.end.jacob0(qin(prev_q));
            J=J(1:3,[1 5 7 9]);
            DMP_vel=(pinv(J,0.001)*xdotl(:,t));
            thetadot=DMP_vel;
            qs(t,:)=prev_q+[thetadot(1)';0;thetadot(2:end)]'*dt;
        else % no - activation link
            qs(t,:)=qs(t-1,:);
        end
    end
    prev_q=qs(t,:);
    arms_flat{end}.plot(qin(prev_q),'jointdiam',0.25,'jointcolor',[0.5 0.5 1],'tilesize',5,'nobase','noname',...
   'workspace',axs,'nowrist')    
    hold on

plot3(xl(:,1),xl(:,2),xl(:,3),'k')


end


%% 
[w,Mu_s,Sigma_s]=DMP_learning(nbDatal,nbVarl,alphal,dt, nbStatesl,Kl,Dl,qs');
[q,qdotl]=DMP_repro(nbDatal,nbVarl,alphal,dt,nbStatesl,Kl,Dl ,w,Mu_s,Sigma_s,1,qs(1,:)',qs(1,:)',qs(end,:)');

isRepro=0;


for t=1:nbDatal
    
    %perturbation
 if t==20
      start_step=t;
      start_pos=q(:,t)+[pi/2 0 0 0 0]';
      [q_repro,qdot_repro]=DMP_repro(nbDatal,nbVarl,alphal,dt,nbStatesl,Kl,Dl ,w,Mu_s,Sigma_s,t,start_pos,qs(1,:)',qs(end,:)');
      q(:,t+1:end)=q_repro(:,t+1:end);
      isRepro=1;        
     
 end

    
arms_flat{end}.plot(qin(q(:,t)'),'jointdiam',0.25,'jointcolor',[0.5 0.5 1],'tilesize',5,'nobase','noname',...
   'workspace',axs,'nowrist')

hold on

plot3(xl(:,1),xl(:,2),xl(:,3),'k')


end

