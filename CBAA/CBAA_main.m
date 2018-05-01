% this is implmentation of CBBA algorithm (single task assignment to a fleet of unit)
% code was written by Jeon Bo Seong of Seoul National University of South
% Korea

%% problem definition 

Nt=4; Nu=3;

% target position (task)
g1=[2 4]; g2=[3 3 ]; g3=[4 3]; g4=[5 2]; gs=[g1;g2;g3;g4];
p1=[1 2]; p2=[2 1]; p3=[3 1]; ps=[p1;p2;p3];

% construct c_ij
C=zeros(Nu,Nt);
X=C; Y=C; H=C;

Js=zeros(3,1);


for i=1:Nu
    for j=1:Nt
        p=ps(i,:); g=gs(j,:);
        C(i,j)=1/sqrt((p(1)-g(1))^2+(p(2)-g(2))^2);
    end
end

% C=C+0.001*rand(Nu,Nt);
%% 
N_iter=20;

for t=1:N_iter

    for i=1:Nu    
        % phase 1 
        xi=X(i,:); yi=Y(i,:); ci=C(i,:);
        if sum(xi)==0 % if no task allocated for agent i  
            hi=indicator_function(ci,yi);
            if ~(sum(hi)==0) % if winning found
                [~,Ji]=max(hi.*ci);
                X(i,Ji)=1; Y(i,Ji)=C(i,Ji); % task allocated for this iteration 
                Js(i)=Ji;
            end
        end
        
        % phase 2 : consensus--winning bid yi update by seeing  yk (of others) 
        
        % update the winning bid 
        for j=1:Nt 
            Y(i,j)=max(Y(:,j));
        end
        
        [~,z]=max(C(:,Js(i)));        
        
        % if I lose .. give up 
        if z ~= i
            X(i,Js(i))=0;
        end
        
    end
end
    



