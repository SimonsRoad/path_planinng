%% Prob2
%% trial 1
a =0.01;

P0_N = [1]; 
P0_D = [1 0 21 0 84 0 64];

P1_N = [-1 1];
P1_D = [1 1];

P2_N = [sqrt(2)*a]; 
P2_D = [1 a];

P10_N = [-1 1];
P10_D = [1 1 21 21 84 84 64 64];
O = [0];

P_N = {O,P2_N,-P0_N ;
            O, O,[1];
            O, P2_N, O;
            [0.1], O,P10_N};
        
P_D = {1,P2_D,P0_D;
            1,1,1;
            1,P2_D,1;
            1,1,P10_D};        

P_sys= tf(P_N,P_D);

[K,CL,GAM,INFO]=h2syn(P_sys,2,1);



%% trial 2
% [w u] = [f1 f2 f3 v]
% [z y] = [e v r g]
a= 0.1;




P0_N = [1]; 
P0_D = [1 0 21 0 84 0 64];

P1_N = [-1 1];
P1_D = [1 1];

P2_N = [sqrt(2)*a]; 
P2_D = [1 a];

P10_N = [-1 1];
P10_D = [1 1 21 21 84 84 64 64];


P_N = {0,P2_N,-P0_N ;
            0, P2_N,0;
            0.1, 0, P10_N};
            
P_D = {1,P2_D,P0_D;
            1,P2_D,1;
            1,1,P10_D};
            
P_sys= tf(P_N,P_D);

[K,CL,GAM,INFO]=h2syn(P_sys,2,1)



