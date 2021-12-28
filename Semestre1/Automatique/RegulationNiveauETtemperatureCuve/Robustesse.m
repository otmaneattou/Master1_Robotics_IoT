%Robustesse 
%% 
%%%%%%%%%%%%%Robustesse de la commande d'état

%%
%Paramétres fixe de système
Qe_0 = 20       ;          %en l/min
THETAs_0R = 80   ;          %en °C
H_0=0.6         ;          %en m
Pu_0=20         ;          %en KW
Qs_0R=15         ;          %en l/min
THETAe=20       ;          %en °C
S=1             ;          %en m^2

%%
%%Modélisation du système


Amat_R = [ -Qe_0/(2*S*H_0)    0           ; 
         0                 -Qe_0/(S*H_0)];

Bmat_R = [1/S                       0                             ;
        (THETAs_0R-THETAe)/(S*H_0)  (THETAs_0R-THETAe)/(S*H_0*Pu_0)];

Cmat_R=eye(2);

Dmat_R=[0 0;0 0];


sys_R=ss(Amat,Bmat,Cmat,Dmat);
%%
%Analyse du système

T = 6;                             %en min (Période d'échantillonnage)

%Représentation d'état du système échantillonné 
sys_d_R         = c2d(sys_R,T);
[Ad_R,Bd_R,Cd_R,Dd_R] = ssdata(sys_d_R);

%matrice de transfert 
syms s

H_R=Cmat_R*inv(s*eye(2)-Amat_R)*Bmat_R;

%%commande par retour d'état

tau1 = 16 ; %en min
tau2 = 15 ; %en min

L_R = place (Ad_R,Bd_R,poles);

Ac_R   = (Ad_R - Bd_R*L);
sys1_R = ss(Ac_R,Bd_R,Cd_R,Dd_R);
t = [0:0.1:100];
u = [0.05*ones(size(t));...
    1*ones(size(t))];
figure(1);
lsim (sys1_R,u,t);
%% matrice de gain K
K = inv(-Cd*inv(Ad-Bd*L)*Bd);
v =K*u;
figure(2);
lsim (sys1_R,v,t);