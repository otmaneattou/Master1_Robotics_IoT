

%%%Régulation de niveau et de température dans une cuve 

%%
%Paramétres fixe de système
Qe_0 = 20       ;          %en l/min
THETAs_0 = 50   ;          %en °C
H_0=0.6         ;          %en m
Pu_0=20         ;          %en KW
Qs_0=20         ;          %en l/min
THETAe=20       ;          %en °C
S=1             ;          %en m^2


%%
%%Modélisation du système


Amat = [ -Qe_0/(2*S*H_0)    0           ; 
         0                 -Qe_0/(S*H_0)];

Bmat = [1/S                       0                             ;
        (THETAs_0-THETAe)/(S*H_0)  (THETAs_0-THETAe)/(S*H_0*Pu_0)];

Cmat=eye(2);

Dmat=[0 0;0 0];


sys=ss(Amat,Bmat,Cmat,Dmat);
%%
%Analyse du système

T = 6;                             %en min (Période d'échantillonnage)

%Représentation d'état du système échantillonné 
sys_d         = c2d(sys,T);
[Ad,Bd,Cd,Dd] = ssdata(sys_d);

%Stabilité au sens de Lyapunov
Xlyap               = lyap(Ad,Bd,Cd);                %Solve Lyapunov Equation         
valeurspropresXlyap = eig(Xlyap); 

%La commandabilité et l'observabilité du système en utilisant le calcul du
%grammien 
Wc = gram(sys_d,'c');
Wo = gram(sys_d,'o');

valeurspropresWc = eig(Wc) ; 
valeurspropresWo = eig(Wo) ;

% 
% %Matrice de transert 
 states = {'h' 'theta' };
 inputs = {'qe' 'pu'};
outputs = {'h' 'theta'};
% 


sys = ss(Amat,Bmat,Cmat,Dmat,'statename',states,...
         'inputname',inputs,...
         'outputname',outputs);

TF = tf(sys);

%%commande par retour d'état

tau1 = 16 ; %en min
tau2 = 15 ; %en min

%les poles du systeme en boucles fermée sont 
pole1=-1/tau1;
pole2=-1/tau2;

poles=[pole1 pole2];
L = place (Ad,Bd,poles);

% reponse du systeme à hc=50mm et thetaC

Ac   = (Ad - Bd*L);
sys1 = ss(Ac,Bd,Cd,Dd,'statename',states,...
         'inputname',inputs,...
         'outputname',outputs);
t = [0:0.1:100];
u = [0.05*ones(size(t));...
    1*ones(size(t))];
figure(1);
lsim (sys1,u,t);
%% matrice de gain K
K = inv(-Cd*inv(Ad-Bd*L)*Bd);
v =K*u;
figure(2);
lsim (sys1,v,t);
%% %Robustesse 
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
sys1_R = ss(Ac_R,Bd_R,Cd_R,Dd_R,'statename',states,...
         'inputname',inputs,...
         'outputname',outputs);
t = [0:0.1:100];
u = [0.05*ones(size(t));...
    1*ones(size(t))];
figure(3);
lsim (sys1_R,u,t);
%% matrice de gain K
K_R = inv(-Cd_R*inv(Ad_R-Bd_R*L_R)*Bd_R);
v =K_R*u;
figure(4);
lsim (sys1_R,v,t);


%% commande modale
tfqetoh = TF(1,1);
tfqetotheta = TF(2,1);
tfputoh = TF(1,2);
tfputotheta = TF(2,2);
%% enlever la fonction de transfert theta = f(qe)
TF2 = [tfqetoh tfputoh;0 tfputotheta];
%% convertir en modele d'etat
sysmod = ss(TF2);
%% reprendre les calcules
sysmod_d = c2d(sysmod,T);
[Amod_d,Bmod_d,Cmod_d,Dmod_d] = ssdata(sysmod_d);

Lmod = place(Amod_d,Bmod_d,poles);
Acmod   = (Amod_d - Bmod_d*Lmod);
sysmodBF = ss(Acmod,Bmod_d,Cmod_d,Dmod_d,'statename',states,...
         'inputname',inputs,...
         'outputname',outputs);
figure(5);
lsim (sysmodBF,u,t);

Kmod = inv(-Cmod_d*inv(Amod_d-Bmod_d*Lmod)*Bmod_d);

v= Kmod*u;
figure(6);
lsim (sysmodBF,v,t);