

%%%R�gulation de niveau et de temp�rature dans une cuve 

%%
%Param�tres fixe de syst�me
Qe_0 = 20       ;          %en l/min
THETAs_0 = 50   ;          %en �C
H_0=0.6         ;          %en m
Pu_0=20         ;          %en KW
Qs_0=20         ;          %en l/min
THETAe=20       ;          %en �C
S=1             ;          %en m^2


%%
%%Mod�lisation du syst�me


Amat = [ -Qe_0/(2*S*H_0)    0           ; 
         0                 -Qe_0/(S*H_0)];

Bmat = [1/S                       0                             ;
        (THETAs_0-THETAe)/(S*H_0)  (THETAs_0-THETAe)/(S*H_0*Pu_0)];

Cmat=eye(2);

Dmat=[0 0;0 0];


sys=ss(Amat,Bmat,Cmat,Dmat);
%%
%Analyse du syst�me

T = 6;                             %en min (P�riode d'�chantillonnage)

%Repr�sentation d'�tat du syst�me �chantillonn� 
sys_d         = c2d(sys,T);
[Ad,Bd,Cd,Dd] = ssdata(sys_d);

%Stabilit� au sens de Lyapunov
Xlyap               = lyap(Ad,Bd,Cd);                %Solve Lyapunov Equation         
valeurspropresXlyap = eig(Xlyap); 

%La commandabilit� et l'observabilit� du syst�me en utilisant le calcul du
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

H=Cmat*inv(s*eye(2)-Amat)*Bmat;

%%
%commande par retour d'�tat

tau1 = 16 ; %en min
tau2 = 15 ; %en min

%les poles du systeme en boucles ferm�e sont 
pole1=-1/tau1;
pole2=-1/tau2;

poles=[pole1 pole2];
L = place (Ad,Bd,poles);

% reponse du systeme � hc=50mm et thetaC

Ac   = (Ad - Bd*L);
sys1 = ss(Ac,Bd,Cd,Dd);
y    = step(sys1,[0.050 1]);



