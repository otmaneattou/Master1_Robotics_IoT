

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

% fonction de transfert 
sys = ss(Amat,Bmat,Cmat,Dmat);
TF = tf(sys);
%evolution des vecteur d'etats
