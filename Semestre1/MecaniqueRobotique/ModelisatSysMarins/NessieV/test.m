clc 
close all 
clear all


syms N E D phi theta psi u v w p q r 
syms Ix Iy Iz
syms F1 F2 F3 F4 F5 F6
syms d1x d2x d1y d2y d1y d3x d4x d5x  d6x
syms rho R L m v g 
syms K1 K2 K4 
syms Kq1 Kq2 Kq3 Kq5 Kq6


%Les paramètres fixes du problème

%% Vecteur d'état
X=[N,E,D,phi,theta,psi,u,v,w,p,q,r]';
X1=[N E D phi theta psi]'; %position vector in Earth frame
X2=[u v w p q r]';         %velocity vector in Body Frame
%%

%%Friction forces
Kl=[K1 0 0 0 0 0;
    0 K2 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 K4 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];

Kq=[Kq1 0 0 0 0 0;
    0 Kq2 0 0 0 0;
    0 0 Kq3 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 Kq5 0;
    0 0 0 0 0 Kq6];

Kb=-Kl*X2-Kq*norm(X2)*X2; 

%Gravity 
W=m*g; %wheight
B=rho*v*g; %Poussé d'archimède
r_g=[0 0 0]';
r_b=[0 0 -0.02]';


g_nu=(-1)*[(W-B)*sin(theta) ;
    -(W-B)*cos(theta)*sin(phi) ;
    -(W-B)*cos(theta)*cos(phi) ;
    -(r_g(2)*W-r_b(2)*B)*cos(theta)*cos(phi)+(r_g(3)*W-r_b(3)*B)*cos(theta)*sin(phi) ;
    (r_g(3)*W-r_b(3)*B)*cos(theta)*cos(phi) ;
    -(r_g(2)*W-r_b(2)*B)*cos(theta)*sin(phi)-(r_g(2)*W-r_b(2)*B)*sin(theta)];

M=[m+0.1*m 0 0 0 0 0;
    0 m+pi*rho*R^2*L 0 0 0 0;
    0 0 m+pi*rho*R^2*L 0 0 0;
    0 0 0 Ix 0 0;
    0 0 0 0 Iy+1/12*(0.4*m*R^2++pi*rho*R^2*L^3) 0;
    0 0 0 0 0 Iz+1/12*(0.4*m*R^2++pi*rho*R^2*L^3)];

%%
%%Les fonctions permettant de faire les transformations entre Body et Ned
f1(X')=u*cos(psi)*cos(theta)+v*(cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))+w*(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta)); %N'
f2(X')=u*sin(psi)*cos(theta)+v*(cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi))+w*(sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi));  %E'
f3(X')=-u*sin(theta)+v*cos(theta)*sin(phi)+w*cos(theta)*cos(phi);  %D'
f4(X')=p+q*sin(phi)*tan(theta)+r*cos(phi)*r*tan(theta);   %theta'
f5(X')=q*cos(phi)-r*sin(phi);                             %phi'
f6(X')=q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta);       %psi'
%% 
%%Les fonctions permettant de faire les transformations entre Body et Ned

%%Les fonctions permettant de faire les transformations entre Body et Ned
f7(X')=(F1+F2+g_nu(1)+Kb(1))/M(1,1)+v*r-w*q;    %u'
f8(X')=(F3+F4+g_nu(2)+Kb(2))/M(2,2)+w*p-u*r;    %v'
f9(X')=(F5+F6+g_nu(3)+Kb(3))/M(3,3)+u*p-v*p;    %w'
f10(X')=((M(5,5)-M(6,6))*q*r+g_nu(4)+Kb(4))/M(4,4);          %p'
f11(X')=1/M(5,5)*(-(M(4,4)-M(6,6))*p*r-d5x*F5+d6x*F6+g_nu(5)+Kb(5)); %q'
f12(X')=1/M(6,6)*(-(M(5,5)-M(4,4))*p*q+d2y*F1+d2y*F2+d3x*F3+d4x*F4+g_nu(6)+Kb(6));  %r'
%%
%La jacobienne de f par rapport à N E D 0 0 0 u v w p q r 

f=[f1 ; f2 ; f3 ; f4 ; f5 ; f6 ; f7 ; f8 ; f9 ; f10 ; f11; f12];
A = jacobian(f , [N,E,D,phi,theta,psi,u,v,w,p,q,r]);
B = jacobian(f ,[F1,F2,F3,F4,F5,F6]);
C=[0,0,0,0,0,0,1,0,0,0,0,0;
   0,0,0,0,0,0,0,1,0,0,0,0;
   0,0,0,0,0,0,0,0,1,0,0,0;
   0,0,0,0,0,0,0,0,0,1,0,0;
   0,0,0,0,0,0,0,0,0,0,1,0;
   0,0,0,0,0,0,0,0,0,0,0,1;
   ];

A=simplify(A(0,0,0,0,0,0,1,0,0,0,0,0));
Amat=formula(A);
[n,n]=size(Amat);
 obs=C;
 for i=1:n-1
     obs=[obs;C*Amat.^i];
 end


if rank(obs)==n
    ob=('systéme observable');
else
   ob=('systéme n''est pas observable');
end
cmb=B;

for i=1:n-1
     cmb=[cmb;Amat.^i*B];
 end

if rank(cmb)==n
    cm=('systéme commandable');
else
   cm=('systéme n''est pas cammandable');
end
%% on a les etat N E D et Psi ne sont pas obsevable 
%on ajoute un capteur de localisation pour calcule les N E D aussi un capteur pour calcule le psi suivant l'axe N