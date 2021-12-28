clc 
close all 
clear all


syms N E D phi theta psi u v w p q r 
syms Ix Iy Iz
syms F1 F2 F3 F4 F5 F6
syms d1x d2x d1y d2y d1y d3x d4x d5x  d6x
syms rho R L m V g 
syms K1 K2 K4 
syms Kq1 Kq2 Kq3 Kq5 Kq6


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

%%
%Gravité et flottabilité 
W=m*g; %wheight
B=rho*v*g; %Poussé d'archimède

R=[  cos(psi)*cos(theta)    -sin(psi)*cos(phi)+cos(psi)*sin(phi)*sin(theta)    sin(psi)*sin(theta)+cos(psi)*cos(phi)*sin(theta);
     sin(psi)*cos(theta)  cos(psi)*cos(phi)+sin(psi)*sin(phi)*sin(theta)     -cos(psi)*sin(theta)+sin(psi)*cos(phi)*sin(theta);
     -sin(theta)  cos(theta)*sin(phi) cos(theta)*cos(phi)];
 
 F_g=[ 0 0 m*g]';  %Axe z est dirigé vers le bas 
 F_b=[0 0 -rho*V*g]';
 
 r_g=[0 0 0]';
 r_b=[0 0 -0.02]';
 
 Gb=[inv(R)*F_g;
     cross(r_g,inv(R)*F_g)]; 
 
 Gb=[inv(R)*F_b;
     cross(r_b,inv(R)*F_b)]; 
 
%% 
%Effets d'actionneurs 
syms d1x d1y d1z
syms d2x d2y d2z
syms d3x d3y d3z
syms d4x d4y d4z
syms d5x d5y d5z
syms d6x d6y d6z

r1=[d1x d1y d1z]';
r2=[d2x d2y d2z]';
r3=[d3x d3y d3z]';
r4=[d4x d4y d4z]';
r5=[d5x d5y d5z]';
r6=[d6x d6y d6z]';



v1=[F1 0 0]';
v2=[F2 0 0]';

v3=[0 F3 0]';
v4=[0 F4 0]';

v5=[0 0 F5]';
v6=[0 0 F6]';

M11=cross(r1,v1);
M22=cross(r2,v2);
M33=cross(r3,v3);
M44=cross(r4,v4);
M55=cross(r5,v5);
M66=cross(r6,v6);

U_b=[1 1 0 0 0 0;
    0 0 1 1 0 0;
    0 0 0 0 1 1;
    M11(1) M22(1) M33(1) M44(1) M55(1) M66(1);
    M11(2) M22(2) M33(2) M44(2) M55(2) M66(2);
    M11(3) M22(3) M33(3) M44(3) M55(3) M66(3)];

%%
%Masse ajoutée 

Masse = [m+0.1*m 0 0 0 0 0;
         0 m+pi*rho*R^2*L 0 0 0 0;
         0 0 m+pi*rho*R^2*L 0 0 0;
         0 0 0 Ix 0 0;
         0 0 0 0 Iy+1/12*(0.4*m*R^2++pi*rho*R^2*L^3) 0;
         0 0 0 0 0 Iz+1/12*(0.4*m*R^2++pi*rho*R^2*L^3)];


%%
%Coriolis 

%%
%%Les fonctions permettant de faire les transformations entre Body et Ned
f1(X')=u*cos(psi)*cos(theta)+v*(cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))+w*(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta)); %N'
f2(X')=u*sin(psi)*cos(theta)+v*(cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi))+w*(sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi));  %E'
f3(X')=-u*sin(theta)+v*cos(theta)*sin(phi)+w*cos(theta)*cos(phi);  %D'
f4(X')=p+q*sin(phi)*tan(theta)+r*cos(phi)*r*tan(theta);   %theta'
f5(X')=q*cos(phi)-r*sin(phi);                             %phi'
f6(X')=q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta);       %psi'




