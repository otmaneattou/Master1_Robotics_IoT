clc 
close all 
clear all


syms N E D phi theta psi u v w p q r 
syms Ix Iy Iz
syms F1 F2 F3 F4 F5 F6
syms d1x d1y d1z
syms d2x d2y d2z
syms d3x d3y d3z
syms d4x d4y d4z
syms d5x d5y d5z
syms d6x d6y d6z

rt1 = [d1x d1y d1z];
rt2 = [d2x d2y d2z];
rt3 = [d3x d3y d3z];
rt4 = [d4x d4y d4z];
rt5 = [d5x d5y d5z];
rt6 = [d6x d6y d6z];

%Les paramètres fixes du problème
syms rho R L m V g
% rho=1000; 
% R=0.15;
% L=1.72;
% m=55;
% V=34/100 ; %Buoyancy volmume coefficient
% g=10; 


%% Vecteur d'état
X=[N,E,D,phi,theta,psi,u,v,w,p,q,r]';
X1=[N E D phi theta psi]'; %position vector in Earth frame
X2=[u v w p q r]';         %velocity vector in Body Frame
%%

%%Friction forces
syms klu klv klw klp klq klr
syms kqu kqv kqw kqp kqq kqr

Kl=diag([klu klv klw klp klq klr]);
Kq=diag([kqu kqv kqw kqp kqq kqr]);



Kb=-Kl*X2-Kq*norm(X2)*X2; 

%Gravity 
W=m*g; %wheight
B=rho*V*g; %Poussé d'archimède

syms r_g1 r_g2 r_g3 
syms r_b1 r_b2 r_b3 
r_g=[0 0 0 ]';
r_b=[0 0 r_b3]';



g_nu=-1*[(W-B)*sin(theta), 
    -(W-B)*cos(theta)*sin(phi), 
    -(W-B)*cos(theta)*cos(phi),
    -(r_g(2)*W-r_b(2)*B)*cos(theta)*cos(phi)+(r_g(3)*W-r_b(3)*B)*cos(theta)*sin(phi) ,
    (r_g(3)*W-r_b(3)*B)*cos(theta)*cos(phi)
    -(r_g(2)*W-r_b(2)*B)*cos(theta)*sin(phi)-(r_g(2)*W-r_b(2)*B)*sin(theta)]';


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

f7(X')=(F1+F2+g_nu(1)+Kb(1))/M(1,1)+v*r-w*q;    %u'
f8(X')=(F3+F4+g_nu(2)+Kb(2))/M(2,2)+w*p-u*r;    %v'
f9(X')=(F5+F6+g_nu(3)+Kb(3))/M(3,3)+u*p-v*p;    %w'
f10(X')=((M(5,5)-M(6,6))*q*r+g_nu(4)+Kb(4))/M(4,4);          %p'
f11(X')=1/M(5,5)*(-(M(4,4)-M(6,6))*p*r-d5x*F5+d6x*F6+g_nu(5)+Kb(5)); %q'
f12(X')=1/M(6,6)*(-(M(5,5)-M(4,4))*p*q+d4x*F1-d1y*F2+d3x*F3+d4x*F4+g_nu(6)+Kb(6));  %r'
%%
%La jacobienne de f par rapport à N E D 0 0 0 u v w p q r 

f=[f1 ;f2; f3 ;f4 ;f5 ;f6; f7; f8; f9; f10; f11; f12];
J_A = jacobian(f', [N E D phi theta psi u v w p q r] );
J_B = jacobian(f', [F1 F2 F3 F4 F5 F6] );

%%
%Modèle de Représentation d'état
A_mat = simplify(J_A(N ,E ,D ,0 ,0 ,0 ,u ,0,0,0,0,0));
B_mat = J_B ;
C_mat = [zeros(6,6)  eye(6)];
D_mat = zeros(6);


%% 
%Commandabilité 
Co=B_mat;
for i=1:11
    Co=[Co A_mat^i*B_mat];
end 
rang_Co=rank(Co(N, E, D, phi, theta, psi, u, v, w, p, q, r)); %Le système n'est pas commandable et le rang vaut 10
% Nous remarquons les premières 6 lignes de la matrice B sont nulles, donc
% les états N E D phi theta psi ne sont pas commandables


%%
%%%Observabilité 
Ob=C_mat;
for i=1:11
    Ob=[Ob ; 
        C_mat*A_mat^i];
end 
rangOb=rank(Ob);

% Nous remarquons les premières 6 colonnes de la matrice C sont nulles, donc
% les états N E D phi theta psi ne sont pas observables

%%














