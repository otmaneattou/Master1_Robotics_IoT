function [Out] = Observateur(VitB_M,AccB_M,PosE_M)

%Nous implitons le code permettant de réaliser le modèle d'état
global ICPos ICSpeed
global Eb Ebinv Kt Tau Img Imb Iag Iab Mg Kl Kq 
global rb rg P B yg xg zg xb yb zb M Iy Iz Ix Ma22 Ma33 Ma44 Ma55 Ma66 Ma11 

%% Initial Speed and position in Earth-fixed frame

ICPos = [0 0 2 0 0 0]   ;
ICSpeed = [0 0 0 0 0 0] ;

rho_water = 1000                    ;  	% Masse volumique de l'eau (kg/m^3)
R = 0.15                            ;  	% Nessie Radius (m)
L = 1.72                            ;  	% Nessie length (m)
M = 55                              ; 	% Nessie mass (kg)
V = 0.4 * (4/3*pi*R^3 + L*pi*R^2)   ;	% Nessie Volume for buoyancy (m^3) % coef changé de 0.34 à 0.4
g = 9.81                            ;	% Earth Gravity (m*s^(-2))
P = M * g                           ;	% Nessie weight (N)
B = rho_water * V * g               ;	% Buoyancy (N)

%% Center of gravity and Buoyancy position in body-fied frame

xg = 0 ;    %x-positon of Center of gravity
yg = 0 ;    %y-positon of Center of gravity
zg = 0 ;    %z-positon of Center of gravity

rg = [xg yg zg]' ;

S_Rgb = S_(rg) ;

xb = 0      ;    % x-positon of Center of Buoyancy
yb = 0      ;    % y-positon of Center of Buoyancy
zb = -0.02  ;    % z-positon of Center of Buoyancy

rb = [xb yb zb]' ;



%% Mass matrices

% Transport matrixes
Huyguens = [yg^2 + zg^2       -xg*yg          -xg*zg    ;
              -xg*yg        xg^2 + zg^2       -yg*zg    ;
              -xg*zg          -yg*zg       xg^2 + yg^2] ;

% Inertia coefficients for a solid cylinder

Ix = (1/2) * M * R^2                    ;
Iy = (1/4) * M * R^2 + (1/12)* M * L^2  ;
Iz = (1/4) * M * R^2 + (1/12)* M * L^2  ;

% Inertia Matrix in center of mass
Img = [Ix       0           0;
       0        Iy          0;
       0        0          Iz];
 
% Transport of inertia matrix to center of body-fixed frame (Theoreme d'Huygens) 

Imb = Img + M * Huyguens ;      % Similar to : : Imb = Img - (M * S_Rgb^2)


Mb_b =  [M*eye(3) , -M*S_Rgb ; M*S_Rgb , Imb] ;


% Added Mass
% p. 7/9
% ----------------------------

Ma11 = 0.1 * M                                  ;	% added mass in surge (kg)
Ma22 = pi * rho_water * R^2 * L                 ;	% added mass in sway (kg)
Ma33 = pi * rho_water * R^2 * L                 ;   % added mass in heave (kg)
Ma44 = 0                                        ;	% added mass in roll (kg)
Ma55 = (R^2/12) * (0.4*M + pi*rho_water*L^3)    ;   % added mass in pitch (kg)
Ma66 = (R^2/12) * (0.4*M + pi*rho_water*L^3)    ;   % added mass in yaw (kg)

% Added mass inertia matrix (solid cylinder) in center of mass

Iag = [Ma44     0       0 ;
        0      Ma55     0 ;
        0       0      Ma66] ;

Maa = [Ma11     0       0 ;
        0      Ma22     0 ;
        0       0      Ma33] ;

%Transport of added mass matrix to center of body-fixed frame (Theoreme d'Huygens)

Iab = Iag + Maa * Huyguens ;    % Similar to : Iab = Iag - (Maa * S_Rgb^2)

Ma_b = [Maa , -Maa*S_Rgb ; Maa*S_Rgb , Iab] ;

% Generalized mass matrix in center of body-fixed frame;

Mg = Mb_b + Ma_b ; 


%% Friction matrices

% Skin linear friction


klu = 2     ;   % surge
klv = 23    ;   % sway
klw = 0     ;   % heave
klp = 10    ;   % roll
klq = 0     ;   % pitch
klr = 0     ;   % yaw

% Skin quadratic friction

kqu = 32    ;   % surge
kqv = 223   ;   % sway
kqw = 263   ;   % heave
kqp = 0     ;   % roll
kqq = 40    ;   % pitch
kqr = 40    ;   % yaw

%Friction matrices

Kq = -diag([kqu kqv kqw kqp kqq kqr]) ;    %Quadratic friction matrix
Kl = -diag([klu klv klw klp klq klr]) ;    %Skin friction matrix


%% Kinematics matrix (in file called generalised_rotation_matrix)




%% Dynamics matrices or kinetics matices




%% Thruster modelling

%Thruster positions in body-fixed frame

% Thruster positions
% p. 3/9
% ----------------------------


d1x = -0.270    ; 
d1y = 0.175     ;
d1z = 0         ;
d2x = -0.270    ; 
d2y = -0.175    ;
d2z = 0         ;
d3x = 0.530     ;
d3y = 0         ;
d3z = 0.05      ;
d4x = -0.715    ;
d4y = 0         ;
d4z = -0.05     ;
d5x = 0.430     ;
d5y = 0         ;
d5z = 0         ;
d6x = -0.620    ;
d6y = 0         ;
d6z = 0         ;


rt1 = [d1x, d1y, d1z]' ;
rt2 = [d2x, d2y, d2z]' ;
rt3 = [d3x, d3y, d3z]' ;
rt4 = [d4x, d4y, d4z]' ;
rt5 = [d5x, d5y, d5z]' ;
rt6 = [d6x, d6y, d6z]' ;

rt = [rt1 rt2 rt3 rt4 rt5 rt6] ;


%Thruster gains  p.9
kt1 = 28    ;
kt2 = 28    ;
kt3 = 11    ;
kt4 = 11    ;
kt5 = 8.5   ;
kt6 = 8.5   ;

%Thruster gain vectors
Kt=[kt1;kt2;kt3;kt4;kt5;kt6]; %thruster gain

%Thruster time constants
Tau1 = 0.8 ;
Tau2 = 0.8 ;
Tau3 = 0.4 ;
Tau4 = 0.4 ;
Tau5 = 0.4 ;
Tau6 = 0.4 ;

%Thruster time constant vectors
Tau = [Tau1;Tau2;Tau3;Tau4;Tau5;Tau6] ;

% Mapping of thruster
Eb_F = [1 1 0 0 0 0 ;
        0 0 1 1 0 0 ;
        0 0 0 0 1 1 ];
    
Eb_M = rt_vect_F(rt)  ;

Eb = [ Eb_F ; Eb_M ] ;
    % Ub

% Inverse Mapping of thruster
Ebinv = inv(Eb) ;

%Modèle d'état 

A_mat =[
 
 0, 0, 0,                        0,          0, 0,                         1,                                                     0,                       0,      0,                                              0,                                              0;
 0, 0, 0,                        0,          0, 1,                         0,                                                     1,                       0,      0,                                              0,                                              0;
 0, 0, 0,                        0,         -1, 0,                         0,                                                     0,                       1,      0,                                              0,                                              0;
 0, 0, 0,                        0,          0, 0,                         0,                                                     0,                       0,      1,                                              0,                                              0;
 0, 0, 0,                        0,          0, 0,                         0,                                                     0,                       0,      0,                                              1,                                              0;
 0, 0, 0,                        0,          0, 0,                         0,                                                     0,                       0,      0,                                              0,                                              1;
 0, 0, 0,                        0, -(10*g)/11, 0, -(10*(klu + 2*kqu))/(11*M),                                                     0,                       0,      0,                                              0,                                              0;
 0, 0, 0, (g*M)/(L*rho_water*pi*R^2 + M),          0, 0,                         0,                        -(klv + kqv)/(L*rho_water*pi*R^2 + M),                       0,      0,                                              0,                                             -1;
 0, 0, 0,                        0,          0, 0,                         0,                           -(g*rho_water)/(L*rho_water*pi*R^2 + M), -kqw/(L*rho_water*pi*R^2 + M),      1,                                              0,                                              0;
 0, 0, 0,                        0,          0, 0,                         0,                                                     0,                       0, -klp/Ix,                                              0,                                              0;
 0, 0, 0,                        0,          0, 0,                         0, -(g*rho_water)/(50*((rho_water*pi*L^3*R^2)/12 + (M*R^2)/30 + Iy)),                       0,      0, -(60*kqq)/(5*rho_water*pi*L^3*R^2 + 2*M*R^2 + 60*Iy),                                              0;
 0, 0, 0,                        0,          0, 0,                         0,                                                     0,                       0,      0,                                              0, -(60*kqr)/(5*rho_water*pi*L^3*R^2 + 2*M*R^2 + 60*Iz)];

B = [                                          0,                                           0,                                           0,                                           0,                                            0,                                           0;
                                           0,                                           0,                                           0,                                           0,                                            0,                                           0;
                                           0,                                           0,                                           0,                                           0,                                            0,                                           0;
                                           0,                                           0,                                           0,                                           0,                                            0,                                           0;
                                           0,                                           0,                                           0,                                           0,                                            0,                                           0;
                                           0,                                           0,                                           0,                                           0,                                            0,                                           0;
                                   10/(11*M),                                   10/(11*M),                                           0,                                           0,                                            0,                                           0;
                                           0,                                           0,                        1/(L*rho_water*pi*R^2 + M),                        1/(L*rho_water*pi*R^2 + M),                                            0,                                           0;
                                           0,                                           0,                                           0,                                           0,                         1/(L*rho_water*pi*R^2 + M),                        1/(L*rho_water*pi*R^2 + M);
                                           0,                                           0,                                           0,                                           0,                                            0,                                           0;
                                           0,                                           0,                                           0,                                           0, -d5x/((rho_water*pi*L^3*R^2)/12 + (M*R^2)/30 + Iy), d6x/((rho_water*pi*L^3*R^2)/12 + (M*R^2)/30 + Iy);
 d2y/((rho_water*pi*L^3*R^2)/12 + (M*R^2)/30 + Iz), d2y/((rho_water*pi*L^3*R^2)/12 + (M*R^2)/30 + Iz), d3x/((rho_water*pi*L^3*R^2)/12 + (M*R^2)/30 + Iz), d4x/((rho_water*pi*L^3*R^2)/12 + (M*R^2)/30 + Iz),                                            0,                                           0];
 



C=[zeros(6), eye(6)];
D=[zeros(6)]; 



PosE_O=[0 0 0 0 0 0]';
AccB_O=AccB_M;
VitB_O=VitB_M;


Out=[PosE_O ; AccB_O ; VitB_O];
