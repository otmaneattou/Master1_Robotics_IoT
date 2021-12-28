
clc;
clear all
close all




L = 4; % Longueur d'une patte

Ef = [6;0]; % Position de l'épaule avant

Er = Ef - [2.5*L;0]; % Position de l'épaule arriere

Base = [Er,Ef];% la base du robot 

a1 = 1;
a2 = 2*a1;

a3 = 1;
a4 = 2*a3;


 
 
 %% calc patte forward (patte avant)
 X1=[3 , 3.36  ,  4  ,         5   ,   6  ,  7   ,                  8   , 8.64  , 9       ,7.5 ,6 ,5,4.5,3.5 ,3 ];%le cycoilde choisi repere X 
 Z1=[-7, -6.5 , -6.076 ,       -5.8 , -5.8 , -5.8 ,                -6.076  , -6.5 , -7    ,-7   ,-7,-7,-7 ,-7,-7 ];% le cycloide choisi reper Y 
l=length(X1);
A1 = (X1(:)- Ef(1))/L;
B1 = (Z1(:)-Ef(2))/L;
alpha2_a_d1=acos(1-0.5*(A1(:).^2+B1(:).^2));% calcule du l'ongle alpha2
for j=1:l
a=cos(alpha2_a_d1(j))-1;
b=sin(alpha2_a_d1(j));
 c=-sin(alpha2_a_d1(j));
d=cos(alpha2_a_d1(j))-1;
M=[a,b;c,d];

 Imat1 = inv(M)
Result1 = Imat1*[A1(j); B1(j)];
 alpha1_a_d1(j)=acos(Result1(1));
 a1(j) = alpha1_a_d1(j);% calcule du l'ongle alpha1
 a2(j) = alpha2_a_d1(j);% calcule du l'ongle alpha2
end 

%% calc patte rear 
Xr1=[-7 , -6.64  ,  -6  ,        - 5   ,  - 4  ,  -3   ,                  -2   , -1.36  , -1       ,-2.5 ,-3,-4.5 ,-5.5 ,-6.5,-7 ];
Zr1=[-7, -6.5 , -6.076 ,       -5.8 , -5.8 , -5.8 ,                -6.076  , -6.5 , -7             ,-7 ,-7 ,-7  ,-7,-7,-7];
Ar = (Xr1(:) - Er(1))/L;
Br = (Zr1(:)-Er(2))/L;
alpha2r=acos(1-0.5*(Ar(:).^2+Br(:).^2));
for j=1:l
a=-cos(alpha2r(j))+1;
b=-sin(alpha2r(j));
 c= -sin(alpha2r(j));
d=cos(alpha2r(j))-1;
Mr=[a,b;c,d];
Imatr = inv(Mr);
Resultr = Imatr*[Ar(j); Br(j)];
alpha1r(j)=acos(Resultr(1));

ar1(j) =alpha1r(j);
ar2 (j)= alpha2r(j);
end 


%% affichage 
j=[1:1:l];
Cf = [L*cos(a1(j)+pi);L*sin(a1(j)+pi)]+Ef; % position du coude
Pf = Cf+[L*cos(a1(j)-a2(j));L*sin(a1(j)-a2(j))];%position d'epaule 
 
Cr = [L*cos(-ar1(j));L*sin(-ar1(j))]+Er; % position du coude rear 
Pr = Cr+[L*cos(pi+ar2(j)-ar1(j));L*sin(pi+ar2(j)-ar1(j))];%position d'epaule rear 

%% affichage 
figure(2)

plot(Base(1,:),Base(2,:),'Color','Green','LineWidth',1.5)%9
axis([Base(1,1)-5, Base(1,2)+5, -8,0]);  

hold on 

for i=1:2:((l*2)-1)
%--- tracahge du patte avant
plot([Ef(1),Cf(i)],[Ef(2),Cf(i+1)],'k')%8
plot([Ef(1),Cf(i)],[Ef(2),Cf(i+1)],'ok')%7
  hold on 
plot([Cf(i),Pf(i)],[Cf(i+1),Pf(i+1)],'k')%6
plot([Cf(i),Pf(i)],[Cf(i+1),Pf(i+1)],'or')%5


%% --tracage du patte arriere 

 plot([Er(1),Cr(i)],[Er(2),Cr(i+1)],'k')%4
plot([Er(1),Cr(i)],[Ef(2),Cr(i+1)],'ok')%3
  hold on 
plot([Cr(i),Pr(i)],[Cr(i+1),Pr(i+1)],'k')%2
plot([Cr(i),Pr(i)],[Cr(i+1),Pr(i+1)],'or')%1

end






%% affichage animé 
F1 = figure(1)

%tracahge model du base 
plot(Base(1,:),Base(2,:),'Color','Green','LineWidth',1.5)%9
axis([Base(1,1)-5, Base(1,2)+5, -8,0]);  
hold on 

plot([Ef(1),Cf(1)],[Ef(2),Cf(2)],'k')
plot([Ef(1),Cf(1)],[Ef(2),Cf(2)],'ok')


plot([Cf(1),Pf(1)],[Cf(2),Pf(2)],'k')
plot([Cf(1),Pf(1)],[Cf(2),Pf(2)],'ok')


%% TRACE PATTE ARRIERE
plot([Er(1),Cr(1)],[Er(2),Cr(2)],'k')
plot([Er(1),Cr(1)],[Er(2),Cr(2)],'ok')


plot([Cr(1),Pr(1)],[Cr(2),Pr(2)],'k')
plot([Cr(1),Pr(1)],[Cr(2),Pr(2)],'ok')


%% affichage animé 
hold off
Axe = F1.Children(1);

Socle1 =Axe.Children(9);
AvantBras_r = Axe.Children(4);
JointsAvantBras_r = Axe.Children(3);

Bras_r = Axe.Children(2);
JointsBras_r = Axe.Children(1);
%bras avant 

AvantBras = Axe.Children(8);
JointsAvantBras = Axe.Children(7);

Bras = Axe.Children(6);
JointsBras = Axe.Children(5);


for k= 1:3
    j=17;
    for i=1:2:((l*2)-1)
        AvantBras.XData = [Ef(1),Cf(j)];
        JointsAvantBras.XData = [Ef(1),Cf(j)];
    
        AvantBras.YData = [Ef(2),Cf(j+1)];
        JointsAvantBras.YData = [Ef(2),Cf(j+1)];
   
        Bras.XData = [Cf(j),Pf(j)];
        JointsBras.XData = [Cf(j),Pf(j)];
    
        Bras.YData = [Cf(j+1),Pf(j+1)];
        JointsBras.YData = [Cf(j+1),Pf(j+1)];
        
        
        
        AvantBras_r.XData = [Er(1),Cr(i)];
        JointsAvantBras_r.XData = [Er(1),Cr(i)];
    
        AvantBras_r.YData = [Er(2),Cr(i+1)];
        JointsAvantBras_r.YData = [Er(2),Cr(i+1)];
   
      Bras_r.XData = [Cr(i),Pr(i)];
        JointsBras_r.XData = [Cr(i),Pr(i)];
    
      Bras_r.YData = [Cr(i+1),Pr(i+1)];
        JointsBras_r.YData = [Cr(i+1),Pr(i+1)];
       
        if j==((l*2)-1)
            j=1;
        else 
            j=j+2;
        end
      %% patte rear 
    % avant 
      
        pause (1)   
        figure(F1)
    end
    
end