%% here you can implement the code in order to have some figures ...
%%
close all





%Génération axe des abscisses (Time)
for i = 1 : size(PosE_M(:,1),1)
    Time(i) = 0.01*i ;
end
   


%Les différents positions de Nessie V
Pos_X=PosE_M(:,1);    % positions X en fonction du temps
Pos_Y=PosE_M(:,2);    % positions Y en fonction du temps
Pos_Z=PosE_M(:,3);    % positions Z en fonction du temps
Orientation_X=PosE_M(:,4);   % phi autour de X en fonction du temps
Orientation_Y=PosE_M(:,5);   % theta autour de X en fonction du temps
Orientation_Z=PosE_M(:,6);   % psi autour de X en fonction du temps



figure(1)   
grid on;   

subplot(211)    
plot(Time,Pos_X,'r',Time,Pos_Y,'m+',Time,Pos_Z,'g*')   

legend('Pos_X : Position selon X','Pos_Y : Positiob selon Y','Pos_Z : Position selon Z','Location','northwest')

  
title('Position De Nessie V en fonction du temps ','FontSize',23,...   
       'FontWeight','bold','FontName',...
       'Times New Roman','Color','k')
xlabel('Time','FontSize',15,...             
       'FontWeight','bold','FontName',...
       'Times New Roman','Color','b')
ylabel('Positions en m','FontSize',15,...      
       'FontWeight','bold','FontName',...
       'Times New Roman','Color','b')

subplot(212)   
plot(Time,Orientation_X,'y',Time,Orientation_Y,'c',Time,Orientation_Z,'b','LineWidth',2)   

legend('Phi : Orientation autour X','Theta : Orientation autour Y','Psy : Orientation autour Z','Location','northwest')


title('orientations angulaires autour de  X, Y et Z en fonction du temps','FontSize',23,...      
       'FontWeight','bold','FontName',...
       'Times New Roman','Color','k')
xlabel('Time','FontSize',15,...             
       'FontWeight','bold','FontName',...
       'Times New Roman','Color','b')
ylabel('orientations angulaires','FontSize',15,...     
       'FontWeight','bold','FontName',...
       'Times New Roman','Color','b')
   
   

 


