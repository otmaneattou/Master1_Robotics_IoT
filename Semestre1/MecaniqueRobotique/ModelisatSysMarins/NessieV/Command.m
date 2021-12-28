function [Out] = Command(PosE,Error_precedent)
global Ebinv
Kp = 100;          % proportional term Kp
Ki = 0;          % Integral term Ki
Kd = 100;          % derivative term Kd 

pos_d =[10 0 0 0 0 0]'; % consigne 

dt=0.01;

Error =  pos_d -PosE  

P_error = Error; % error propr
D_error  = (Error - Error_precedent)/dt; % error dériv
I_error  = (Error + Error_precedent)*dt/2; %  error integral   
PID  = Kp*P_error + Ki*I_error+ Kd*D_error;
   
Thrust=Ebinv*PID*100;
Error_precedent=Error;
Out=[Thrust;Error_precedent];
