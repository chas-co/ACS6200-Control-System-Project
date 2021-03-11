%This Programme is for the design of the PID controller for a single buck converter
%functions for Buck converter
clc
clear
Vi=100; %input voltage 
R=0.4608; %Converter Resistance Value
L=0.240e-3; %Converter inductance value
rL=0.002; % Inductor Resistance 
C=271.28e-6; %Converter Capacitance value
fsw = 10000; % switching frequency
fs = 10*fsw; % sampling frequency
T=1/fs; % Sampling time
%% State Space Model of the Buck Converter
Ao=[-rL/L -1/L;1/C -1/(R*C)]; % A is the Dynamics Matrix
Bo=[Vi/L; 0]; % B is the Control Input Matrix
Co=[0 1]; % C is the Ouput Matrix 
Do=0; % D is the direct term
buckSS= ss(Ao,Bo,Co,Do,'StateName',{'il','vc'},'inputname',{'d'},'outputname','Vo'); % Creates the state space model of the buck converter

% transfer function between output voltage and duty ratio
TFvO_d=tf(buckSS);

figure(1), margin(TFvO_d), grid on

%% Designing the Voltage Controller for a single buck converter using LQR to tune the PI/PID  
% Formulation of the state space of the system for PID with lqr tuning 
num=TFvO_d.Numerator{1};
den=TFvO_d.Denominator{1};
A1=[0 1 0
    0 0 1
    0 -den(3) -den(2)];
B1= [0
     0
    -num(3)];
% Choose the weight matrices Qx and Qu 
Qx=[5.5e1 0 0 
     0 25e-6 0 
     0 0 5e-14];
Qu= 1;
% Solve the Continuous Algebraic Riccati Equation for the P  in the equation PA + A' - PBR- 1 BT + Q = 0 
[P,EE,GG] = care(A1,B1,Qx,Qu);
%Calculate for Ki Kp and Kd using the equation u(t)=-(R^-1)B'Px(t)
tert=lqr(A1,B1,Qx,Qu)
u=-(1/Qu)*B1'*P;
Ki=u(1); %integral gain
Kp=u(2); %Proportional gain
Kd=u(3); %Derivative gain
%% Computing the Close loop system and Checking for Performance
s=tf('s');
buckPID=pid(Kp,Ki,Kd);%Kp+(Ki/s)+ s*Kd;
buckLoopGain= buckPID*TFvO_d;% Loop gain of the 
buckCL=feedback(buckLoopGain,1);

%Checking the closed loop poles
buckpoles=pole(buckCL);
%Ploting the time domain and frequency response
figure(3), step(buckCL,0.1)
figure (4)
margin(buckLoopGain)
stepinfo(buckCL)