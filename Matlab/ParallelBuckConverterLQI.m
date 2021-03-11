%%TThis Programme is for the design of the LQI controller for the
%%paralleled buck converters
%% Initialising Converter Design Parameter
clear
Vi1=100; %input voltage 
Vi2=100;
R=0.4608; %Converter Resistance Value
L1=0.479e-3; %Converter inductance value
rL1=0.002; %Inductor Resistance
L2=0.342e-3; %Converter 2 inductance value
rL2=0.002; %Inductor Resistance
C=271.28e-6; %Converter Capacitance value
fsw = 10000; % switching frequency
fs = 2*fsw; % sampling frequency
T=1/fs; % Sampling time

%% State Space Model of the Buck Converter
Ao=[-rL1/L1 0 -(1/L1);0 -rL2/L2 -(1/L2);1/C 1/C -(1/(R*C))]; % A is the Dynamics Matrix
Bo=[Vi1/L1 0;0 Vi2/L2;0 0]; % B is the Control Input Matrix
Co=[0 0 1]; % C is the Ouput Matrix 
Do=0; % D is the direct term
buckPss= ss(Ao,Bo,Co,Do); % Creates the state space model of the buck converter
%% Testing Reachability Controllability and Observabilityy
SysContr=ctrb(Ao,Bo);
SysRank=rank(SysContr);
SysObsv=obsv(Ao,Co);
SysORank=rank(SysObsv);
%% State Feedback Controller Design Usung LQR with Integral
% augmented matrices
Aaug=[Ao zeros(3,1);-Co 0];
Baug=[Bo;zeros(1,2)];
 Caug=[Co,0];
Daug=0;
% The Augmented State Space System
buckPSSAug=ss(Aaug,Baug,Caug,Daug,'StateName',{'i1','i2','vc','zi'},'inputname',{'d1','d2'},'outputname','Vo');
% The Weighting Matrices for the States Qx and the Input Qu are chosen to solve for the feedback gains
Qx1=[11e-5 0 0 0 
     0 53e-5 0 0 
     0 0 1e-6 0 
     0 0 0 1000];
Qu1=[5 0 
     0 5 ]; 
% % The Augmented State feedback gain Kaug is computed using the lqi command.
% Kaug is made up of the state feedback gain K and the integral gain Ki
Kaug=lqi(buckPss, Qx1,Qu1);
K= Kaug(1:2,1:3);
Ki=-Kaug(1:2,4);
% The Closed Loop System is computed and the eigen values checked for
% stability
Acl3=[Ao-Bo*K Bo*Ki; -Co 0];
eigval2=eig(Acl3);
buckPSScl=ss(Acl3,[0;0;0;1],Caug,Daug);
figure(1),step(buckPSScl,0.1), grid on