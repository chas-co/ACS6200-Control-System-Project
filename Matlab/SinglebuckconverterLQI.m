%%This Programme is for the simulation Modelling and Simulation of a Buck
%%Converter for Railway Application
%% Initialising Converter Design Parameter
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
Ao=[-rL/L -(1/L);1/C -(1/(R*C))]; % A is the Dynamics Matrix
Bo=[Vi/L; 0]; % B is the Control Input Matrix
Co=[0 1]; % C is the Output Matrix 
Do=0; % D is the direct term
buckSS= ss(Ao,Bo,Co,Do,'StateName',{'il','vc'},'inputname',{'d'},'outputname','Vo'); % Creates the state space model of the buck converter
%Transfer Function Model of the buck converter
buckTF=tf(buckSS);
%% Testing Reachability Controllability and Observabilityy
SysContr=ctrb(Ao,Bo);
SysRank=rank(SysContr);
SysObsv=obsv(Ao,Co);
SysORank=rank(SysObsv);

%% State Feedback Controller Design Usung LQR with Integral

% augmented matrices
Aaug=[Ao zeros(2,1);-Co 0]; %State Augmented Matrix
Baug=[Bo;0];                %Input Augmented Matrix
Caug=[Co,0];                %Output Augmented Matrix
Daug=0;
% The Augmented State Space System
buckSSAug=ss(Aaug,Baug,Caug,Daug,'StateName',{'il','vc','zi'},'inputname',{'d'},'outputname','Vo');
% The Weighting Matrices for the States Qx and the Input Qu are chosen to solve for the feedback gains 
Qx=[1e-5 0 0 
     0 1e-6 0 
     0 0 15]; %State Weights
Qu=0.1; %Input Weight 
% The Augmented State feedback gain Kaug is computed using the lqi command.
% Kaug is made up of the state feedback gain K and the integral gain Ki
Kaug=lqi(buckSS, Qx,Qu);
K=Kaug(1:2) 
Ki=-Kaug(3)
% The Closed Loop System is computed and the eigen values checked for
% stability
Acl3=[Ao-Bo*K Bo*Ki; -Co 0];
buckSSEigval=eig(Acl3);
buckSSCl=ss(Acl3,[0;0;1],Caug,Daug, 'StateName',{'il','vc','zi'},'inputname',{'r'},'outputname','Vo');
figure(1),step(buckSSCl,0.1), grid on
Gr=zpk([],0,1);
figure(3), step(Gr,Gr*buckSSCl,10), grid on
legend('ramp input','ramp response')
%figure(2),margin(tf(buckSSCl)), grid on
stepinfo(buckSSCl)
