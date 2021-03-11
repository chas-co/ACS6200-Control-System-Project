% This script is for the design of the Droop controller for the paralleled
% buck converters
clc
clear
%% The system parameters are declared

R=0.4608; % Load resistance
Vi1=100; Vi2=100;% Input Voltage for converter 1 & 2
fs=10000; % Switching Frequency
L1=0.479e-3; L2=0.342e-3;% Filter Inductor
rL1=0.002; rL2=0.002; % Inductor Resistance for converter 1 & 2
C= 271.25e-6;   %Filter Capacitance
rC=0; %Capacitor Resistance ESR
D=0.48; %Duty cycle
Rdrp=0.0093; %droop resistance
Vm=1; % sawtooth waveform with a peak to peak amplitude 

% The State Space model and Transfer Function of the Buck Converter 1 
A =[-rL1/L1 -1/L1;1/C -1/(R*C)]; B = [Vi1/L1;0]; Cc = [1 0]; Dd = 0;
Gid1 = tf(ss(A,B,Cc,Dd, 'StateName',{'vc','il'},'inputname',{'d'},'outputname','I')); %  duty cycle to Current Transfer Function
Gvd1 = tf(ss(A,B,[0 1],Dd, 'StateName',{'vc','il'},'inputname',{'d'},'outputname','V')); % duty cycle to voltage Transfer Function
currentPlant=minreal(Gid1*1/Vm);

% The State Space model and Transfer Function of the Buck Converter 2
Aa =[-rL2/L2 -1/L2;1/C -1/(R*C)]; Bb = [Vi2/L2;0]; Ccc = [1 0]; Ddd = 0;
Gid2 = tf(ss(Aa,Bb,Ccc,Ddd, 'StateName',{'vc','il'},'inputname',{'d'},'outputname','I')); % Current to duty cycle Transfer Function
Gvd2 = tf(ss(Aa,Bb,[0 1],Ddd, 'StateName',{'vc','il'},'inputname',{'d'},'outputname','V')); % Current to duty cycle Transfer Function
currentPlant2=minreal(Gid2*1/Vm);

%% PID Current Controller Design using LQR tuning method 
% The State Space Matrices for PID lqr tuning formulation Converter 1
num=currentPlant.Numerator{1};
den=currentPlant.Denominator{1};
A1=[0 1 0; 0 0 1; 0 -den(3) -den(2)]; % A matrix for state variable representation of Gid
B1= [0;0;-num(3)]; % B matrix for state variable representation of Gid

% The State Space Matrices for PID lqr tuning formulation Converter 2
num2=currentPlant2.Numerator{1};
den2=currentPlant2.Denominator{1};
A2=[0 1 0; 0 0 1;0 -den2(3) -den2(2)]; % A matrix for state space representation of Gid
B2= [0;0;-num2(3)]; % B matrix for state variable representation of Gid

% The weight matrices Qx and Qu converter 1
Qx= [65e5 0 0;0 20e-6 0; 0 0 7e-7]; % State Weight Matrix
Qu= 10;        % Input weight matirx

% The weight matrices Qx and Qu Converter 2
Qx2= [70e5 0 0; 0 2e-6 0; 0 0 3e-7]; % State Weight Matrix
Qu2= 10;        % Input weight matirx

% The Continuous Algebraic Riccati Equation for the P  in the equation PA + A' - PBR- 1 BT + Q = 0 is solved
[P,H,GG] = care(A1,B1,Qx,Qu);
[P2,H2,GG2] = care(A2,B2,Qx2,Qu2);

% the Ki and Kp gain values for the PI controller using the equation u(t)=-(R^-1)B'Px(t) are computed
u=-(1/Qu)*B1'*P;  u2=-(1/Qu2)*B2'*P2; 
iKi=u(1);  iKi2=u2(1); %Intergral Gain value 
iKp=u(2);  iKp2=u2(2); %Proportional Gain value
iKd=u(3);  iKd2=u2(3); %Derivative Gain value
 
%% Computing the Closed Loop Transfer Function
s=tf('s');

% PI/PID controller transfer function using the calculated gains is derived
currentController=iKp+(iKi/s)+iKd*s; 
currentController2=iKp2+(iKi2/s)+iKd2*s; 

%  the Current loop gain is computed
currentLoopGain=minreal( currentController*currentPlant);
currentLoopGain2=minreal( currentController2*currentPlant2);

% The closed loop transfer function is computed
currentPlantCL=feedback(currentLoopGain,1);
currentPlantCL2=feedback(currentLoopGain2,1);

% The design is validated by checking system performance on step input and bode plot
figure(3), step(currentPlantCL), grid on 
figure(4), step(currentPlantCL2), grid on 
figure (5), margin(currentLoopGain2), grid on 
figure (6), margin(currentLoopGain), grid on 

%% V-I Voltage loop PI Controller Design
% The tunable PI controller with its inputs and outputs are  set up
Gvi=minreal(Gvd1/Gid1); Gvi2=minreal(Gvd2/Gid2);
VPI0 = tunablePID('voltageController','pi'); VPI02 = tunablePID('voltageController2','pi');
voltagePI=VPI0; voltagePI2=VPI02;
voltagePI.InputName='e'; voltagePI2.InputName='e2';
voltagePI.OutputName='iref'; voltagePI2.OutputName='iref2';

% The other system blocks are set up 
% The droop resistance feedback block 
Rdroop=tf(Rdrp);  Rdroop2=tf(Rdrp);
Rdroop.InputName='il';  Rdroop2.InputName='il2';
Rdroop.OutputName='Vdrp'; Rdroop2.OutputName='Vdrp';

% The closed loop current loop block
currentPlantCL.InputName = 'iref'; currentPlantCL2.InputName = 'iref2';
currentPlantCL.OutputName = 'il'; currentPlantCL2.OutputName = 'il2';

% The current to output voltage transfer function 
Gvi.InputName = 'il'; Gvi2.InputName = 'il2';
Gvi.OutputName = 'Vo'; Gvi2.OutputName = 'Vo2';

% The Feedback junction 
sum1 = sumblk('e = Vref - Vo- Vdrp'); sum2 = sumblk('e2 = Vref2 - Vo2- Vdrp2');

% The Voltage Plant for the design of the voltage loop PI controller is setup
voltagePlant=connect(Gvi,currentPlantCL,'iref','Vo');
voltagePlant2=connect(Gvi2,currentPlantCL2,'iref2','Vo2');

% The controller structure to be tuned is set up
C0=connect(voltagePI,sum1,{'Vref','Vo','Vdrp'},'iref');
C02=connect(voltagePI2,sum2,{'Vref2','Vo2','Vdrp2'},'iref2');

% The desired crossover frequency is chosen and the looptune function is used to tune the system
%wc is chosen far below the wc of the current control loop 
wc1=100;
wc12=100;
[~,vloopBlock,~,Info] = looptune(voltagePlant,C0,wc1);
[~,vloopBlock2,~,Info] = looptune(voltagePlant2,C02,wc12);

% The PI controller is extracted from tuned vloopBlock
vloopPI = getBlockValue(vloopBlock,'voltageController');
vloopPI2 = getBlockValue(vloopBlock2,'voltageController2');

% The closed loop voltage transfer function/ voltage restoration loop transfer function is computed 
vResPlant= minreal((vloopPI*minreal(tf(voltagePlant)))/(1+(vloopPI*minreal(tf(voltagePlant))*(1+(Rdrp/Gvi)))));
figure (7), step(vResPlant), grid on
vResPlant2= minreal((vloopPI2*minreal(tf(voltagePlant2)))/(1+(vloopPI2*minreal(tf(voltagePlant2))*(1+(Rdrp/Gvi2)))));
figure (8), step(vResPlant2), grid on

%% Voltage Restoration Loop
% The tunable PI controller with its inputs and outputs are  set up
TPI=tunablePID('vResPI','pi');
vResCon = TPI* [1 , -1];
vResCon.InputName = {'Vgref','Vgrid'};
vResCon.OutputName = 'Vres';

% The voltage restoration plant input and output are set up
vResPlant.InputName = 'Vres';
vResPlant.OutputName = 'Vgrid';

% The desired crossover frequency is chosen and the looptune function is used to tune the system
%wc is chosen far below the wc of the voltage control loop 
wc2 = 2;   
[~,voltageResController,~,Info] = looptune(vResPlant,vResCon,wc2);

% The PI controller is extracted and  the closed-loop transfer function is computed
vResPI = getBlockValue(voltageResController,'vResPI');
buckSystemCL = connect(vResPlant,voltageResController,'Vgref','Vgrid');
figure(9), step(buckSystemCL)