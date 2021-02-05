% Alban-Félix Barreteau, M1 CORO SIP
% Mail : alban-felix.barreteau@eleves.ec-nantes.fr
% Matlab R2020a Update 5 (9.8.0.1451342), Student license
% Signal processing, Subject 3 (Space shuttle attitude simulation)

% loic.michel@ec-nantes.fr
% eric.le-carpentier@ec-nantes.fr

%----------------------------------------------------------------%
%% Variables declaration
clc

w0=2; %Nominal angular frequency omega_0
G=1;
x0=[0;0]; %Initial state vector
tmin=0; tmax=8; %Time domain
ts=0.1; %Sampling period
tspan=[tmin tmax]; %CT case 
Nspan=[tmin/ts;tmax/ts]; %DT case


%% Simulink

% Parameters of the PWM
E=0.05; %Nominal torque
tpwm=0.02; %Period of the PWM

% Parameters of the PID controller
Ki=2.70;
Kp=1.25;
Kd=0.75;

% Attack angle alphatilde
alphanom=0.3; %Nominal value
alphac0=0.30; alphatildec0=alphac0-alphanom; %Initial wanted value
alphac1=0.27; alphatildec1=alphac1-alphanom; %Final wanted value

% Torque input u
tstep=1; %The begining of the torque
tc=tmin:0.0001:tmax;
u=0.05*(tc>=tstep); u0=0; u1=0.05; %The torque values


%% Prelaminary study 

% State space representation in the CT case
A=[0 1;-w0^2 0]; 
B=[0;w0^2*G];
C=[1 0];
D=[0];
sysc=ss(A,B,C,D); %Definition of the SSR
[numctf,denctf]=tfdata(sysc,'v'); %Extract the num and the den of the continuous TF
%hc=tf(numctf, denctf); %The TF of the CT system


%% Calculation of the discretized SSR

sysd=c2d(sysc,ts,'zoh'); %Discretized the SSR with the ZOH method
[Atilde,Btilde,Ctilde,Dtilde,ts]=ssdata(sysd); %Extract the discretized matrix of the SSR
[numdtf,dendtf,ts]=tfdata(sysd,'v'); %Extract the num and the den of the discretized TF

% % Numerical computation 
% tf(sysc)
% tf(sysd)
% ss(sysd)


%% ode45 : Solving the differential equation in the CT case
tic
[t,X] = ode45(@(t,x) navettecontinue(A,B,x,t), tspan, x0); timeCTode=toc;
figure(10); hold off
plot(t,X)
legend('\alphatilde_{CT}','\omega_{CT}',"location",'southeast')
xlabel('Time t (in s)'); ylabel('Evolution of the 2 components of the state vector CT case');
set(figure(10),'name','ode45: Describe the evolution of the state vector x(t)')


%% ore : Solving the recursion equation in the DT case
tic
[n, Xe] = ore(@(n,xe) equarec(Atilde,Btilde,n,xe,ts), Nspan, x0); timeDTore=toc;
figure(20); hold off
plot(n*ts,Xe,'x')
legend('\alphatilde_{DT}','\omega_{DT}',"location",'southeast')
xlabel('Time t (in s)'); ylabel('Evolution of the 2 components of the state vector DT case');
set(figure(20),'name','ore: Describe the recursion of the state vector x[n]')


%% Evolution of the state vector x for both CT and DT systems
figure(30), hold off
plot(t,X), hold on, plot(n*ts,Xe,'x')
legend('\alphatilde_{CT}','\omega_{CT}','\alphatilde_{DT}','\omega_{DT}',"location",'southeast')
xlabel('Time t (in s)'); ylabel('Evolution of the 2 components of the state vector');
set(figure(30),'name','ode45 & ore: Describe the evolution of the state vector x in both systems')


%% Simulation of alphatilde (CT and DT systems)
figure(40); hold off
plot(tc,u,'k'), hold on
plot(t,X(:,1),'b-')
plot(n*ts,Xe(:,1),'rx')
legend('Input u(t)','\alphatilde_{CT}','\alphatilde_{DT}',"location",'southeast')
xlabel('Time t (in s)'); ylabel('Evolution of \alphatilde in CT and DT systems');
set(figure(40),'name','ode45 & ore: Evolution of alphatilde for both CT and DT systems')

% Calculation time comparison
ComputationTimeCT=['The computation time in the continuous time (CT) case is ',num2str(timeCTode),'second(s)']
ComputationTimeDT=['The computation time in the discrete time (DT) case is ',num2str(timeDTore),'second(s)']
RT=['The computation time is ',num2str(timeCTode/timeDTore),' times longer in the CT case with the ode45 function than in the DT one with the ore function']


