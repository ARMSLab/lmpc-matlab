% ARMS Lab 2018

close all; clear; clc;
profile on 
% MODEL PARAMETERS
global g ;g = 9.81; % gravity

global Rl; Rl = 0.013; %// joint radius
global Rp; Rp = 0.022; %// motor pulley radius
global PI; PI = 3.1415926535897932; %// Pi
%epsilon = 1e-2;  %// weight of control signals
%H = 0.813;   %// height of (0,0) point from ground level 0.845-0.032

%// spring constants
global alpha1; alpha1 = 12400;% // spring coefficient
global beta1;  beta1  = 1360; %// spring coefficient
global alpha2; alpha2 = 13600; %// spring coefficient
global beta2;  beta2  = 1350; %// spring coefficient
global Lmax;   Lmax   = 0.025; %// spring displacement range, original is 0.04
global x0;     x0     = 0.003; %// initial spring compression

%// link constants
global L1; L1  = 0.433; %// link 1 length
global Lc1;Lc1 = 0.1659; %//link 2 center mass distance
global Lc2;Lc2 = 0.1; %// rotor center mass distance
global m1; m1  = 0.5588; %// mass of link 2
global m2; m2  = 0.2765;%// mass rotor
global m3; m3  = 0.0741; %// mass of ball and magnet
global I1; I1  = 0.009644; %// inertia of link 2 relative to its center of mass
global I2; I2  = 0.00035; %//0.00056743;// inertia of rotor relative to its center of mass
global I3; I3  = 0.00004; %// inertia of ball relative to its center of mass
global b1; b1  = 0.0062; %// damping coefficient 0.0062
global b2; b2  = 1.5542e-4; %// disk damping

%// motor constants
global Im; Im = 0.00139; %// motor rotor inertia, kgm^2 24e-7+47.4e-7
global bm; bm = 0.02; %// motor rotor damping
global Kk; Kk = 0.9720; %// torque constant, Nm/A
global Rm; Rm = 5.4; %// motor resistance, Ohm
% Lm = 0.43e-3; // motor inductance, H
global u10; u10 = 0*PI/180; %// position of motor 1 when spring just starts to extend, in rad
global u20; u20 = 0*PI/180; %// position of motor 2 when spring just starts to extend, in rad
global u1i; u1i = u10 + x0/Rp; %// initial position of motor 1 before motion starts
global u2i; u2i = u20 - x0/Rp; %// initial position of motor 2 before motion starts
global MAXMOTORTORQUE; MAXMOTORTORQUE = 3.4; %// motor stall torque
global dumax; dumax = 5.0;   %// maximum motor velocity, rad/s
global MAXCURRENT; MAXCURRENT = 4.0; %// motor stall current


%Global CONSTANTS
global M;
M = zeros(2,2); %!
m11 = m1*Lc1*Lc1 + m2*Lc2*Lc2 + m3*L1*L1 + I1 + I2 + I3;
m12 = I2;
m22 = I2;
M(1,1)=m11;
M(1,2)=m12;
M(2,1)=m12;
M(2,2)=m22;
global Bk;
Bk = [b1 0 ; 0 b2]; %!
global N1;
N1=g*(m1*Lc1+m2*Lc2+m3*L1);%!
global invM;
invM=inv(M);


np=10; %prediction horizon

nx= 7; % number of states
nu=3;  % number of control inputs
no = 7; % number of outputs 

t_start = 0.00000 ; %// beginning of link motion
t_end = 3.50; %// total time to excite the two-link system, or time till ball leaves the link end
Ts=0.005;
N_points = t_end/Ts+1;

% this is matrices to represent output as y=C*x+D*u
C = eye(no);
D = zeros(nx,nu);
%Initial point of states
x = zeros(nx, 1);
%initial 'velocity' of states  (dx/dt at t0)
dx=x;

x(1)= -PI/2;
x(4) = 0.13636;
x(5) = -0.13636;

% IMPORTANT PARAMETERS FOR MPC CONTROLLER

y = zeros(nx, t_end/Ts+1);

% reading reference from reference file 
rr=zeros(np*nx,1);
ref=zeros(11,741+np);
[ref(1,1:741),ref(2,1:741),ref(3,1:741),ref(4,1:741),ref(5,1:741),ref(6,1:741),ref(7,1:741),ref(8,1:741),ref(9,1:741),ref(10,1:741),ref(11,1:741)] = textread('reference_extended.txt','%f %f %f %f %f %f %f %f %f %f %f');

% model is anonymous function that represents the equation which describes 
% system dynamic model and in the form of dx/dt =f(x,u).
model = @(x,u) nonlin_eq_VSA(x,u);

A=zeros(nx,nx);
B=zeros(nx,nu);
K=zeros(nx,1);

% control inputs
ui=zeros(nu,1);
u=zeros(nu*np,1);
un = zeros(t_end/Ts+1, nu);
uk=zeros(nu,t_end/Ts+1,np);

%maximum and minimum allowed input
mc1=4;
mc2=4;
mc3=3.4;



%weighting matrices of MPC controller
wx = [100 0.5 1e-6 1e-3 1e-3 1e-6 1e-6];
wu = [0.01 0.01 8];

%constraints for inputs in the whole horizon in the form of AS*u <=BS
As=[-1 0 0;1 0 0; 0 -1 0 ; 0 1 0; 0 0 -1; 0 0 1];
AS=zeros(2*nu*np,nu*np);
for ind1=1:np
    AS((2*nu*(ind1-1)+1):2*nu*ind1,(nu*(ind1-1)+1):nu*ind1)=As;
end

Bs=[mc1;mc1;mc2;mc2;mc3;mc3];
BS=repmat(Bs,np,1);

% weighting coefficient reflecting the relative importance of states  
Q = diag(repmat(wx,1,np));
% weight coefficient penalizing relative substantial changes in inputs 
R = diag(repmat(wu,1,np));

% Setting the quadprog with 200 iterations at maximum
opts = optimoptions('quadprog', 'MaxIter', 200, 'Display','off');
% MAIN SIMULATION LOOP
for ind1 =1:length(y)
    % setting-up reference to vector rr in appropriate way
    for ind2 = 1:np
        rr(nx*(ind2-1)+1:ind2*nx,1)=ref(2:(1+nx),ind1+ind2-1);
    end
    % evaluating system output
    y(:,ind1)=C*x+D*ui;
    % linearization step    
    [Ac,Bc,Kc] = linearize_model_VSA(x,dx,ui);
    % discretization step
    [A,B,K] = discretize(Ac,Bc,Kc,Ts);
    % calculating Hessian and gradient of cost function
    [G,f] = grad_n_hess(R , Q , A , B , C , D , K, rr,np, x);
    
    % using quadprog to find solution for our cost function J in
    % given constraints, solution is given as u = [u(1); u(2); ... ; u(np)]    
    u = quadprog(G,f,AS, BS, [], [], [], [], [],opts);
    %savinf all values of u
    for ind2 = 1:np
        uk(:,ind1,ind2)=u((3*(ind2-1)+1):3*ind2);
    end
    %providing first solution as input to our system
    ui = u(1:nu);
    %for first 200 steps inputs are not provided 
    if(ind1<200)
        ui=[0;0;0];
    end
    %storing input
    un(ind1,:) = ui;
    % Propagate system state with Runge-Kutta 4 order integrator
    % i.e. simulate one step forward    
    [x, dx] = RK4(x,ui,Ts,model);
    
end
profile viewer
%figure 1 shows all points of first state changes in each time step as
%responce to each control input on horizon length, compared to reference
figure(1)
yk=zeros(no,np);
%plot(t_start:Ts:t_end,y(1,:),t_start:Ts:t_end,ref(2,1:741));
plot(t_start:Ts:t_end,ref(2,1:N_points));
x = zeros(nx, 1);
dx=x;
x(1)= -PI/2;
x(4) = 0.13636;
x(5) = -0.13636;
for ind1=1:N_points
    hold on
    for ind2=1:np
        [x,dx] = RK4(x, uk(:,ind1,ind2),Ts,model);
        yk(:,ind2)=x;
    end
    x=y(:,ind1);
    plot(Ts*ind1:Ts:Ts*(ind1+np-1),yk(1,:));
end
%  shows first and second state compared to reference and all 3 control
%  inputs 
figure(2)
tt = t_start:Ts:t_end;
subplot(3,1,1)
plot(tt ,y(1,:),'b', tt ,ref(2,1:(t_end/Ts+1)),'r--');
subplot(3,1,2)
plot(tt ,y(2,:),'b',tt,ref(3,1:(t_end/Ts +1)),'r--');
subplot(3,1,3)
plot(tt ,un);
