<link rel="stylesheet" href="/lmpc-matlab/assets/scripts/styles/default.css">
<script src="/lmpc-matlab/assets/scripts/highlight.pack.js"></script>
<script>hljs.initHighlightingOnLoad();</script>

<script type="text/x-mathjax-config">
MathJax.Hub.Config({
  tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}
});
</script>
<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js?config=TeX-MML-AM_CHTML">
</script>


# Tutorial on Linearized MPC controller

This tutorial covers implementation of basic parts of successive linearization based model predictive controller. This script shows how to implement controller for nonlinear system provided by equation 

\begin{align}
\dot{x} &= f(x,u) \newline
y&=Cx+Du
\end{align}
. For details of derivation please refer to  Zhakatayev, Altay, et al. "[Successive linearization based model predictive control of variable stiffness actuated robots.](## Citation)" IEEE AIM 2017

## Problem statement
MPC controller requires several parameters:
`np` - horizon length
`Q`  - matrix representing relative importance of states (also we use vector 'wx')
`R`  - matrix representing penalizing large values in control inputs ( vector 'wu')

Model Predictive Control scheme can be formulated as an optimization problem with horizon length $$np$$:
 
$$ \min\limits_{ u } J = \frac{1}{2}(X-rr)^TQ(X-rr) + \frac{1}{2}u^TRu = \frac{1}{2}u^TGu + f^Tu + Constant $$

subject to:

$$ A_{con} u \leq B_{con}$$ 

$$ X = [x(1),x(2),...,x(np)]^T $$

$$ x(k+1) = f(x(k),u(k))$$

In order to solve this problem MATLAB built-in `quadprog()` function is used. Please refer to documentation of `quadprog()`  [function for details](https://www.mathworks.com/help/optim/ug/quadprog.html?requestedDomain=true).
In fact, any nonlinear optimization problem solver can be used to come up with a solution. For example, qpOASES is suitable for real-time operation of robotic systems.

## Diagram of how LMPC controller works 
![figure 1](figure2.gif)
## Tutorial objectives
This tutorial covers implementation of basic parts of SLMPC controller. They are:
1. [Linearization of model](https://en.wikipedia.org/wiki/Linearization) 
2. [Discretization of linearized model](https://en.wikipedia.org/wiki/Discretization)
3. solving Linearized MPC controller optimiation problem with simple constraints

We setup out tutorial on simple mathematical pendulum, with equation of motion:

$$\frac{\partial x_1}{\partial t} = x_2 $$ 

$$\frac{\partial x_2}{\partial t} = -\frac{g}{l} sin(x_1) -b*x_2 + u $$

Following MATLAB code shows how do we implement MPC controller:

<pre>
<code class="matlab">
close all; clear; clc;
% global parameters associated with dynamic model of the system 
global g; g = 9.81;  
global l; l = 0.1;  
global b; b = 0.2;  
% this is matrices to represent output as y=C*x+D*u
C = eye(2);  
D = [0;0];
%initial point of states
x = [0.001; 0];
%control input max and min values
ui=0;
umax = 80;
umin =-20;
% IMPORTANT PARAMETERS
np = 40;       % horizon length 
nx = 2;        % number of states 
nu = 1;        % number of inputs
no = size(C,1);% number of outputs
Ts = 0.001;    % step size
Tfinal = 0.5;  % final time
wx = [1000 1]; % relative importance of states
wu = 0.00001;  % penalizing weights of control inputs
% generating simple step reference of the form [0.5 ; 0] 
ref =0.5*[ones(1,Tfinal/Ts +np);zeros(1,Tfinal/Ts +np)];
% model is anonymous function that represents the equation which describes 
% system dynamic model and in the form of dx/dt =f(x,u). 
model = @(x,u) nonlin_eq(x,u); 
% FOR MPC CONTROLLER
rr = zeros(np*nx,1);
y  = zeros(no,Tfinal/Ts);
uh = zeros(nu,Tfinal/Ts);
%constraints for inputs in the whole horizon in the form of Acon*u <=Bcon
[Acon,Bcon] = simple_constraints(umax,umin,np,nu);
% weighting coefficient reflecting the relative importance of states and control inputs 
Q = diag(repmat(wx, 1, np)); 
R = diag(repmat(wu, 1, np));
% Setting the quadprog with 200 iterations at maximum
opts = optimoptions('quadprog', 'MaxIter', 200, 'Display','off');
% MAIN SIMULATION LOOP
for t=1:Tfinal/Ts
    
    rr =  ref_for_hor(rr,ref,t,np,nx);% reference vector for whole horizon  
    y(:,t) = C*x+D*ui;                % evaluating system output 
    [x, dx] = RK4(x,ui,Ts,model);     % i.e. simulate one step forward with Runge-Kutta 4 order integrator
    [A, B, K] = linearize_model(x,dx,ui);                           % linearization step
    [Ad,Bd,Kd] = discretize(A,B,K,Ts);                              % discretization step
    [G, f] = grad_n_hess(R, Q, Ad, Bd, C, D, Kd, rr, np, x);        % calculating Hessian and gradient of cost function
    u = quadprog(G, f, Acon, Bcon, [], [], [], [], [],opts);
    ui = u(1:nu);     %providing first solution as input to our system
    uh(:,t) = ui;     %storing input
end
% plotting the results
tt = Ts:Ts:Tfinal;
subplot(3,1,1)
plot(tt ,y(1,:),'b', tt ,ref(1,1:(Tfinal/Ts)),'r--');
title('x_1(t) vs t');
legend('state1','reference');
subplot(3,1,2)
plot(tt ,y(2,:),'b',tt,ref(2,1:(Tfinal/Ts)),'r--');
legend('state2','reference');
title('x_2(t) vs t');
subplot(3,1,3)
plot(tt ,uh);
title('u(t) vs t');
</code>
</pre>

### Results
In figure below, results of above code is shown. x1 and x2 are states of system described in [tutorial objectives](## Tutorial objectives). The red dashed lines reprents desired reference for each state. The u(t) represents solution of MPC controller. 
![figure 2](figure1.png)

## Citation
If you use our code please we kindly ask to cite the following paper:

<script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/jquery/1.4.2/jquery.min.js"></script>
<script type="text/javascript" src="https://cdn.rawgit.com/pcooksey/bibtex-js/b81606e85986fa8ad0eb66954493bc1c0b3d7ab1/src/bibtex_js.js"></script>
<bibtex src="https://armslab.github.io/lmpc-matlab/assets/citation.bib"></bibtex>

<div id="bibtex_display"></div>
