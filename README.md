<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js?config=TeX-MML-AM_CHTML">
</script>

# Tutorial on Linearized MPC controller

This is tutorial of Linearized MPC controller for nonlinear system. In this tutorial basic parts of Linearized MPC controller would be examined. This script shows how to implement controller for nonlinear system provided by equation **dx/dt = f(x,u)** and **y=C*x+D*u**. For details of derivation please refer to  [Zhakatayev, Altay, et al. "Successive linearization based model predictive control of variable stiffness actuated robots." IEEE AIM 2017](http://ieeexplore.ieee.org/document/8014275/)

 Model Predictive Control scheme can be formulated as an optimization problem with horizon length $$np$$:
 
$$ \min\limits_{ u } J = \frac{1}{2}(X-rr)^TQ(X-rr) + \frac{1}{2}u^TRu = \frac{1}{2}u^TGu + f^Tu + C $$

s.t:

$$ A_{con} u \leq B_{con}$$ 

$$ X = [x(1),x(2),...,x(np)] $$

$$ x(k+1) = f(x(k),u(k))$$

In order to solve this problem MATLAB built-in quadprog() function is used. Please refer to documentation of quadprog() function for details. In fact, any nonlinear optimization problem solver can be used to come up with a solution. For example, qpOASES is suitable for real-time operation of robotic systems.


linearize, discretize and how to find solution with simple constraints are shown. We setup out tutorial on simple mathematical pendulum, with equation of motion:

$$\frac{\partial x_1}{\partial t} = x_2 $$ 

$$\frac{\partial x_2}{\partial t} = -\frac{g}{l} sin(x_1) -b*x_2 + u $$
