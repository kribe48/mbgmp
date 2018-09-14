function [opti, X, U, T, modelParams, initStateParams, finalStateParams] = setup_opti_car(modelParameters,N)
%SETUP_OPTI_CAR Summary of this function goes here
%   Detailed explanation goes here
opti = casadi.Opti(); % Optimization problem

nx = 5;
nu = 1;
% ---- decision variables ---------
X = opti.variable(nx,N+1); % state trajectory

% Control signal
U = opti.variable(nu,N);

% final time
T = opti.variable();
% Parameters
modelParams = opti.parameter(4);
finalStateParams = opti.parameter(nx);
initStateParams = opti.parameter(nx);

L = modelParams(1);
alpha_max = modelParams(2);
w_max = modelParams(3);
u_max = modelParams(4);

opti.set_value(modelParams, modelParameters);
% ---- dynamic constraints --------
f = @(x,u,p) [cos(x(3));sin(x(3));tan(x(4))/p;x(5);u]; % dx/dt = f(x,u)

% ----- Runge-Kutta integration --------
dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),             U(:,k), L);
   k2 = f(X(:,k)+dt/2*k1, U(:,k), L);
   k3 = f(X(:,k)+dt/2*k2, U(:,k), L);
   k4 = f(X(:,k)+dt*k3,   U(:,k), L);
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- control and state constraints -----------
alpha = X(4,:);
w = X(5,:);
opti.subject_to(-u_max<=U<=u_max);    
opti.subject_to(-alpha_max<= alpha <= alpha_max);
opti.subject_to(-w_max<= w <= w_max);

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ----- Zero initialization on parameters
opti.set_value(initStateParams, zeros(1,5));
opti.set_value(finalStateParams, zeros(1,5));
% ---------- Select solver
p_opts.expand = true;
s_opts.linear_solver = 'mumps';
s_opts.max_iter = 1000;
s_opts.print_level = 0;
%opti.callback(@(i) plot_iteration(i,opti.debug.value(x),opti.debug.value(y)));
opti.solver('ipopt', p_opts, s_opts); % set numerical backend
end

