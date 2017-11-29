function [T, X, V, A] = MS2PEDE(X0, V0, A0, M, C, K FN, D)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
t = 0.5;
siz = size(X0);
T = zeros(1);
X = zeros(1,siz(1));
V = zeros(1,siz(1));
A = zeros(1,siz(1));
X(1) = X0;
V(1) = V0;
A(1) = A0;
T(1) = D.t_in;

%% Predict
% integrate for the displacement x and velocity v

% step size
h = (D.t_out - D.t_in)/D.N;

X1p = X0 + h*V0 + h^2/2*A0;

V1p = V0 +h*A0;

%% Evaluate
A1p = M^(-1) * ( FN(t,D) - C*V1p - K*X1p);

%% Correct 
X1 = X0 + h/2*(V1p + V0) - h^2/12*(A1p-A0);

V1 = V0 + h/2*(A1p+A0);

%% Evaluate

A1 = M^(-1) * (FN(t,D) - C*V1 - K*X1);


%% Remaining
for i = 1:D.N
    
    % predict
    
    


end

