function [T, X, V, A] = MS2PECE(X0, V0, A0, M, C, K, FN, D)
%PECE method that integrates M(d^2x/dt^2) + C(dx/dt) + Kx = f(t)
%   Developed by Dr. Alan Freed, this is a third-order accurate method
%   where an explicit predictor is used to get an initial estimate for the
%   solution
%   Inputs:
%       X0  intial conditions for displacement
%       V0  inital conditions for velocity
%       A0  inital conditions for acceleration
%       M   mass matrix
%       C   damping matrix
%       K   stiffness matrix
%       FN  forcing function handle
%       D   data structure used with the forcing function handle
%
%   Outputs:
%       T   times for the solutions
%       X   displacement solutions
%       V   velocity solutions
%       A   acceleration solutions

% Input error handling
sM = size(M);
sC = size(C);
sK = size(K);

sX = size(X0);
sV = size(V0);
sA = size(A0);

% if sM(1) ~= sM(2) || sC(1) ~= sC(2) || sK(1) ~= sK(2)
%     error('The M,C, and K matrices must be square in size.');
% elseif  size(M) ~= size(C) || size(C) ~= size(K) || size(M) ~= size(K)
%     error('The M,C,and K matrices are not the same size');
% elseif sX(2) ~= 1 || sV(2) ~= 1 || sA ~= 1
%     error('X0, V0, and A0 must be column vectors.');
% elseif sX ~= sV ~= sA
%     error('X0, V0, and A0 must be column vectors with the same amount of rows.');
% end
[siz, ~] = size(X0);
T = zeros(1);
X = zeros(1,siz);
V = zeros(1,siz);
A = zeros(1,siz);
X(1,:) = X0;
V(1,:) = V0;
A(1,:) = A0;
T(1) = D.t_in;

t = D.t_prev;

%% Predict
% integrate for the displacement x and velocity v

% step size
h = (D.t_out - D.t_in)/D.N;

X1p = X0 + h.*V0 + h^2/2.*A0;

V1p = V0 +h.*A0;

%% Evaluate
A1p = M^(-1) * ( FN(t,D) - C*V1p - K*X1p);

%% Correct 

t1 = D.t_in + h;
X1 = X0 + h/2.*(V1p + V0) - h^2/12.*(A1p-A0);

V1 = V0 + h/2.*(A1p+A0);

%% Evaluate  do it twice

A1 = M^(-1) * (FN(t,D) - C*V1 - K*X1);

T(2,:) = t1;
X(2,:) = X1;
V(2,:) = V1;
A(2,:) = A1;

%% Remaining
for i = 2:D.N
    
    % predict
    X2p = 1/3.*(4.*X(i,:) - X(i-1,:))+ h/6.*(3.*V(i,:) + V(i-1,:))...
        + h^2/36.*(31.*A(i,:)-A(i-1,:)); 
    V2p = 1/3.*(4.* V(i,:) - V(i-1,:)) + 2*h/3.*(2.*A(i,:) - A(i-1,:));
    
    % evaluate
    A2p = M^(-1) * ( FN(t,D) - C*V2p' - K*X2p');
    T(i+1) = T(i) + h;
    
    X2 = 1/3.*(4.*X(i,:) - X(i-1,:)) + h/24.*(V2p + 14.*V(i,:) + V(i-1,:)) +...
        h^2/72.*(10.*A2p' + 51.*A(i,:) - A(i-1,:));
    V2 = 1/3.*(4*V(i,:) - V(i-1,:)) + 2*h/3.*A2p';
    
    A2 = M^(-1) * ( FN(t,D) - C*V2' - K*X2');
    X(i+1,:) = X2;
    V(i+1,:) = V2;
    A(i+1,:) = A2;
    

end
end

