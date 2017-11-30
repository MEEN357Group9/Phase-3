function [T, X, V, A] = MS2PECE(X0, V0, A0, M, C, K, FN, D)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[siz, ~] = size(X0);
TM = zeros(1);
XM = zeros(1,siz);
VM = zeros(1,siz);
AM = zeros(1,siz);
XM(1,:) = X0;
VM(1,:) = V0;
AM(1,:) = A0;
TM(1) = D.t_in;

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

%% Evaluate

A1 = M^(-1) * (FN(t,D) - C*V1 - K*X1);

TM(2,:) = t1;
XM(2,:) = X1;
VM(2,:) = V1;
AM(2,:) = A1;

%% Remaining
for i = 2:D.N
    
    % predict
    X2p = 1/3.*(4.*XM(i,:) - XM(i-1,:))+ h/6.*(3.*VM(i,:) + VM(i-1,:))...
        + h^2/36.*(31.*AM(i,:)-AM(i-1,:)); 
    V2p = 1/3.*(4.* VM(i,:) - VM(i-1,:)) + 2*h/3.*(2.*AM(i,:) - AM(i-1,:));
    
    % evaluate
    A2p = M^(-1) * ( FN(t,D) - C*V2p' - K*X2p');
    TM(i+1) = TM(i) + h;
    
    X2 = 1/3.*(4.*XM(i,:) - XM(i-1,:)) + h/24.*(V2p + 14.*VM(i,:) + VM(i-1,:)) +...
        h^2/72.*(10.*A2p' + 51.*AM(i,:) - AM(i-1,:));
    V2 = 1/3.*(4*VM(i,:) - VM(i-1,:)) + 2*h/3.*A2p';
    
    A2 = M^(-1) * ( FN(t,D) - C*V2' - K*X2');
    XM(i+1,:) = X2;
    VM(i+1,:) = V2;
    AM(i+1,:) = A2;
    

end
end

