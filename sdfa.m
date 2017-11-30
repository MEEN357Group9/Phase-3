% MEEN 357 Fall 2017 Project, Phase 3
clc        % clear the terminal/command window
clear all  % clear memory from the workspace
close all  % close extra windows, e.g., graphs

% load the forcing function for the Baja vehicle
ff_baja_6;
D = ff_data;       % established in ff_baja_6
% initialize variables to pass to the integration algorithms
FN  = @(t, D) get_forcing_function(t, D);
X0  = get_static_deflection(D.model, D.car);
DOF = size(X0, 1);
V0  = zeros(DOF, 1);
A0  = zeros(DOF, 1);
M   = get_mass_matrix(D.model, D.car);
C   = get_damping_matrix(D.model, D.car);
K   = get_stiffness_matrix(D.model, D.car);
% integrate to solve the governing system of equations
%[TM, XM, VM, AM] = MS2PECE(X0, V0, A0, M, C, K, FN, D);


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



[TN, XN, VN, AN] = Newmark(X0, V0, A0, M, C, K, FN, D);
% construct the heave curves acquired from both integrators

figure(1)
% Displacement curves
X1 = zeros(D.N+1,1);
X2 = zeros(D.N+1,1);
for ii = 1:D.N+1
   X1(ii) = XM(ii,1);
   X2(ii) = XN(ii,1);
end
subplot(3,1,1)
plot(TM, X1, 'r-', TM, X2, 'g-.')
title('Displacements: Heave of Car Driving Agony Way')
xlabel('Time [s]')
ylabel('Displacement [ft]')
legend('MS2PECE', 'Newmark')

% Velocity curves
V1 = zeros(D.N+1,1);
V2 = zeros(D.N+1,1);
for ii = 1:D.N+1
   V1(ii) = VM(ii,1);
   V2(ii) = VN(ii,1);
end
subplot(3,1,2)
plot(TM, V1, 'r-', TN, V2, 'g-.')
title('Velocities: Heave of Car Driving Agony Way')
xlabel('Time [s]')
ylabel('Velocity [ft/s]')
legend('MS2PECE', 'Newmark')

% Acceleration curves
A1 = zeros(D.N+1,1);
A2 = zeros(D.N+1,1);
for ii = 1:D.N+1
   A1(ii) = AM(ii,1);
   A2(ii) = AN(ii,1);
end
subplot(3,1,3)
plot(TM, A1, 'r-', TN, A2, 'g-.')
title('Accelerations: Heave of Car Driving Agony Way')
xlabel('Time [s]')
ylabel('Acceleration [ft/s^2]')
legend('MS2PECE', 'Newmark')
