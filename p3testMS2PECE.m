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
[TM, XM, VM, AM] = MS2PECE(X0, V0, A0, M, C, K, FN, D);
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
