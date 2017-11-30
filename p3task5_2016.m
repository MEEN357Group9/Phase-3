% p3task5  Drives the  2016 Texas A&M race car of Phase 1 over the
% speed bump roadway using forcing-function data structures f_2016_1, f_2016_2, f_2016_3
% f_2016_4, f_2016_5, and f_2016_6.
%
% Phase 3, Task 5
% Group 9: Kelsey Banasik, Zarah Navarro, Sonia Sanchez, Harland Ashby
%


%%
% Quarter Car 1 DOF
run ff_2016_1;

D = ff_data;
FN = @(t, D) get_forcing_function(t, D);
X0 = get_static_deflection(D.model, D.car);
DOF = size(X0, 1);
V0 = zeros(DOF, 1);
A0 = zeros(DOF, 1);
M = get_mass_matrix(D.model, D.car);
C = get_damping_matrix(D.model, D.car);
K = get_stiffness_matrix(D.model, D.car);
[T1, X1, V1, A1] = MS2PECE(X0, V0, A0, M, C, K, FN, D);


% Quarter Car 2 DOF
run ff_2016_2;

D = ff_data;
FN = @(t, D) get_forcing_function(t, D);
X0 = get_static_deflection(D.model, D.car);
DOF = size(X0, 1);
V0 = zeros(DOF, 1);
A0 = zeros(DOF, 1);
M = get_mass_matrix(D.model, D.car);
C = get_damping_matrix(D.model, D.car);
K = get_stiffness_matrix(D.model, D.car);
[T2, X2, V2, A2] = MS2PECE(X0, V0, A0, M, C, K, FN, D);


% Half Car 2 DOF
run ff_2016_3;

D = ff_data;
FN = @(t, D) get_forcing_function(t, D);
X0 = get_static_deflection(D.model, D.car);
DOF = size(X0, 1);
V0 = zeros(DOF, 1);
A0 = zeros(DOF, 1);
M = get_mass_matrix(D.model, D.car);
C = get_damping_matrix(D.model, D.car);
K = get_stiffness_matrix(D.model, D.car);
[T3, X3, V3, A3] = MS2PECE(X0, V0, A0, M, C, K, FN, D);


% Half Car 4 DOF
run ff_2016_4;

D = ff_data;
FN = @(t, D) get_forcing_function(t, D);
X0 = get_static_deflection(D.model, D.car);
DOF = size(X0, 1);
V0 = zeros(DOF, 1);
A0 = zeros(DOF, 1);
M = get_mass_matrix(D.model, D.car);
C = get_damping_matrix(D.model, D.car);
K = get_stiffness_matrix(D.model, D.car);
[T4, X4, V4, A4] = MS2PECE(X0, V0, A0, M, C, K, FN, D);



% Full Car 3 DOF
run ff_2016_5;

D = ff_data;
FN = @(t, D) get_forcing_function(t, D);
X0 = get_static_deflection(D.model, D.car);
DOF = size(X0, 1);
V0 = zeros(DOF, 1);
A0 = zeros(DOF, 1);
M = get_mass_matrix(D.model, D.car);
C = get_damping_matrix(D.model, D.car);
K = get_stiffness_matrix(D.model, D.car);
[T5, X5, V5, A5] = MS2PECE(X0, V0, A0, M, C, K, FN, D);



% Full Car 7 DOF
run ff_2016_6;

D = ff_data;
FN = @(t, D) get_forcing_function(t, D);
X0 = get_static_deflection(D.model, D.car);
DOF = size(X0, 1);
V0 = zeros(DOF, 1);
A0 = zeros(DOF, 1);
M = get_mass_matrix(D.model, D.car);
C = get_damping_matrix(D.model, D.car);
K = get_stiffness_matrix(D.model, D.car);
[T6, X6, V6, A6] = MS2PECE(X0, V0, A0, M, C, K, FN, D);


%% Heave

%Displacement
subplot(3,1,1)
plot(T1,X1(:,1),'k', T2,X2(:,1),'r', T3,X3(:,1),'b', T4,X4(:,1),'g', ...
    T5,X5(:,1),'c', T6,X6(:,1),'m')
ylim([-0.3 0.2])
legend('1/4 car 1 DOF','1/4 car 2 DOF','1/2 car 2 DOF','1/2 car 4 DOF', ...
    'full car 3 DOF','full car 7 DOF')
title('Displacements: Heave of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Displacement [ft]')

% Velocity
subplot(3,1,2)
plot(T1,V1(:,1),'k', T2,V2(:,1),'r', T3,V3(:,1),'b', T4,V4(:,1),'g', ...
    T5,V5(:,1),'c', T6,V6(:,1),'m')
ylim([-5 10])
legend('1/4 car 1 DOF','1/4 car 2 DOF','1/2 car 2 DOF','1/2 car 4 DOF', ...
    'full car 3 DOF','full car 7 DOF')
title('Velocities: Heave of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Velocity [ft/s]')

%Acceleration
subplot(3,1,3)
plot(T1,A1(:,1),'k', T2,A2(:,1),'r', T3,A3(:,1),'b', T4,A4(:,1),'g', ...
    T5,A5(:,1),'c', T6,A6(:,1),'m')
ylim([-200 300])
legend('1/4 car 1 DOF','1/4 car 2 DOF','1/2 car 2 DOF','1/2 car 4 DOF', ...
    'full car 3 DOF','full car 7 DOF')
title('Accelerations: Heave of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Acceleration [ft/s^2]')

%% Pitch

figure

%Displacement
subplot(3,1,1)
plot(T3,X3(:,2),'b', T4,X4(:,2),'g', T5,X5(:,2),'c', T6,X6(:,2),'m')
ylim([-4 4])
legend('1/2 car 2 DOF','1/2 car 4 DOF','full car 3 DOF','full car 7 DOF')
title('Rotation: Pitch of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Rotation [deg]')

% Velocity
subplot(3,1,2)
plot(T3,V3(:,2),'b', T4,V4(:,2),'g', T5,V5(:,2),'c', T6,V6(:,2),'m')
ylim([-100 100])
legend('1/2 car 2 DOF','1/2 car 4 DOF','full car 3 DOF','full car 7 DOF')
title('Spin: Pitch of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Velocity [deg/s]')

%Acceleration
subplot(3,1,3)
plot(T3,A3(:,2),'b', T4,A4(:,2),'g', T5,A5(:,2),'c', T6,A6(:,2),'m')
ylim([-4000 4000])
legend('1/2 car 2 DOF','1/2 car 4 DOF','full car 3 DOF','full car 7 DOF')
title('Rate of Spin: Pitch of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Acceleration [deg/s^2]')

%% Roll

figure

%Displacement
subplot(3,1,1)
plot(T5,X5(:,3),'c', T6,X6(:,3),'m')
ylim([-4 4])
legend('full car 3 DOF','full car 7 DOF')
title('Rotation: Roll of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Rotation [deg]')

% Velocity
subplot(3,1,2)
plot(T5,V5(:,3),'c', T6,V6(:,3),'m')
ylim([-100 100])
legend('full car 3 DOF','full car 7 DOF')
title('Spin: Roll of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Velocity [deg/s]')

%Acceleration
subplot(3,1,3)
plot(T5,A5(:,3),'c', T6,A6(:,3),'m')
ylim([-4000 4000])
legend('full car 3 DOF','full car 7 DOF')
title('Rate of Spin: Roll of Car Hitting a Speed Bump')
xlabel('Time [s]')
ylabel('Acceleration [deg/s^2]')

%% Comparison

figure

% Heave
subplot(2,2,1)

% Pitch
subplot(2,2,2)

% Roll
subplot(2,2,3)

% Warp
subplot(2,2,4)


