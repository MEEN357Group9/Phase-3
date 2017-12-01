% p3task5  Drives the  baja Texas A&M race car of Phase 1 over the
% speed bump roadway using forcing-function data structures f_baja_1, f_baja_2, f_baja_3
% f_baja_4, f_baja_5, and f_baja_6.
%
% Phase 3, Task 5
% Group 9: Kelsey Banasik, Zarah Navarro, Sonia Sanchez, Harland Ashby
%

%%
% Quarter Car 1 DOF
run ff_baja_1;

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
run ff_baja_2;

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
run ff_baja_3;

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
run ff_baja_4;

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
run ff_baja_5;

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
run ff_baja_6;

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

figure

%Displacement
subplot(3,1,1);
plot(T1,X1(:,1),'k', T2,X2(:,1),'r', T3,X3(:,1),'b', T4,X4(:,1),'g', ...
    T5,X5(:,1),'c', T6,X6(:,1),'m');
legend('1/4 Car 1 DOF','1/4 Car 2 DOF','1/2 Car 2 DOF','1/2 Car 4 DOF', ...
    'Full Car 3 DOF','Full Car 7 DOF');
title('Heave Displacement of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Displacement [ft]');

% Velocity
subplot(3,1,2);
plot(T1,V1(:,1),'k', T2,V2(:,1),'r', T3,V3(:,1),'b', T4,V4(:,1),'g', ...
    T5,V5(:,1),'c', T6,V6(:,1),'m');
legend('1/4 Car 1 DOF','1/4 Car 2 DOF','1/2 Car 2 DOF','1/2 Car 4 DOF', ...
    'Full Car 3 DOF','Full Car 7 DOF');
title('Heave Velocity of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Velocity [ft/s]');

%Acceleration
subplot(3,1,3);
plot(T1,A1(:,1),'k', T2,A2(:,1),'r', T3,A3(:,1),'b', T4,A4(:,1),'g', ...
    T5,A5(:,1),'c', T6,A6(:,1),'m');
legend('1/4 Car 1 DOF','1/4 Car 2 DOF','1/2 Car 2 DOF','1/2 Car 4 DOF', ...
    'Full Car 3 DOF','Full Car 7 DOF');
title('Heave Acceleration of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Acceleration [ft/s^2]');

%% Pitch

figure

%Displacement
subplot(3,1,1);
plot(T3,X3(:,2)*180/pi,'b', T4,X4(:,2)*180/pi,'g', ...
    T5,X5(:,2)*180/pi,'c', T6,X6(:,2)*180/pi,'m');
legend('1/2 Car 2 DOF','1/2 Car 4 DOF','Full Car 3 DOF','Full Car 7 DOF');
title('Pitch Displacement of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Rotation [deg]');

% Velocity
subplot(3,1,2);
plot(T3,V3(:,2)*180/pi,'b', T4,V4(:,2)*180/pi,'g', ...
    T5,V5(:,2)*180/pi,'c', T6,V6(:,2)*180/pi,'m');
legend('1/2 Car 2 DOF','1/2 Car 4 DOF','Full Car 3 DOF','Full Car 7 DOF');
title('Pitch Velocity of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Velocity [deg/s]');

%Acceleration
subplot(3,1,3);
plot(T3,A3(:,2)*180/pi,'b', T4,A4(:,2)*180/pi,'g', ...
    T5,A5(:,2)*180/pi,'c', T6,A6(:,2)*180/pi,'m');
legend('1/2 Car 2 DOF','1/2 Car 4 DOF','Full Car 3 DOF','Full Car 7 DOF');
title('Pitch Acceleration of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Acceleration [deg/s^2]');

%% Roll

figure

%Displacement
subplot(3,1,1);
plot(T5,X5(:,3)*180/pi,'c', T6,X6(:,3)*180/pi,'m');
legend('Full Car 3 DOF','Full Car 7 DOF');
title('Roll Displacement of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Rotation [deg]');

% Velocity
subplot(3,1,2);
plot(T5,V5(:,3)*180/pi,'c', T6,V6(:,3)*180/pi,'m');
legend('Full Car 3 DOF','Full Car 7 DOF');
title('Roll Velocity of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Velocity [deg/s]');

%Acceleration
subplot(3,1,3);
plot(T5,A5(:,3)*180/pi,'c', T6,A6(:,3)*180/pi,'m');
legend('Full Car 3 DOF','Full Car 7 DOF');
title('Roll Acceleration of Baja Car over Agony');
xlabel('Time [s]');
ylabel('Acceleration [deg/s^2]');

%% Comparison

% Car lengths 
l = ff_data.car.chassis.length;
cg = get_cg(ff_data.car); % ft
lf = cg; % ft
lr = l - cg; % Wheelbase in ft
rf = ff_data.car.chassis.radius_f; % ft
rr = ff_data.car.chassis.radius_r; % ft
L = lf + lr; % Wheelbase in ft
W = rf + rr; % Tire Tracking Width

% Vertical Motion for Warp
zdf = X6(:,4);    % heave of driver front axle
zpf = X6(:,5);    % heave of passenger front axle
zpr = X6(:,6);    % heave of passenger rear axle
zdr = X6(:,7);    % heave of driver rear axle

% Axle Plane z, theta, phi, and omega
heave_a = (lr/(2*L))*zdf + (lr/(2*L))*zpf + (lf/(2*L))*zpr + (lf/(2*L))*zdr;
pitch_a = (-1/(2*L))*zdf + (-1/(2*L))*zpf + (1/(2*L))*zpr + (1/(2*L))*zdr;
roll_a = (-1/(2*W))*zdf + (1/(2*W))*zpf + (1/(2*W))*zpr + (-1/(2*W))*zdr;
warp_a = (-rr/W)*zdf + (rr/W)*zpf + (-rf/W)*zpr + (rf/W)*zdr;


% Motion VS Axle Plane
figure

% Heave
subplot(2,2,1);
plot(T6,heave_a,'r', T6,X6(:,1),'b');
legend('Axle Plane','Chassis Plane');
title('Heave Displacement for Baja Car');
xlabel('Time [s]');
ylabel('Heave Displacement [ft]');

% Pitch
subplot(2,2,2);
plot(T6,pitch_a*180/pi,'r', T6,X6(:,2)*180/pi,'b');
legend('Axle Plane','Chassis Plane');
title('Pitch Displacement for Baja Car');
xlabel('Time [s]');
ylabel('Pitch Displacement [deg]');

% Roll
subplot(2,2,3);
plot(T6,roll_a*180/pi,'r', T6,X6(:,3)*180/pi,'b');
legend('Axle Plane','Chassis Plane');
title('Roll Displacement for Baja Car');
xlabel('Time [s]');
ylabel('Roll Displacement [deg]');

% Warp
subplot(2,2,4);
plot(T6,warp_a,'r');
legend('Axle Plane');
title('Warp of Axle Plane for Baja Car');
xlabel('Time [s]');
ylabel('Warp Displacement [ft]');

