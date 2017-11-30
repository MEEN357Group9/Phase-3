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


%%
