% Phase 2, Task 3a


% Load data that we need
run car_2014.m

% Create the struct fields with which appropriate data will be paired
field1 = 't_prev'; % Data managed by the trajectory integrator (sec)
field2 = 'X_prev'; % Data managed by the trajectory integrator (ft)
field3 = 'car'; % Race car being driven (ie FSAE_Race_Car)
field4 = 'model'; % Vibration model (ie half_car_4_DOF)
field5 = 'trajectory'; % A handle to any admissible trajectory function
field6 = 't_in'; % Time upon entering the trajectory (sec)
field7 = 't_out'; % Time when leaving the trajectory (sec)
field8 = 'V_in'; % Velocity upon entering the trajectory (mph)
field9 = 'V_out'; % Velocity when leaving the trajectory (mph)
field10 = 'N'; % Number of time steps (ie h = [t_out - t_in]/N)
field11 = 'roadway_d'; % A handle to any admissible roadway function traversed by driver side
field12 = 'X_enter_d'; % Distance to engage the roadway waveform
field13 = 'roadway_p'; % A handle to any admissible roadway function traversed by passenger side
field14 = 'X_enter_p'; % Distance to engage the roadway waveform

% Now call specific data or straight up structs to values for big struct
value1 = 0;
value2 = 0;
value3 = FSAE_Race_Car;
value4 = 'quarter_car_1_DOF';
value5 = @(t, X, h, t_in, t_out, V_in, V_out, FSAE_Race_Car) trajectory(t, X, h, t_in, t_out, V_in, V_out, FSAE_Race_Car);
value6 = 0; % sec
value7 = 1.5; % sec
value8 = 5; %mph
value9 = 5; % mph
value10 = 2500; 
value11 = @(wheelbase,X_enter,X,V)speed_bump(wheelbase,X_enter,X,V);
value12 = 1; % ft
value13 = @(wheelbase,X_enter,X,V)speed_bump(wheelbase,X_enter,X,V);
value14 = 1; % ft

% Now to set up the struct for the car
ff_data = struct(field1, value1, field2, value2, field3, value3, field4, value4, ...
    field5, value5, field6, value6, field7, value7, field8, value8, ...
    field9, value9, field10, value10, field11, value11, field12, value12, ...
    field13, value13, field14, value14);



