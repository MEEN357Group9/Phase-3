% MEEN 357 Fall 2017 Project, Phase 2
% This is a data structure for the Texas A&M 2018 FSAE Baja car.

% Load the data structures for the 2018 Texas A&M Baja car.

driver_harry;
wheel_front_baja;
wheel_rear_baja;
suspension_front_baja;
suspension_rear_baja;
motor_baja;
chassis_baja;

% The following fields are common amoung all FSAE and Baja race cars.

field46 = 'team';              % name of the racing team
field47 = 'year';              % year the car competed
field48 = 'top_speed';         % maximum speed of the car
field49 = 't2top_speed';       % time to go from 0 to top speed
field50 = 'pilot';             % driver of the car
field51 = 'chassis';           % the chassis design
field52 = 'power_plant';       % the engine in the car
field53 = 'suspension_front';  % the front suspension design
field54 = 'suspension_rear';   % the rear  suspension design
field55 = 'wheel_front';       % the front wheel assembly
field56 = 'wheel_rear';        % the rear  wheel assembly

% compute top speed properties

top_speed = 43;                % miles per hour
time2top  = 10.0;              % seconds

% The following values define an FSAE race car.

value46 = 'Texas A&M';
value47 = 2014;
value48 = top_speed;           % miles per hour
value49 = time2top;            % seconds
value50 = pilot;
value51 = chassis;
value52 = power_plant;
value53 = suspension_front;
value54 = suspension_rear;
value55 = wheel_front;
value56 = wheel_rear;

% The data struct for FSAE and Baja race cars is:

FSAE_Race_Car = struct(field46, value46, field47, value47, ...
                       field48, value48, field49, value49, ...
                       field50, value50, field51, value51, ...
                       field52, value52, field53, value53, ...
                       field54, value54, field55, value55, ...
                       field56, value56);
