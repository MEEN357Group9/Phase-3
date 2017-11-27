

% First we load in all the data we need
run driver_harry.m
run chassis_2016.m
run wheel_front_2016.m
run wheel_rear_2016.m
run suspension_front_2016.m
run suspension_rear_2016.m
run motor_2016.m

% Create the struct fields with which appropriate data will be paired
fieldn1 = 'Team'; % Name of the racing team
fieldn2 = 'Year'; % The year the car competed
fieldn3 = 'top_speed'; % Maximum speed of the car
fieldn4 = 't2top_speed'; % Time to go from 0 to top speed
fieldn5 = 'pilot'; % Data for the driver of the car
fieldn6 = 'chassis'; % Data providing chassis dimensions
fieldn7 = 'power_plant'; % Data for the engine of the car
fieldn8 = 'suspension_front'; % Data for the front suspension
fieldn9 = 'suspension_rear'; % Data for the rear suspension
fieldn10 = 'wheel_front'; % Data for the front wheel assembly
fieldn11 = 'wheel_rear'; % Data for the rear wheel assembly

% Now call specific data or straight up structs to values for big struct
valuen1 = chassis.team;
valuen2 = chassis.year; % date
valuen3 = 175; % mph
valuen4 = pi*2.1/(acos((valuen3-120)/valuen3)); % seconds
valuen5 = pilot;
valuen6 = chassis;
valuen7 = power_plant;
valuen8 = suspension_front;
valuen9 = suspension_rear;
valuen10 = wheel_front;
valuen11 = wheel_rear;

% Now to set up the struct for the car

FSAE_Race_Car = struct(fieldn1, valuen1, fieldn2, valuen2, fieldn3, valuen3, fieldn4, valuen4, ... 
    fieldn5, valuen5, fieldn6, valuen6, fieldn7, valuen7, fieldn8, valuen8, fieldn9, valuen9, ...
    fieldn10, valuen10, fieldn11, valuen11);

    
    