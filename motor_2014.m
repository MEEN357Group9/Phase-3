% MEEN 357 Fall 2017 Project, Phase 2
% Data structure for the motor and transmission for the Texas A&M 2014 car.

% The following fields are common amoung all motors.

field29 = 'model';      % make and size of the power plant
field30 = 'weight';     % weight of engine and transmission unit
field31 = 'diameter';   % diameter as an approximation to a sphere

% The following values pertain to a single cylinder engine.

value29 = 'Yamaha WR450';  % 450 cc single cylinder engine
value30 = 70;              % pounds
value31 = 9;               % inches

% The data struct for a 1 cylinder engine and transmission:

power_plant = struct(field29, value29, field30, value30, field31, value31);
