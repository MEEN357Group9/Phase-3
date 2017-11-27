% MEEN 357 Fall 2017 Project, Phase 2
% Data structure for motor and transmission for Texas A&M 2018 Baja car.

% The following fields are common amoung all motors.

field29 = 'model';      % make and size of the power plant
field30 = 'weight';     % weight of engine and transmission unit
field31 = 'diameter';   % diameter as an approximation to a sphere

% The following values pertain to a single cylinder engine.

value29 = 'Briggs and Stratton Model 20 INTEK';
value30 = 88;           % pounds
value31 = 12;           % inches

% The data struct for a 1 cylinder engine with separate transmission:

power_plant = struct(field29, value29, field30, value30, field31, value31);
