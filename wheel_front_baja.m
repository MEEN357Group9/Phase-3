% MEEN 357 Fall 2017 Project, Phase 2
% This is a data structure for the front wheel on Texas A&M 2018 Baja car.

% The following fields are common amoung all wheel assemblies.

field05 = 'model';        % make and size of the tire
field06 = 'weight';       % weight of the tire and its wheel assembly
field07 = 'travel';       % verticle distance the front axle can travel
field08 = 'deflection';   % approximate maximum deflection of the tire
field09 = 'k';            % stiffness of the tire from Hoosier data sheet
field10 = 'c';            % damping from the tire

% The following values pertain to a front wheel.

value05 = 'Maxxis Razr2 23x7-10';
value06 = 25.0;          % pounds
value07 = 12.0;          % inches
value08 = 6.5;           % inches
value09 = 250.0;         % pounds / inch
value10 = 40.0;          % pounds / (inch/sec)

% The data struct for a front wheel is:

wheel_front = struct(field05, value05, field06, value06, ...
                     field07, value07, field08, value08, ...
                     field09, value09, field10, value10);
