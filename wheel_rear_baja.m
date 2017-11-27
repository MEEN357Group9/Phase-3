% MEEN 357 Fall 2017 Project, Phase 2
% This is a data structure for the rear wheel on Texas A&M 2018 Baja car.

% The following fields are common amoung all wheel assemblies.

field11 = 'model';        % make and size of the tire
field12 = 'weight';       % weight of the tire and its wheel assembly
field13 = 'travel';       % verticle distance the rear axle can travel
field14 = 'deflection';   % approximate maximum deflection of the tire
field15 = 'k';            % stiffness of the tire
field16 = 'c';            % damping from the tire

% The following values pertain to a rear wheel.

value11 = 'Maxxis Razr2 23x7-10';
value12 = 27.0;          % pounds
value13 = 12.0;          % inches
value14 = 6.5;           % inches
value15 = 300.0;         % pounds / inch
value16 = 20.0;          % pounds / (inch/sec)

% The data struct for a rear wheel is:

wheel_rear = struct(field11, value11, field12, value12, ...
                    field13, value13, field14, value14, ...
                    field15, value15, field16, value16);
