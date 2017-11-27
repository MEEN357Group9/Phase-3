% MEEN 357 Fall 2017 Project, Phase 2
% This is a data structure for the rear wheel on the Texas A&M 2014 car.

% The following fields are common amoung all wheel assemblies.

field11 = 'model';        % make and size of the tire
field12 = 'weight';       % weight of the tire and its wheel assembly
field13 = 'travel';       % verticle distance the rear axle can travel
field14 = 'deflection';   % approximate maximum deflection of the tire
field15 = 'k';            % stiffness of the tire from Hoosier data sheet
field16 = 'c';            % damping from the tire

% The following values pertain to a rear wheel.

value11 = 'Hoosier  18X7.5-10 R25B';
value12 = 25;             % pounds
value13 = 2;              % inches
value14 = 2;              % inches
value15 = 772.20;         % pounds / inch  (18 psi 200 lbs load)
value16 = 3;              % pounds / (inch/sec)

% The data struct for a rear wheel is:

wheel_rear = struct(field11, value11, field12, value12, ...
                    field13, value13, field14, value14, ...
                    field15, value15, field16, value16);
