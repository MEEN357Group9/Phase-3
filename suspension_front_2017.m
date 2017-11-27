% MEEN 357 Fall 2017 Project, Phase 2
% This is a data structure for the front suspension of Texas A&M 2017 car.

% The following fields are common amoung suspensions.

field17 = 'model';      % make and model of the shock absorber
field18 = 'location';   % shock location: 'inboard' or 'outboard'
field19 = 'angle';      % angle (degrees) of shock shaft from vertical 
                        % this field applies for outboard mounted shocks
field20 = 'travel';     % total piston travel of the shock absorber
field21 = 'k';          % stiffness for the suspension spring
field22 = 'c';          % damping coefficient for the shock absorber

% The following values to these fields pertain to a front suspension.

value17 = 'Ohlins TTX25 MKII';  % shock assembly used
value18 = 'outboard';           % location of the shock absorber assembly
value19 = 44;                   % angle:     degrees
value20 = 2;                    % stroke:    inches
value21 = 350;                  % stiffness: pounds / inch
value22 = 5;                    % damping:   pounds / (inch/sec)

% The data struct for a front suspension follows below.

suspension_front = struct(field17, value17, field18, value18, ...
                          field19, value19, field20, value20, ...
                          field21, value21, field22, value22);
