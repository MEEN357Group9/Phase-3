% MEEN 357 Fall 2017 Project, Phase 2
% This is a data structure for the rear suspension of Texas A&M 2017 car.

% The following fields are common amoung suspensions.

field23 = 'model';      % make and model of the shock absorber
field24 = 'location';   % shock location: 'inboard' or 'outboard'
field25 = 'angle';      % angle (degrees) of shock shaft from vertical 
                        % this field applies for outboard mounted shocks
field26 = 'travel';     % total piston travel of the shock absorber
field27 = 'k';          % stiffness for the suspension spring
field28 = 'c';          % damping coefficient for the shock absorber

% The following values to these fields pertain to a rear suspension.

value23 = 'Ohlins TTX25 MKII';  % shock assembly used
value24 = 'outboard';           % location of the shock absorber assembly
value25 = 31;                   % angle - does not apply, as it's inboard
value26 = 2;                    % stroke:    inches
value27 = 350;                  % stiffness: pounds / inch
value28 = 5;                    % damping:   pounds / (inch/sec) 

% The data struct for a rear suspension follows below.

suspension_rear = struct(field23, value23, field24, value24, ...
                         field25, value25, field26, value26, ...
                         field27, value27, field28, value28);