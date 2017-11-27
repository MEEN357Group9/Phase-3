% MEEN 357 Fall 2017 Project, Phase 2
% This is the data structure for driver Harry.

% The following fields are common amoung all drivers.

field01 = 'name';      % the driver's name
field02 = 'height';    % the driver's height in inches
field03 = 'girth';     % the driver's girth (at rib cage) in inches
field04 = 'weight';    % the dirver's weight in pounds

% The following values to these fields pertain to Harry.

value01 = 'Harry';
value02 = 6 * 12 + 5;  % inches
value03 = 30;          % inches
value04 = 200;         % pounds

% The data struct for Harry follows below.

pilot = struct(field01, value01, field02, value02, ...
               field03, value03, field04, value04);