function [R_f, R_r, R_dot_f, R_dot_r] = agony(wheelbase, X_enter, X, V)
%
% AGONY
%
% Provides the profile for a drive down agony road.
%
%
% INPUTS   wheelbase  distance between front and rear axles            [ft]
%          X_enter    roadway location where front axle enters agony   [ft]
%          X          current location of the front axle               [ft]
%          V          current speed of the vehicle                 [ft/sec]
% OUTPUTS  R_f        vertical displacement at the front axle          [ft]
%          R_r        vertical distancement at the rear  axle          [ft]
%          R_dot_f    vertical velocity of the front axle          [ft/sec]
%          R_dot_r    vertical velocity of the rear  axle          [ft/sec]
%

% verify the input

if ~isscalar(wheelbase)
  error('Variable wheelbase must be a scalar in call to agony.');
end
if ~isscalar(X_enter)
  error('Variable X_enter must be a scalar in call to agony.');
end
if ~isscalar(X)
  error('Variable X must be a scalar in call to agony.');
end
if ~isscalar(V)
  error('Variable V must be a scalar in call to agony.');
end
if wheelbase <= 0
  error('Variable wheelbase > 0 in a call to agony.');
end
if X < 0
  error('Variable X must not be negative in a call to agony.');
end
if V < 0
  error('Variable V must not be negative in a call to agony.');
end

% main body of the function

% quantify the geometry of agony road, dimensions are in feet

% first leg of the roadway: a hill

length0 = 3;

X_enter = X_enter + length0;

length1 = 25;
height1 = 1;
top1    = 0;

% second leg of the roadway: a pot hole

length2 = 1;
height2 = -3 / 12;
top2    = 0.75;

% third leg of the roadway: a valley 

length3 = 15;
height3 = -1;
top3    = 3;

% fourth leg of the roadway: another pot hole

length4 = 2;
height4 = -3 / 12;
top4    = 1.75;

% the front axle

if (X < X_enter) || (X > X_enter + length1 + length2 + length3 + length4)
  % front axle is not on agony road
  R_f     = 0;
  R_dot_f = 0;
elseif (X - X_enter) <= length1
  % front axle is in leg 1
  [R_f, R_dot_f] = bump(length1, height1, top1, X - X_enter, V);
elseif (X - X_enter - length1) <= length2
  % front axle is in leg 2
  [R_f, R_dot_f] = bump(length2, height2, top2, X - X_enter - length1, V);
elseif (X - X_enter - length1 -length2) <= length3
  % front axle is in leg 3
  [R_f, R_dot_f] = bump(length3, height3, top3, ...
                        X - X_enter - length1 - length2, V);
else
  % front axle is in leg 4, the final leg of the roadway
  [R_f, R_dot_f] = bump(length4, height4, top4, ...
                        X - X_enter - length1 - length2 - length3, V);
end

% the rear axle

if (X - wheelbase < X_enter) || ...
   (X - wheelbase > X_enter + length1 + length2 + length3 + length4)
  % rear axle is not on agony road
  R_r     = 0;
  R_dot_r = 0;
elseif (X - wheelbase - X_enter) <= length1
  % rear axle is in leg 1
  [R_r, R_dot_r] = bump(length1, height1, top1, X - wheelbase - X_enter, V);
elseif (X - wheelbase - X_enter - length1) <= length2
  % rear axle is in leg 2
  [R_r, R_dot_r] = bump(length2, height2, top2, ...
                 X - wheelbase - X_enter - length1, V);
elseif (X - wheelbase - X_enter - length1 - length2) <= length3
  % rear axle is in leg 3
  [R_r, R_dot_r] = bump(length3, height3, top3, ...
                 X - wheelbase - X_enter - length1 - length2, V);
else
  % rear axle is in leg 4, the final leg of the roadway
  [R_r, R_dot_r] = bump(length4, height4, top4, ...
                 X - wheelbase - X_enter - length1 - length2 - length3, V);
end

end
