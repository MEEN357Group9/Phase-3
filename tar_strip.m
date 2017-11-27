function [R_f, R_r, R_dot_f, R_dot_r] = tar_strip(wheelbase, X_enter, X, V)
%
% TAR_STRIP
%
% Provides the profile for a tar strip.
%
%
% INPUTS   wheelbase  distance between front and rear axles            [ft]
%          X_enter    roadway location where front axle hits the tar   [ft]
%          X          current location of the front axle               [ft]
%          V          current speed of the vehicle                 [ft/sec]
% OUTPUTS  R_f        vertical displacement at the front axle          [ft]
%          R_r        vertical distancement at the rear  axle          [ft]
%          R_dot_f    vertical velocity of the front axle          [ft/sec]
%          R_dot_r    vertical velocity of the rear  axle          [ft/sec]
%

% verify the input

if ~isscalar(wheelbase)
  error('Variable wheelbase must be a scalar in call to tar_strip.');
end
if ~isscalar(X_enter)
  error('Variable X_enter must be a scalar in call to tar_strip.');
end
if ~isscalar(X)
  error('Variable X must be a scalar in call to tar_strip.');
end
if ~isscalar(V)
  error('Variable V must be a scalar in call to tar_strip.');
end
if wheelbase <= 0
  error('Variable wheelbase > 0 in a call to tar_strip.');
end
if X < 0
  error('Variable X must not be negative in a call to tar_strip.');
end
if V < 0
  error('Variable V must not be negative in a call to tar_strip.');
end

% main body of the function

% quantify the geometry of the tar strip, dimensions are in feet

length = 2     / 12;
height = 0.125 / 12;
top    = 1.5   / 12;

% the front axle

if (X < X_enter) || (X > X_enter + length)
  % front axle is not on the tar strip
  R_f     = 0;
  R_dot_f = 0;
else
  % front axle is on the tar strip
  [R_f, R_dot_f] = bump(length, height, top, X - X_enter, V);
end

% the rear axle

if (X - wheelbase < X_enter) || (X - wheelbase > X_enter + length)
  % rear axle is not on the tar strip
  R_r     = 0;
  R_dot_r = 0;
else
  % rear axle is on the tar strip
  [R_r, R_dot_r] = bump(length, height, top, X - wheelbase - X_enter, V);
end

end
