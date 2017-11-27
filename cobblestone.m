function [R_f, R_r, R_dot_f, R_dot_r] = cobblestone(wheelbase, X_enter, X, V)
%
% COBBLESTONE
%
% Provides the profile for crossing a cobblestone road or intersection.
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
  error('Variable wheelbase must be a scalar in call to cobblestone.');
end
if ~isscalar(X_enter)
  error('Variable X_enter must be a scalar in call to cobblestone.');
end
if ~isscalar(X)
  error('Variable X must be a scalar in call to cobblestone.');
end
if ~isscalar(V)
  error('Variable V must be a scalar in call to cobblestone.');
end
if wheelbase <= 0
  error('Variable wheelbase > 0 in a call to cobblestone.');
end
if X < 0
  error('Variable X must not be negative in a call to cobblestone.');
end
if V < 0
  error('Variable V must not be negative in a call to cobblestone.');
end

% main body of the function

% quantify the geometry of a cobblestone, dimensions are in feet

length = 0.25;
height = 0.05;
top    = 0.15;

% The total number of cobblestones crossed.  There are 4 cobblestones per 
% foot.  The cobblestone roadway is 4 wheelbases long +/- a cobblestone.

cobblestones = ceil(4 * 4 * wheelbase);  % + with ceil, - with floor

% the front axle

if (X < X_enter) || (X > X_enter + cobblestones/4)
  % front axle is not on the cobblestone roadway
  R_f     = 0;
  R_dot_f = 0;
else
  % front axle is crossing the cobblestone, place X at edge of cobbles
  X_cobble = X - X_enter;
  % determine what fraction of a foot the front axle is located at
  frac_ft   = X_cobble - floor(X_cobble);
  % determine how many cobbles in this foot that have been driven over
  cobbles  = floor(4 * frac_ft);
  % locate the axle position relative to a cobblestone
  X_bump   = frac_ft - length * cobbles;
  [R_f, R_dot_f] = bump(length, height, top, X_bump, V);
end

% the rear axle

if (X < X_enter + wheelbase) || (X > X_enter + wheelbase + cobblestones/4)
  % rear axle is not on the cobblestone roadway
  R_r     = 0;
  R_dot_r = 0;
else
  % rear axle is crossing the cobblestone, place X at edge of cobbles
  X_cobble = X - X_enter - wheelbase;
  % determine what fraction of a foot the rear axle is located at
  frac_ft   = X_cobble - floor(X_cobble);
  % determine how many cobbles in this foot that have been driven over
  cobbles  = floor(4 * frac_ft);
  % locate the axle position relative to a cobblestone
  X_bump   = frac_ft - length * cobbles;
  [R_r, R_dot_r] = bump(length, height, top, X_bump, V);
end

end


