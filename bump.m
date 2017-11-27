function [R, R_dot] = bump(length, height, top, X, V)
%
% BUMP
%
% Provides the profile for a generic bump.
%
% INPUTS   length  is the overall width of the bump
%          height  is how tall the bump is, negative height gives a hole
%          top     is the width of a flat region on top of the bump
%          X       is the current position along the bump
%          V       is the current speed of the vehicle
% OUTPUTS  R       is the vertical displacement caused by the bump
%          R_dot   is the vertical speed caused by the bump
%

% verify the input

if ~isscalar(length)
  error('Variable length must be a scalar in call to bump.');
end
if ~isscalar(height)
  error('Variable height must be a scalar in call to bump.');
end
if ~isscalar(top)
  error('Variable top must be a scalar in call to bump.');
end
if ~isscalar(X)
  error('Variable X must be a scalar in call to bump.');
end
if ~isscalar(V)
  error('Variable V must be a scalar in call to bump.');
end
if length <= 0
  error('The bump must have a positive length.');
end
if (top < 0) || (top >= length)
  error('The top of the bump must be sized such that 0 <= top < length.');
end

% main body of the function

if (X <= 0) || (X >= length)
  % located either ahead or behind the bump
  R     = 0;
  R_dot = 0;
else
  % locate the position X is at on the bump
  if X < (length - top) / 2 
    phi = 2 * pi * X / (length - top);
  elseif X < (length + top) / 2 
    phi = pi;
  else
    phi = 2 * pi * (X - top) / (length - top);
  end
  % determine the vertical displacement and vertical velocity
  R     = (height / 2) * (1 - cos(phi));
  R_dot = ((pi * height) / (length - top)) * sin(phi) * V;
end

end
