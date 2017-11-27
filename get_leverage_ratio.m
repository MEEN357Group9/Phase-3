function LR = get_leverage_ratio(front_or_rear, FSAE_Race_Car)
%
% GET_LEVERAGE_RATIO
%
% This function returns an appropriate leverage ratio for either the front 
% or the rear suspension assembly for a specified race car. 
%
% Inputs: 
%   front_or_rear    is a string with addmissible values that includes
%                    'front' and 'rear'
%   FSAE_Race_Car    is a struct containing data for the car and driver
%
% Output: 
%   LR               the leverage ratio 
%
% How the leverage ratio works:
%   K @ axle = leverage ratio * stiffness of the spring
%   C @ axle = leverage ratio * damping of the shock absorber
% In other words, the spring stiffness and damping coefficient must be 
% translated from the shock absorber assemblies out to the vehicle axles by
% multiplying their values by the appropriate leverage ratio.
% 
% For inboard suspensions:
%   LR = ((travel of shock piston) / (travel of wheel axle))^2
%
% For outboard suspensions:
%   LR = cos(rake angle)

% Test for valid inputs.

if ~ischar(front_or_rear)
  error('Argument front_or_rear sent to get_leverage_ratio is not a string.');
end
if ~isstruct(FSAE_Race_Car)
  error('The car sent to get_leverage_ratio was not an FSAE_Race_Car.');
end
if nargin ~= 2
  error('Only 2 input variables are allowed for function get_leverage_ratio.');
end
if nargout ~= 1
  error('Only 1 output variable is allowed for function get_leverage_ratio.');
end

% Compute the leverage ratio.

if strcmp(front_or_rear, 'front') == 1
  if strcmp(FSAE_Race_Car.suspension_front.location, 'inboard') == 1
    LR = (FSAE_Race_Car.suspension_front.travel ...
       /  FSAE_Race_Car.wheel_front.travel)^2;
  elseif strcmp(FSAE_Race_Car.suspension_front.location, 'outboard') == 1
    LR = cos(FSAE_Race_Car.suspension_front.angle * pi / 180);
  else
    error('Inadmissible front suspension shock assembly location.');
  end
elseif strcmp(front_or_rear, 'rear') == 1
  if strcmp(FSAE_Race_Car.suspension_rear.location, 'inboard') == 1
    LR = (FSAE_Race_Car.suspension_rear.travel ...
       /  FSAE_Race_Car.wheel_rear.travel)^2;
  elseif strcmp(FSAE_Race_Car.suspension_rear.location, 'outboard') == 1
    LR = cos(FSAE_Race_Car.suspension_rear.angle * pi / 180);
  else
    error('Inadmissible rear suspension shock assembly location.');
  end
else
  error('Inadmissible string option sent to get_leverage_ratio.');
end

