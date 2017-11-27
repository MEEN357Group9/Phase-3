function [t,X,V] = trajectory(t, X, h, t_in, t_out, V_in, V_out, FSAE_Race_Car)
%
% TRAJECTORY
%
% Provides a generic trajectory that accelerates or decelerates a vehicle
% from an input velocity of V_in to an output velocity of V_out over a time 
% interval of t_in to t_out by applying a smooth transitional profile.
%
% INPUTS   t              time at the beginning of integration step [sec]
%          X              front axle location at beginning of  step [ft]
%          h              time step size of integration             [sec]
%          t_in           time when entering the trajectory         [sec]
%          t_out          time when leaving  the trajectory         [sec]
%          V_in           velocity when entering the trajectory     [mph]
%          V_out          velocity when leaving  the trajectory     [mph]
%          FSAE_Race_Car  is the formula SAE race car being driven
% OUTPUTS  t              time at the end of the integration step   [sec]
%          X              front axle location at end of the  step   [ft]
%          V              velocity of vehicle at end of the  step   [ft/s]
%

% verify the input

if ~isscalar(t)
  error('Variable t must be a scalar in call to trajectory.');
end
if ~isscalar(X)
  error('Variable X must be a scalar in call to trajectory.');
end
if ~isscalar(h)
  error('Variable h must be a scalar in call to trajectory.');
end
if ~isscalar(t_in)
  error('Variable t_in must be a scalar in call to trajectory.');
end
if ~isscalar(t_out)
  error('Variable t_out must be a scalar in call to trajectory.');
end
if ~isscalar(V_in)
  error('Variable V_in must be a scalar in call to trajectory.');
end
if ~isscalar(V_out)
  error('Variable V_out must be a scalar in call to trajectory.');
end
if ~isstruct(FSAE_Race_Car)
  error('A data structure for the race car does not exist.');
end
if t_in < 0
  error('Time t_in must not be negative in call to trajectory.');
end
if t_out <= t_in
  error('Time t_out > t_in in calls to trajectory.');
end
if V_in < 0
  error('Velocity V_in must not be negative in call to trajectory.');
end
if V_out < 0
  error('Velocity V_out must not be negative in call to trajectory.');
end

% main body of the function

% test to make sure that the assigned velocity profile is feasible

V_top = FSAE_Race_Car.top_speed;
t_top = FSAE_Race_Car.t2top_speed;

% convert velocities from mi/hr to ft/sec

Vin  = V_in  * 5280 / 3600;
Vout = V_out * 5280 / 3600;
Vtop = V_top * 5280 / 3600;

gravity = 32.174;  % feet / sec^2

% checks made in units of mph

if (V_in < 0) || (V_out < 0) 
  error('Entering and exiting velocities cannot be negative valued.');
end
if (V_in > FSAE_Race_Car.top_speed) || (V_out > FSAE_Race_Car.top_speed)
  error('Velocities cannot exceed the maximum speed of the race car.');
end

% checks made in units of ft/sec

if Vout > Vin 
  if (Vout - Vin) / (t_out - t_in) > (Vtop / t_top) 
    error('Requested acceleration exceeds capability of the race car.');
  end
else
  if (Vout - Vin) / (t_out - t_in) < (-1.4 * gravity)
    error('Requested braking exceeds capability of the race car.');
  end
end

% get velocities at beginning, midpoint and end of the integration step

Vlo  = velocity_profile(t,       t_in, t_out, Vin, Vout);
Vmid = velocity_profile(t + h/2, t_in, t_out, Vin, Vout);
Vhi  = velocity_profile(t + h,   t_in, t_out, Vin, Vout);

% create the output fields

t = t + h;
X = X + h * (Vlo + 4 * Vmid + Vhi) / 6;
V = Vhi;

end
