function V = velocity_profile(t, t_in, t_out, V_in, V_out)
%
% VELOCITY_PROFILE
%
% Provides a veolcity profile described by a haversine function.
%
% INPUTS   t              time within the interval [t_in, t_out]    [sec]
%          t_in           time when entering the profile            [sec]
%          t_out          time when leaving  the profile            [sec]
%          V_in           velocity when entering the profile     [ft/sec]
%          V_out          velocity when leaving  the profile     [ft/sec]
% OUTPUTS  V              velocity of vehicle at time t          [ft/sec]
%

% verify the input

if ~isscalar(t)
  error('Variable t must be a scalar in call to trajectory.');
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

% main body of the function

V = V_in + ((V_out - V_in) / 2) * (1 - cos(pi*(t - t_in)/(t_out - t_in)));

end