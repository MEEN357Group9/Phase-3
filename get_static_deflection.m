function z0 = get_static_deflection( vibration_model, FSAE_Race_Car )
%This function returns the damping matrix of a car given the car data and
%vibration model
% Inputs:
%   vibration_model       Is a string indicating the desired vibration model
%   FSAE_Race_car         Is a structure that has the data for quantifying 
%                              the mass matrix
%
% Outputs:
%  C                      Is a matrix containing the appropiate
%                              damping in the appropriate places


% valid inputs
ad1 = 'quarter_car_1_DOF';
ad2 = 'quarter_car_2_DOF';
ad3 = 'half_car_2_DOF';
ad4 = 'half_car_4_DOF';
ad5 = 'full_car_3_DOF';
ad6 = 'full_car_7_DOF';



% input error checking
if ~ischar(vibration_model) 
    % the vibration model input is not a string
    error('vibration_model must be a string');
elseif strcmp(ad1,vibration_model) || strcmp(ad2,vibration_model) ...
        || strcmp(ad3,vibration_model) || strcmp(ad4, vibration_model)...
        || strcmp(ad5,vibration_model) || strcmp(ad6, vibration_model)
    % all clear for run
elseif ~strcmp(ad1,vibration_model) && ~strcmp(ad2,vibration_model) ...
        && ~strcmp(ad3,vibration_model) && ~strcmp(ad4,vibration_model)...
        && ~strcmp(ad5,vibration_model) && ~strcmp(ad6,vibration_model)
    % not the proper string
    error('The string for vibration_model must be either ''quarter_car_1_DOF'', ''quarter_car_2_DOF'', ''half_car_2_DOF'',  ''half_car_4_DOF'', ''full_car_3_DOF'', or ''full_car_7_DOF''. ');
    
elseif ~isstruct(FSAE_Race_Car) 
    error('FSAE_Race_Car must be a structure');
else
end


% actual calculation and composition
if strcmp(ad1, vibration_model)
    % For 1 DOF
    % find the stiffness
    K = get_stiffness_matrix(vibration_model, FSAE_Race_Car);
    % K is in K(1)
    % gives units of lbf/ft
    
    % find the weight (quarter)
    w = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight + ...
        FSAE_Race_Car.power_plant.weight) / 4;
    
    % find z0
    z0 = w/K(1); % gives units of ft
    
    
elseif strcmp(ad2, vibration_model)
    % For 2 DOF
    % find the stiffness
    K = get_stiffness_matrix(vibration_model, FSAE_Race_Car);
    % gives units of lbf/ft
    
    % find the weight (quarter)
    w = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight + ...
        FSAE_Race_Car.power_plant.weight) / 4;
    
    ww = ( FSAE_Race_Car.wheel_front.weight + FSAE_Race_Car.wheel_rear.weight ) / 2;
    
    W = [ w; ww];
    
    % find z0
    z0 = K\W; % gives units of ft
       
elseif strcmp(ad3, vibration_model)
    % For half car 2 DOF
    % find the stiffness
    K = get_stiffness_matrix(vibration_model, FSAE_Race_Car);
    % gives units of lbf/ft
    
    % find the weight (half)
    w = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight + ...
        FSAE_Race_Car.power_plant.weight) / 2; % lbf
    
    W = [ w ; 0 ]; % gives lbf
    
    z0 = K\W; % ft 
    
elseif strcmp(ad4, vibration_model)
    % For half car 4 DOF
    % find the stiffness
    K = get_stiffness_matrix(vibration_model, FSAE_Race_Car);
    % gives units of lbf/ft or ft lbf/rad
    
    % find the weight (half)
    w = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight + ...
        FSAE_Race_Car.power_plant.weight) / 2; % lbf
    
    % front wheel weight
    wf = FSAE_Race_Car.wheel_front.weight; % lbf
    
    % rear wheel weight
    wr = FSAE_Race_Car.wheel_rear.weight; % lbf
    
    W = [ w ; 0 ; wf ; wr ];
    
    z0 = K\W; % ft or rad
    
elseif strcmp(ad5, vibration_model)
    % For full car 3 DOF
    % find the stiffness
    K = get_stiffness_matrix(vibration_model, FSAE_Race_Car);
    % gives units of lbf/ft or ft lbf/rad
    
    % find the weight (half)
    w = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight + ...
        FSAE_Race_Car.power_plant.weight); % lbf
    
    W = [w;0;0];
    
    z0 = K\W;

elseif strcmp(ad6, vibration_model)
    % For full car 3 DOF
    % find the stiffness
    K = get_stiffness_matrix(vibration_model, FSAE_Race_Car);
    % gives units of lbf/ft or ft lbf/rad
    
    % find the weight (half)
    w = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight + ...
        FSAE_Race_Car.power_plant.weight); % lbf
    M = get_mass_matrix(vibration_model, FSAE_Race_Car);
    
    W = [w;0;0;M(4,4)*32.174;M(5,5)*32.174;M(6,6)*32.174;M(7,7)*32.174];
    
    z0 = K\W;
end

end

