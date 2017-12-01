function M = get_mass_matrix( vibration_model, FSAE_Race_Car )
%This function returns the mass matrix of a car given the car data and
%vibration model
% Inputs:
%   vibration_model       Is a string indicating the desired vibration model
%   FSAE_Race_car        Is a structure that has the data for quantifying 
%                                        the mass matrix
%
% Outputs:
%  M                               Is a matrix containing the appropiate
%                                        masses in the appropriate places

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
        && ~strcmp(ad5, vibration_model) && ~strcmp(ad6, vibration_model)
    % not the proper string
    error('The string for vibration_model must be either ''quarter_car_1_DOF'' or  ''quarter_car_2_DOF''.');
    
elseif ~isstruct(FSAE_Race_Car) 
    error('FSAE_Race_Car must be a structure');
else
end

% Determining and producing the matrix
if strcmp(ad1,vibration_model) == 1
    % M matrix for quarter car 1 DOF
    % excludes the mass of the wheel assembly
    % chassis weight plus driver weight plus motor weight divided by 4 for
    % quarter car and then divided by 32.2 for slugs 
    
    m = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight ...
        + FSAE_Race_Car.power_plant.weight ) / 4 / 32.2 ; % slugs
    
    % matrix form 
    M = [m];
    
elseif strcmp(ad2,vibration_model) == 1
    % M matrix for quarter car 2 DOF
    % includes the mass of the wheel assembly
    % chassis weight plus driver weight plus motor weight divided by 4 for
    % quarter car and then divided by 32.2 for slugs 
    
    m = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight ...
        + FSAE_Race_Car.power_plant.weight )/4 /32.2 ; % slugs
    % average of the weight of the tires
    mw = ( FSAE_Race_Car.wheel_front.weight + FSAE_Race_Car.wheel_rear.weight )...
        / 2 / 32.2; % slugs
    
    % matrix form
    M = [m 0; 0 mw];
    
elseif strcmp(ad3, vibration_model) == 1
    % M matrix for half car 2 DOF
    % excludes the mass of the wheel assembly
    % chassis weight plus driver weight plus motor weight divided by 2 for
    % half car and then divided by 32.17 for slugs 
    m = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight ...
        + FSAE_Race_Car.power_plant.weight ) / 2 / 32.2 ; % slugs
    
    % the moment of interia
    J = get_Jy(FSAE_Race_Car)/2; % slug ft^2
    
    % matrix form
    M = [m 0; 0 J];
    
elseif strcmp(ad4, vibration_model) == 1
    % M matrix for half car 4 DOF
    % includes the mass of the wheel assembly but the masses are individual
    % chassis weight plus driver weight plus motor weight divided by 2 for
    % half car and then divided by 32.17 for slugs 
    m = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight ...
        + FSAE_Race_Car.power_plant.weight ) / 2 / 32.2 ; % slugs
    
    % the moment of interia 
    J = get_Jy(FSAE_Race_Car)/2; % slug ft^2 / rad
    
    % front wheel mass 
    mf = ( FSAE_Race_Car.wheel_front.weight ) / 32.2; % slugs
    
    % rear wheel mass
    mr = ( FSAE_Race_Car.wheel_rear.weight ) / 32.2; % slugs
    
    % mass matrix
    M = [m 0 0 0; 0 J 0 0; 0 0 mf 0; 0 0 0 mr];
    
elseif strcmp(ad5,vibration_model)
    % M matrix for the full car 3 DOF
    m = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight ...
        + FSAE_Race_Car.power_plant.weight ) / 32.2 ; % slugs
    % the moment of interia 
    Jy = get_Jy(FSAE_Race_Car); % slug ft^2 / rad
    Jx = get_Jx(FSAE_Race_Car); % slug ft^2 / rad
    
    M = [m 0 0;0 Jy 0;0 0 Jx];
    
elseif strcmp(ad6,vibration_model)
    % M matrix for the full car 7 DOF
    m = ( FSAE_Race_Car.chassis.weight + FSAE_Race_Car.pilot.weight ...
        + FSAE_Race_Car.power_plant.weight ) / 32.2 ; % slugs
    % the moment of interia 
    Jy = get_Jy(FSAE_Race_Car); % slug ft^2 / rad
    Jx = get_Jx(FSAE_Race_Car); % slug ft^2 / rad
    mf = ( FSAE_Race_Car.wheel_front.weight ) / 32.2; % slugs
    mr = ( FSAE_Race_Car.wheel_rear.weight ) / 32.2; % slugs
    
    M = [m 0 0 0 0 0 0;
        0 Jy 0 0 0 0 0;
        0 0 Jx 0 0 0 0;
        0 0 0 mf 0 0 0;
        0 0 0 0 mf 0 0;
        0 0 0 0 0 mr 0;
        0 0 0 0 0 0 mr];
else
    
    error('something went wrong');
end

end
