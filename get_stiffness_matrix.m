function K = get_stiffness_matrix( vibration_model, FSAE_Race_Car )
%This function returns the damping matrix of a car given the car data and
%vibration model
% Inputs:
%   vibration_model       Is a string indicating the desired vibration model
%   FSAE_Race_car        Is a structure that has the data for quantifying 
%                                        the mass matrix
%
% Outputs:
%  K                               Is a matrix containing the appropiate
%                                        stiffnes in the appropriate places

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
if strcmp('quarter_car_1_DOF', vibration_model)
    % For quarter car 1 DOF
    % For front suspension
    LRFK = get_leverage_ratio('front', FSAE_Race_Car);
    KF = LRFK * FSAE_Race_Car.suspension_front.k;
    
    % For rear suspension
    LRRK = get_leverage_ratio('rear', FSAE_Race_Car);
    KR = LRRK * FSAE_Race_Car.suspension_rear.k;
    
    % Average damping
    avgK = (KF + KR)/2*12; % gives units of lb/ft
    K =  avgK ;
    
    
elseif strcmp('quarter_car_2_DOF', vibration_model)
    % For quarter car 2 DOF
    % For front suspension
    LRFK = get_leverage_ratio('front', FSAE_Race_Car);
    KF = LRFK * FSAE_Race_Car.suspension_front.k;
    
    % For rear suspension
    LRRK = get_leverage_ratio('rear', FSAE_Race_Car);
    KR = LRRK * FSAE_Race_Car.suspension_rear.k;
    
    % Average damping
    avgK = (KF + KR)/2*12; % gives units of lb/ft 

    % For the wheels
    KW = ( FSAE_Race_Car.wheel_front.k + FSAE_Race_Car.wheel_rear.k )/2*12; % gives units of lb/ft
    
    K = [ avgK -avgK; -avgK avgK+KW];
    
    
elseif strcmp(ad3, vibration_model)
    % For half car 2 DOF
    
    % note: l is full length of chassis; lf is the length from the front of the FSAE racecar to the
    % center of gravity; lr is the length from the rear of the FSAE racecar
    % to the center of gravity
    
    % getting the lengths 
    l = FSAE_Race_Car.chassis.wheelbase/12;
    cg = get_cg(FSAE_Race_Car); % ft
    lf = cg; % ft
    lr = l - cg; % ft
    
    % note: k1 is the front stiffness; k2 is the rear stiffness; kf is the
    % front wheel stiffness; kr is the rear wheel stiffness 
    
    
    % For front stiffness
    k1LR = get_leverage_ratio('front', FSAE_Race_Car);
    k1 = k1LR * FSAE_Race_Car.suspension_front.k * 12; % ft
    
    % For rear stiffness
    k2LR = get_leverage_ratio('rear', FSAE_Race_Car);
    k2 = k2LR * FSAE_Race_Car.suspension_rear.k * 12; % ft
    
    % matrix form
    K = [ k1+k2, -k1*lf+k2*lr;
        -k1*lf+k2*lr, k1*lf^2+k2*lr^2];
    
    
elseif strcmp(ad4, vibration_model)
    % For half car 4 DOF
    
    % note: l is full length of chassis; lf is the length from the front of the FSAE racecar to the
    % center of gravity; lr is the length from the rear of the FSAE racecar
    % to the center of gravity
    
    % getting the lengths 
    l = FSAE_Race_Car.chassis.wheelbase/12;
    cg = get_cg(FSAE_Race_Car); % ft
    lf = cg; % ft
    lr = l - cg; % ft
    
    % note: k1 is the front stiffness; k2 is the rear stiffness; kf is the
    % front wheel stiffness; kr is the rear wheel stiffness 
    
    % For front stiffness
    k1LR = get_leverage_ratio('front', FSAE_Race_Car);
    k1 = k1LR * FSAE_Race_Car.suspension_front.k * 12; % ft
    
    % For rear stiffness
    k2LR = get_leverage_ratio('rear', FSAE_Race_Car);
    k2 = k2LR * FSAE_Race_Car.suspension_rear.k * 12; % ft
    
    % For front wheel stiffness
    kf = FSAE_Race_Car.wheel_front.k * 12; % ft
    
    % For rear wheel stiffness
    kr = FSAE_Race_Car.wheel_rear.k * 12; % ft
    
    % matrix form
    K = [ k1+k2, -k1*lf+k2*lr, -k1, -k2;
        -k1*lf+k2*lr, k1*lf^2+k2*lr^2, k1*lf, -k2*lr;
        -k1, k1*lf, k1+kf, 0;
        -k2, -k2*lr, 0, k2+kr]; % in lb/ft and ft*lb/rad
    
    
elseif strcmp(ad5, vibration_model)
    
    % getting the lengths 
    l = FSAE_Race_Car.chassis.wheelbase/12;
    cg = get_cg(FSAE_Race_Car); % ft
    lf = cg; % ft
    lr = l - cg; % ft
    
    rf = FSAE_Race_Car.chassis.radius_f/12; % ft
    rr = FSAE_Race_Car.chassis.radius_r/12; % ft
    
    k1LR = get_leverage_ratio('front', FSAE_Race_Car); 
    k1 = k1LR * FSAE_Race_Car.suspension_front.k * 12; % ft
    k2 = k1;
    
    k2LR = get_leverage_ratio('rear', FSAE_Race_Car);
    k3 = k2LR * FSAE_Race_Car.suspension_rear.k * 12; % ft
    k4 = k3;
        
    K = [k1+k2+k3+k4, -(k1+k2)*lf+(k3+k4)*lr, -(k1-k2)*rf+(k3-k4)*rr;...
        -(k1+k2)*lf+(k3+k4)*lr, (k1+k2)*lf^2+(k3+k4)*lr^2, (k1-k2)*lf*rf+(k3-k4)*lr*rr;...
        -(k1-k2)*rf+(k3-k4)*rr, (k1-k2)*lf*rf+(k3-k4)*lr*rr, (k1+k2)*rf^2+(k3+k4)*rr^2];
    
elseif strcmp(ad6, vibration_model)
    
    % getting the lengths 
    l = FSAE_Race_Car.chassis.wheelbase/12;
    cg = get_cg(FSAE_Race_Car); % ft
    lf = cg; % ft
    lr = l - cg; % ft
    
    rf = FSAE_Race_Car.chassis.radius_f/12; % ft
    rr = FSAE_Race_Car.chassis.radius_r/12; % ft
    
    k1LR = get_leverage_ratio('front', FSAE_Race_Car);
    k2 = k1LR * FSAE_Race_Car.suspension_front.k * 12; % ft;
    k1 = k2;
    
    k2LR = get_leverage_ratio('rear', FSAE_Race_Car);
    k4 = k2LR * FSAE_Race_Car.suspension_rear.k * 12; % ft;
    k3 = k4;
    
    kdf = FSAE_Race_Car.wheel_front.k * 12; % ft
    kpf = FSAE_Race_Car.wheel_front.k * 12; % ft
    kdr = FSAE_Race_Car.wheel_rear.k * 12; % ft
    kpr = FSAE_Race_Car.wheel_rear.k * 12; % ft
    
    K = [k1+k2+k3+k4, -(k1+k2)*lf+(k3+k4)*lr, -(k1-k2)*rf+(k3-k4)*rr, -k1, -k2, -k3, -k4;...
        -(k1+k2)*lf+(k3+k4)*lr, (k1+k2)*lf^2+(k3+k4)*lr^2, (k1-k2)*lf*rf+(k3-k4)*lr*rr, k1*lf, k2*lf, -k3*lr, -k4*lr;...
        -(k1-k2)*rf+(k3-k4)*rr, (k1-k2)*lf*rf+(k3-k4)*lr*rr, (k1+k2)*rf^2+(k3+k4)*rr^2, k1*rf, -k2*rf, -k3*rr, k4*rr;...
        -k1, k1*lf, k1*rf, k1+kdf, 0, 0, 0;...
        -k2, k2*lf, -k2*rf, 0, k2+kpf, 0, 0;...
        -k3, -k3*lr, -k3*rr, 0, 0, k3+kpr, 0;...
        -k4, -k4*lr, k4*rr, 0, 0, 0, k4+kdr];
        
else 
    error('something went wrong')
end

end

