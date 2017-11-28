function cg = get_cg(FSAE_Race_Car)
%This function finds the location of the center of gravity of the car along the 
%X axis given the car data
%
% Inputs:
%       FSAE_Race_car        Is a structure that has the data for quantifying 
%                            the mass matrix
%
% Outputs:
%       cg                   Is a scalar value that gives the location of the 
%                            center of gravity measured from the rear axle to 
%                            the front axle 


if ~isstruct(FSAE_Race_Car) %checks to see if input is a struct
    error('FSAE_Race_Car must be a structure')
end

% finds the weight times distance of that weight for each component
mx_chassis = (FSAE_Race_Car.chassis.weight)/32.2*... %weight in slugs
    (FSAE_Race_Car.chassis.cg_X);%distance
mx_motor = (FSAE_Race_Car.power_plant.weight)/32.2*... %weight in slugs
    (FSAE_Race_Car.chassis.motor_X); %distance
mx_pilot_torso = (0.6*FSAE_Race_Car.pilot.weight)/32.2*... %weight in slugs
    (-(FSAE_Race_Car.pilot.girth/(2*pi))+FSAE_Race_Car.chassis.seat_X); %distance
mx_pilot_legs = (0.4*FSAE_Race_Car.pilot.weight)/32.2*... %weight in slugs
    (-0.6*FSAE_Race_Car.pilot.height/2+FSAE_Race_Car.chassis.seat_X); %distance

%takes the sums of mx and the sum of the total weight
mx_sum = mx_chassis + mx_motor + mx_pilot_legs + mx_pilot_torso; %takes the sum of all the weightes times their corresponding distances
m_sum = (FSAE_Race_Car.chassis.weight)/32.2 + (FSAE_Race_Car.power_plant.weight)/32.2 +...
    (0.6*FSAE_Race_Car.pilot.weight)/32.2 + (0.4*FSAE_Race_Car.pilot.weight)/32.2;

%finds the location of the center of gravity for the whole system
cg = (mx_sum/m_sum)/12; %converts answer from in. to ft.

end