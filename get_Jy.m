
function [Jy] = get_Jy(FSAE_Race_Car)
%Get_Jy This function takes the center of gravity as well as the data
%structor FSAE_Race_Car and produces the moment of inertia about the Y
%axis.
       %Import:
            %FSAE_Race_Car is a data structor.
        %Output:
            %Jy is the moment of inertia about the Y axis. 
            
            
%Importing the center of gravity. 
cg=get_cg(FSAE_Race_Car);
%Finding Moment of inertial of the motor


%% Motor
d_motor=FSAE_Race_Car.power_plant.diameter; %diameter of the motor in in.
dis_motor=FSAE_Race_Car.chassis.motor_X/12; %distance the motor is from the center of the car in ft. 
w_motor=FSAE_Race_Car.power_plant.weight;% weight of the motor.
m_motor=w_motor/32.174; %converting from lbf to slug.
r_motor=d_motor/2; %finding the radius.

i_motor_self=2/5*m_motor*r_motor^2; %finding the moment of interial about its own center of mass, units: slugs*in^2

i_motor_self=i_motor_self/(12^2); %converting slug in^2 to slug ft^2

i_motor_cg=i_motor_self+m_motor*(abs(cg-dis_motor))^2; %calculating the moment of intertia about the center of gravity.


%% Driver
leg_length=FSAE_Race_Car.pilot.height*.6/12; %the length of the drivers legs in ft. 
body_length=FSAE_Race_Car.pilot.height*.4/12; %length of the rest of the drivers body in ft. 
leg_weight=FSAE_Race_Car.pilot.weight*.4/32.174; %weight of the drivers legs in slugs.
body_weight=FSAE_Race_Car.pilot.weight*.6/32.174; %weight of the rest of the drivers body in slugs.
leg_displacement=FSAE_Race_Car.chassis.seat_X/12-leg_length/2; %Center line of the legs.
r=FSAE_Race_Car.pilot.girth/(2*pi)/12; %determinig the radius in ft.
i_leg_self=1/12*leg_weight*(3*r^2+leg_length^2); %moment of inertia about itself.
i_leg_cg=i_leg_self+leg_weight*(abs(leg_displacement-cg))^2; %Moment of inertia about center of gravity.
body_displacement=FSAE_Race_Car.chassis.seat_X/12-r; %finding the centerline of the body cylinder.
i_body_self=1/12*body_weight*(3*r^2+body_length^2); %Moment of inertia about its center line.
i_body_cg=i_body_self+body_weight*(abs(body_displacement-cg))^2; %Moment of inertia about the center of gravity.


%% Chassiss
r_chassie=FSAE_Race_Car.chassis.diameter/24; %Finding the radius of the chassis in feet.
m_chassis=FSAE_Race_Car.chassis.weight/32.174; %Mass of the chassis in slugs.
l_chassis=FSAE_Race_Car.chassis.length/12; %Length of the chassis in ft.
cg_chassis=FSAE_Race_Car.chassis.cg_X/12; %Center of gravity in ft.


i_chassis_self=1/12*m_chassis*(6*r_chassie^2+l_chassis^2); %Moment of inertia about its own center of gravity.

i_chassis_cg=i_chassis_self+m_chassis*(abs(cg_chassis-cg))^2; %Moment of inertia about the center of gravity. 


%% Total
Jy=(i_motor_cg+i_leg_cg+i_body_cg+i_chassis_cg); %Summing the moments of inertias. 
end
