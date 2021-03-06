function [Jx] = get_Jx(FSAE_Race_Car)
%Get_Jx This function takes the center of gravity as well as the data
%structor FSAE_Race_Car and produces the moment of inertia about the Z
%axis.
       %Import:
            %FSAE_Race_Car is a data structor.
        %Output:
            %Jx is the moment of inertia about the Y axis.         
%% Chassis
r_chassie=FSAE_Race_Car.chassis.diameter/24; %Finding the radius of the chassis in feet.
m_chassis=FSAE_Race_Car.chassis.weight/32.174; %Mass of the chassis in slugs.
%l_chassis=FSAE_Race_Car.chassis.length/12; %Length of the chassis in ft.
%(not needed)
d_chassis=FSAE_Race_Car.chassis.cg_Z/12; %Distance to use in the parallel axis theorm in ft.
i_chassis_self=m_chassis*r_chassie^2; %Moment of inertia about its own center of gravity.
i_chassis_cg=i_chassis_self+m_chassis*d_chassis^2; %Moment of inertial about the center of gravity. 
%% Engine 
r_motor=FSAE_Race_Car.power_plant.diameter/24; %radius of the motor in ft.
m_motor=FSAE_Race_Car.power_plant.weight/32.178;% mass of the motor.
d_motor=FSAE_Race_Car.chassis.motor_Z/12; %Distance to be used in parallel axis therom. 
i_motor_self=.4*m_motor*r_motor^2; %Moment of inertia about itsself.
i_motor_cg=i_motor_self+m_motor*d_motor^2; %Moment of inertia about cg. 
%% Driver 
%leg_length=FSAE_Race_Car.pilot.height*.6/12; %the length of the drivers
%legs in ft. (Not needed)
body_length=FSAE_Race_Car.pilot.height*.4/12; %length of the rest of the drivers body in ft. 
leg_weight=FSAE_Race_Car.pilot.weight*.4/32.174; %weight of the drivers legs in slugs.
body_weight=FSAE_Race_Car.pilot.weight*.6/32.174; %weight of the rest of the drivers body in slugs.
r=FSAE_Race_Car.pilot.girth/(2*pi)/12; %determinig the radius in ft.
i_leg_self=1/2*leg_weight*r^2; %moment of inertia about itself.
i_body_self=1/12*body_weight*(3*r^2+body_length^2); %Moment of inertia about its center line.
d_leg=(FSAE_Race_Car.chassis.seat_Z+FSAE_Race_Car.pilot.girth/(2*pi))/12; %distance for legs.
d_body=(FSAE_Race_Car.chassis.seat_Z+FSAE_Race_Car.pilot.girth/(pi)+FSAE_Race_Car.pilot.height/5)/12; %distance for body.
i_leg_cg=i_leg_self+leg_weight*d_leg^2; %Intertia about cg.
i_body_cg=i_body_self+body_weight*d_body^2; %Intertia about cg.
%% Total
Jx=i_chassis_cg+i_motor_cg+i_leg_cg+i_body_cg; %summing all the moments. 
end

