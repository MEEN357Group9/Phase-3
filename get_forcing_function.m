function [FF, ff_data] = get_forcing_function(t,ff_data)
%This function returns the forcing function of a car given the car data and
%vibration model and updates the data structure to pass back in the next
%call
% Inputs:
%   t                     Time
%   ff_data               Data Structure
%
% Outputs:
%   FF                    Is a matrix containing the appropiate
%                              forcing functions acting on the car
%   ff_data               Updated data structure containing new 
%                              t and X values


if ~isstruct(ff_data) 
    error('ff_data must be a structure')
end


[t,X,V] = ff_data.trajectory(ff_data.t_prev, ff_data.X_prev,...
    (ff_data.t_out - ff_data.t_in)/ff_data.N, ff_data.t_in,...
    ff_data.t_out, ff_data.V_in, ff_data.V_out, ff_data.car);

%Driver
[R_f_d, R_r_d, dRdt_f_d, dRdt_r_d] = ff_data.roadway_d(ff_data.car.chassis.wheelbase/12,...
    ff_data.X_enter_d, X, V);

%Passenger
[R_f_p, R_r_p, dRdt_f_p, dRdt_r_p] =  ff_data.roadway_p(ff_data.car.chassis.wheelbase/12,...
    ff_data.X_enter_p, X, V);


%To get forcing function
switch ff_data.model 
    case 'quarter_car_1_DOF'
        %For quater car 1 DOF
        w=(ff_data.car.chassis.weight+ff_data.car.pilot.weight...
            +ff_data.car.power_plant.weight)/4;
        
        % For front suspension
        LRF = get_leverage_ratio('front', ff_data.car);
        CF = LRF * ff_data.car.suspension_front.c;
    
        % For rear suspension
        LRR = get_leverage_ratio('rear', ff_data.car);
        CR = LRR * ff_data.car.suspension_rear.c;
    
        % Average damping
        c = (CF + CR)/2*12; % gives units of lb/(ft/sec)
    
         % For front suspension
        LRFK = get_leverage_ratio('front', ff_data.car);
        KF = LRFK * ff_data.car.suspension_front.k;
    
        % For rear suspension
        LRRK = get_leverage_ratio('rear', ff_data.car);
        KR = LRRK * ff_data.car.suspension_rear.k;
    
        % Average damping
        k = (KF + KR)/2*12; % gives units of lb/ft
        
        
        %Forcing function 
        FF = (w - c*dRdt_f_d - k*R_f_d);
        
    case 'quarter_car_2_DOF'
        %For quater car 2 DOF
        %This is for the front half of the quarter car. 
        ww=(ff_data.car.wheel_front.weight+ff_data.car.wheel_rear.weight)/2;
        w=(ff_data.car.chassis.weight+ff_data.car.pilot.weight...
            +ff_data.car.power_plant.weight)/4; %One forth the weight of the car in lbf.
        
        % front damping ratio
        c=(ff_data.car.wheel_front.c+ff_data.car.wheel_rear.c)/2*12; % ft
        
        % front spring constant
        k=(ff_data.car.wheel_front.k+ff_data.car.wheel_rear.k)/2*12; % ft

        % forcing function matrix 
        FF=[w;...
           ww-c*dRdt_f_d-k*R_f_d];
    case 'half_car_2_DOF'
        % For half car 2 DOF
        % For driver only
        w = ( ff_data.car.chassis.weight + ff_data.car.pilot.weight + ...
        ff_data.car.power_plant.weight) / 2; % lbf
        
        % front damp
        c1LR = get_leverage_ratio('front', ff_data.car);
        c1 = c1LR * ff_data.car.suspension_front.c * 12; % ft
    
        % rear damp
        c2LR = get_leverage_ratio('rear', ff_data.car);
        c2 = c2LR * ff_data.car.suspension_rear.c * 12; %ft
        
        % For front stiffness
        k1LR = get_leverage_ratio('front', ff_data.car);
        k1 = k1LR * ff_data.car.suspension_front.k * 12; % ft
    
        % For rear stiffness
        k2LR = get_leverage_ratio('rear', ff_data.car);
        k2 = k2LR * ff_data.car.suspension_rear.k * 12; % ft
        
        % getting the lengths 
        l = ff_data.car.chassis.length;
        cg = get_cg(ff_data.car); % ft
        lf = cg; % ft
        lr = l - cg; % ft
        
        % Forcing function matrix
        FF = [ (w - dRdt_f_d*c1 - dRdt_r_d*c2 - R_f_d*k1 - R_r_d*k2);
            (c1*lf*dRdt_f_d - lr*dRdt_r_d*c2 + lf*R_f_d*k1 -lr*R_r_d*k2)];
        
               
    case 'half_car_4_DOF'
        % For Half Car 4 DOF
        % For Driver Only
        w = ( ff_data.car.chassis.weight + ff_data.car.pilot.weight + ...
        ff_data.car.power_plant.weight) / 2; % lbf
        
        % Wheel Weight
        wf = ff_data.car.wheel_front.weight; % lbf
        wr = ff_data.car.wheel_rear.weight; % lbf
        
        % Front Damp
        c1LR = get_leverage_ratio('front', ff_data.car);
        c1 = c1LR * ff_data.car.suspension_front.c * 12; % ft
    
        % Rear Damp
        c2LR = get_leverage_ratio('rear', ff_data.car);
        c2 = c2LR * ff_data.car.suspension_rear.c * 12; %ft
        
        % For Front Stiffness
        k1LR = get_leverage_ratio('front', ff_data.car);
        k1 = k1LR * ff_data.car.suspension_front.k * 12; % ft
    
        % For Rear Stiffness
        k2LR = get_leverage_ratio('rear', ff_data.car);
        k2 = k2LR * ff_data.car.suspension_rear.k * 12; % ft
       
        % Forcing Function Matrix
        FF = [w ; 0 ; (wf - c1*dRdt_f_d - k1*R_f_d) ; (wr - c2*dRdt_r_d - k2*R_r_d)];

end

ff_data.t_prev = t;
ff_data.X_prev = X;

end
