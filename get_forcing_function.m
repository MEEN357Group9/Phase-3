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

%% Input Error Checking

if ~isstruct(ff_data) 
    error('ff_data must be a structure')
end

%% Roadway Values

[t,X,V] = ff_data.trajectory(ff_data.t_prev, ff_data.X_prev,...
    (ff_data.t_out - ff_data.t_in)/ff_data.N, ff_data.t_in,...
    ff_data.t_out, ff_data.V_in, ff_data.V_out, ff_data.car);

%Driver
[R_df, R_dr, dRdt_df, dRdt_dr] = ff_data.roadway_d(ff_data.car.chassis.wheelbase/12,...
    ff_data.X_enter_d, X, V);

%Passenger
[R_pf, R_pr, dRdt_pf, dRdt_pr] =  ff_data.roadway_p(ff_data.car.chassis.wheelbase/12,...
    ff_data.X_enter_p, X, V);

%% Forcing Function

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
        
        r_reg = (R_df + R_dr) / 2;
        r_dot = (dRdt_df + dRdt_dr) / 2;
        
        %Forcing function 
        FF = (w - c*r_dot - k*r_reg);
        
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

        r_reg = (R_df + R_dr) / 2;
        r_dot = (dRdt_df + dRdt_dr) / 2;
        
        % forcing function matrix 
        FF=[w; ww-c*r_dot-k*r_reg];
        
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
        k3LR = get_leverage_ratio('rear', ff_data.car);
        k2 = k3LR * ff_data.car.suspension_rear.k * 12; % ft
        
        % getting the lengths 
        l = ff_data.car.chassis.wheelbase/12;
        cg = get_cg(ff_data.car); % ft
        lf = cg; % ft
        lr = l - cg; % ft
        
        % Forcing function matrix
        FF = [ (w - dRdt_df*c1 - dRdt_dr*c2 - R_df*k1 - R_dr*k2);
            (c1*lf*dRdt_df - lr*dRdt_dr*c2 + lf*R_df*k1 -lr*R_dr*k2)];
          
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
        k3LR = get_leverage_ratio('rear', ff_data.car);
        k2 = k3LR * ff_data.car.suspension_rear.k * 12; % ft
       
        % Forcing Function Matrix
        FF = [w ; 0 ; (wf - c1*dRdt_df - k1*R_df) ; (wr - c2*dRdt_dr - k2*R_dr)];
    
    case 'full_car_3_DOF'
        % Full car with 3 degrees of freedom
        w = ( ff_data.car.chassis.weight + ff_data.car.pilot.weight + ...
            ff_data.car.power_plant.weight); % lbf
        
        % Front damp
        c1LR = get_leverage_ratio('front', ff_data.car);
        c1 = c1LR * ff_data.car.suspension_front.c*12; % ft
        c2 = c1;
    
        % Rear damp
        c3LR = get_leverage_ratio('rear', ff_data.car);
        c3 = c3LR * ff_data.car.suspension_rear.c*12; %ft
        c4 = c3;
        
        % Front Stiffness
        k1LR = get_leverage_ratio('front', ff_data.car);
        k1 = k1LR * ff_data.car.suspension_front.k*12; % ft
        k2 = k1;
    
        % Rear Stiffness
        k3LR = get_leverage_ratio('rear', ff_data.car);
        k3 = k3LR * ff_data.car.suspension_rear.k*12; % ft
        k4 = k3;
        
        % Car lengths 
        l = ff_data.car.chassis.wheelbase/12;
        cg = get_cg(ff_data.car); % ft
        lf = cg; % ft
        lr = l - cg; % ft
        rf = ff_data.car.chassis.radius_f/12; % ft
        rr = ff_data.car.chassis.radius_r/12; % ft
        
        % Forcing Function Matrix
        FF = [w - c1*dRdt_df - c2*dRdt_pf - c3*dRdt_pr - c4*dRdt_dr ...
            - k1*R_df - k2*R_pf - k3*R_pr - k4*R_dr; ...
            (c1*dRdt_df + c2*dRdt_pf + k1*R_df + k2*R_pf)*lf ...
            - (c3*dRdt_pr + c4*dRdt_dr + k3*R_pr + k4*R_dr)*lr; ...
            (c1*dRdt_df - c2*dRdt_pf + k1*R_df - k2*R_pf)*rf ...
            - (c3*dRdt_pr - c4*dRdt_dr + k3*R_pr - k4*R_dr)*rr];
        
    case 'full_car_7_DOF'
        %For full car with 7 degrees of freedom
        w = ( ff_data.car.chassis.weight + ff_data.car.pilot.weight + ...
            ff_data.car.power_plant.weight); % lbf
        
        % Wheel Weight
        wdf = ff_data.car.wheel_front.weight; % lbf
        wpf = wdf; % lbf
        wdr = ff_data.car.wheel_rear.weight; % lbf
        wpr = wdr; % lbf
    
        % Front Damp
        cdfLR = get_leverage_ratio('front', ff_data.car);
        %cdfLR = 1;
        cdf = cdfLR * ff_data.car.suspension_front.c*12; % ft
        cpf = cdf; % ft
    
        % Rear Damp
        cdrLR = get_leverage_ratio('rear', ff_data.car);
        %cdrLR = 1;
        cdr = cdrLR * ff_data.car.suspension_rear.c*12; %ft
        cpr = cdr; % ft
        
        % For Front Stiffness
        kdfLR = get_leverage_ratio('front', ff_data.car);
        %kdfLR = 1;
        kdf = kdfLR * ff_data.car.suspension_front.k*12; % ft
        kpf = kdf; % ft
    
        % For Rear Stiffness
        kdrLR = get_leverage_ratio('rear', ff_data.car);
        %kdrLR = 1;
        kdr = kdrLR * ff_data.car.suspension_rear.k*12; % ft
        kpr = kdr; % ft
        
        % Forcing Function
        FF = [w; 0; 0; wdf - cdf*dRdt_df - kdf*R_df; ...
            wpf - cpf*dRdt_pf - kpf*R_pf; wpr - cpr*dRdt_pr - kpr*R_pr; ...
            wdr - cdr*dRdt_dr - kdr*R_dr];
        
end

ff_data.t_prev = t;
ff_data.X_prev = X;

end
