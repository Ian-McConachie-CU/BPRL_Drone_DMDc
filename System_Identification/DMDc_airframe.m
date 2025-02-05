clear all;
close all;
clc;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% control variables

dt = 1/40;

dmdc = 0; %% set to 1 to run and plot dmdc, set to 0 to plot input interval
gen = 1;

%%% mode,  0 = roll , 1 = pitch , 2 = heave , 3 = yaw , other = full state

%%%%%%%% defines the interval of interest 
ts_search = 50; % used to find the input to the modes
te_search = 320;

%%%%%%%%%%%% roll + gen
mode = 5;
% ts = 94;
% te = 114; 
ts = 178;
te = 240; 
ts_gen = 188;
te_gen = 198;

% %%%%%%%%%% pitch + gen
% mode = 1;
% ts = 205;
% te = 225;
% ts_gen = 230;
% te_gen = 250;

% %%%%%%%%%%% heave + gen
% mode = 2;
% ts = 126; 
% te = 136; 
% ts_gen = 150;
% te_gen = 162;

% %%%%%%%%%% yaw + gen
% mode = 3;
% ts = 150; 
% te = 160; 
% ts_gen = 195;
% te_gen = 205;

input_shift = -0.004;

[time_import,inputs_import,RCIN_import,states_import] = data_import("drone_data_1.bin",'mocap_1.csv', dt , 1.45);

%%%%Extract data
time = time_import - time_import(1);
pos_x = states_import(:,1);
pos_y = states_import(:,2);
pos_z = states_import(:,3);
phi = states_import(:,4);
theta = states_import(:,5);
psi = states_import(:,6);
p = states_import(:,7);
q = states_import(:,8);
r = states_import(:,9);
u = states_import(:,10);
v = states_import(:,11);
w = states_import(:,12);

thrust = inputs_import(:,1) ;
roll_torque = inputs_import(:,2);
pitch_torque = inputs_import(:,3);
yaw_torque = inputs_import(:,4) ;


throttle = RCIN_import(:,1); 
roll_input = RCIN_import(:,2); 
pitch_input = RCIN_import(:,3); 
yaw_input = RCIN_import(:,4);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% input data processing 

shift_spot = 100 / dt;

throttle = throttle - throttle(shift_spot);
roll_input = roll_input - roll_input(shift_spot);
pitch_input = pitch_input - pitch_input(shift_spot);
yaw_input = yaw_input - yaw_input(shift_spot);

thrust = thrust - thrust(shift_spot) + input_shift;
% roll_torque = roll_torque - roll_torque(shift_spot);
% pitch_torque = pitch_torque - pitch_torque(shift_spot);
% yaw_torque = yaw_torque - yaw_torque(shift_spot);
% 


% % % % Filter data
% fs = 1/dt;
% fc = 10;
% order = 10;
% 
% thrust = filterData(thrust,fs,fc,order);
% roll_torque = filterData(roll_torque,fs,fc,order);
% pitch_torque = filterData(pitch_torque,fs,fc,order);
% yaw_torque = filterData(yaw_torque,fs,fc,order);
% 
% 
% % % % % Filter data
% meanval = 3;
% 
% thrust = movmean(thrust,meanval);
% roll_torque = movmean(roll_torque,meanval);
% pitch_torque = movmean(pitch_torque,meanval);
% yaw_torque = movmean(yaw_torque,meanval);




%% Define System States
% Ensure that these states are correctly indexed for your dataset
% 1. phi - pitch angle
% 2. theta - roll angle
% 3. psi - yaw angle
% 4. p - inertial angular x velocity in body frame
% 5. q - inertial angular y velocity in body frame
% 6. r - inertial angular z velocity in body frame
% 7. x - inertial x position (may ignore)
% 8. y - inertial y position (may ignore)
% 9. z - inertial z position
% 10. u - inertial linear x velocity in body frame
% 11. v - inertial linear y velocity in body frame
% 12. w - inertial linear z velocity in body frame

orientationEuler = [phi.';theta.';psi.'];
angularVelocity = [p.';q.';r.'];
position = [pos_x.';pos_y.';pos_z.'];
linearVelocity = [u.';v.';w.'];

state_matrix_X = [orientationEuler; angularVelocity; position; linearVelocity];

actuator_matrix = [thrust.';roll_torque.';pitch_torque.';yaw_torque.'];

control_matrix = [throttle.';roll_input.';pitch_input.';yaw_input.'];


%% Extract Relevant Data for DMDc
time_start = ts/dt;
time_end = te/dt;

phi_dmdc = state_matrix_X(1, time_start:time_end);
theta_dmdc = state_matrix_X(2, time_start:time_end);
psi_dmdc = state_matrix_X(3, time_start:time_end);
p_dmdc = state_matrix_X(4, time_start:time_end);
q_dmdc = state_matrix_X(5, time_start:time_end);
r_dmdc = state_matrix_X(6, time_start:time_end);
u_dmdc = state_matrix_X(10, time_start:time_end);
v_dmdc = state_matrix_X(11, time_start:time_end);
w_dmdc = state_matrix_X(12, time_start:time_end);

M1_dmdc = actuator_matrix(1, time_start:time_end);
M2_dmdc = actuator_matrix(2, time_start:time_end);
M3_dmdc = actuator_matrix(3, time_start:time_end);
M4_dmdc = actuator_matrix(4, time_start:time_end);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xTime = [0: dt: te-ts];

xOutput_roll = [phi_dmdc; p_dmdc ; v_dmdc]; 
xOutput_pitch = [theta_dmdc; q_dmdc ; u_dmdc]; 
xOutput_heave = [ w_dmdc]; 
xOutput_yaw = [r_dmdc]; 
xOutput_roll_pitch = [phi_dmdc; theta_dmdc;p_dmdc;q_dmdc;u_dmdc;v_dmdc]; 

% xInputs_act = [M1_dmdc;M2_dmdc;M3_dmdc;M4_dmdc];


%% DMDc Algorithm 
if mode == 0 % roll mode 
    xOutput = xOutput_roll;
    xInputs_act = M2_dmdc;
    xInput_gen = actuator_matrix(2,:);
elseif mode == 1 % pitch mode 
    xOutput = xOutput_pitch;
    xInputs_act = M3_dmdc;
    xInput_gen = actuator_matrix(3,:);
elseif mode == 2 % heave mode 
    xOutput = xOutput_heave;
    xInputs_act = M1_dmdc;
    xInput_gen = actuator_matrix(1,:);
elseif mode == 3 % yaw mode 
    xOutput = xOutput_yaw;
    xInputs_act = M4_dmdc;
    xInput_gen = actuator_matrix(4,:);
else % full state  
    xOutput = xOutput_roll_pitch;
    xInputs_act = [M2_dmdc;M3_dmdc];
    xInput_gen = actuator_matrix(2:3,:);
end

sys_act = DMDC_alg(xInputs_act, xOutput, dt);   
continuousSys = d2c(sys_act, 'tustin');
y_act = lsim(continuousSys, xInputs_act, xTime, xOutput(:,1)); 


%%%%%%%%%%%Plotting the Estimated vs Ground Truth for Position and Velocity


if dmdc == 0

    figure(1); set(gcf, 'Color', 'w');
    subplot(4,1,1); plot(time, control_matrix(1,:),'k'); ylabel('throttle', 'FontWeight', 'bold'); hold on;grid on; xlim([ts_search,te_search]);
    subplot(4,1,2); plot(time, control_matrix(2,:),'r'); ylabel('roll', 'FontWeight', 'bold'); hold on;grid on; xlim([ts_search,te_search]);
    subplot(4,1,3); plot(time, control_matrix(3,:),'b'); ylabel('pitch', 'FontWeight', 'bold'); hold on;grid on; xlim([ts_search,te_search]);
    subplot(4,1,4); plot(time, control_matrix(4,:),'g'); ylabel('yaw', 'FontWeight', 'bold'); hold on;grid on; xlim([ts_search,te_search]);

    figure(2); set(gcf, 'Color', 'w');
    subplot(4,1,1); plot(time, actuator_matrix(1,:),'k'); ylabel('U heave', 'FontWeight', 'bold'); hold on;grid on; xlim([ts_search,te_search]);
    subplot(4,1,2); plot(time, actuator_matrix(2,:),'r'); ylabel('U roll', 'FontWeight', 'bold'); hold on;grid on; xlim([ts_search,te_search]);ylim([-40,40]);
    subplot(4,1,3); plot(time, actuator_matrix(3,:),'b'); ylabel('U pitch', 'FontWeight', 'bold'); hold on;grid on; xlim([ts_search,te_search]);ylim([-40,40]);
    subplot(4,1,4); plot(time, actuator_matrix(4,:),'g'); ylabel('U yaw', 'FontWeight', 'bold'); hold on;grid on; xlim([ts_search,te_search]);


else

    te_plus = te + 10;

    
    plot1 = Plot_Model(mode , state_matrix_X , xInput_gen , continuousSys , ts , te , dt , 6 , '');
    plot1_in = Plot_inputs(time , control_matrix , actuator_matrix , ts , te , 7 , '');


    if gen == 1
        plot2 = Plot_Model(mode , state_matrix_X , xInput_gen , continuousSys , ts_gen , te_gen , dt , 10 , ' Generation');
        plot2_in = Plot_inputs(time,control_matrix,actuator_matrix, ts_gen , te_gen , 11 , ' Generation');
    end

end

function Plot_in = Plot_inputs(time,control_matrix,actuator_matrix,ts,te,seed , title_input)

    figure(seed); set(gcf, 'Color', 'w');
    sgtitle(strcat("RC Inputs" , title_input),'FontSize', 14);
    subplot(4,1,1); plot(time, control_matrix(1,:),'k'); ylabel('throttle', 'FontWeight', 'bold'); hold on;grid on; xlim([ts,te]);
    subplot(4,1,2); plot(time, control_matrix(2,:),'r'); ylabel('roll', 'FontWeight', 'bold'); hold on;grid on; xlim([ts,te]);
    subplot(4,1,3); plot(time, control_matrix(3,:),'b'); ylabel('pitch', 'FontWeight', 'bold'); hold on;grid on; xlim([ts,te]);
    subplot(4,1,4); plot(time, control_matrix(4,:),'g'); ylabel('yaw', 'FontWeight', 'bold'); hold on;grid on; xlim([ts,te]);


    figure(seed+1); set(gcf, 'Color', 'w'); 
    sgtitle(strcat("Open Loop Inputs" , title_input),'FontSize', 14);

    subplot(4,1,1)
    plot(time, actuator_matrix(1,:), 'k', 'Linewidth', 1.5); grid on;
    ylabel('heave input', 'FontSize', 11); 
    xlim([ts,te]);
    
    subplot(4,1,2)
    plot(time, actuator_matrix(2,:), 'k', 'Linewidth', 1.5); grid on;
    ylabel('roll input', 'FontSize', 11);
    xlim([ts,te]);
    
    subplot(4,1,3)
    plot(time, actuator_matrix(3,:), 'k', 'Linewidth', 1.5); grid on;
    ylabel('pitch input', 'FontSize', 11);
    xlim([ts,te]);

    subplot(4,1,4)
    plot(time, actuator_matrix(4,:), 'k', 'Linewidth', 1.5); grid on;
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('yaw input', 'FontSize', 11);
    xlim([ts,te]);

    Plot_in = seed;
end

function Plot = Plot_Model(mode_type , state_matrix_X , Input , sys , ts , te , dt, seed , title_input)

    Time_gen = [0 : dt: te-ts];
    

    % Heave
    w_ref = state_matrix_X(12, ts / dt: te / dt);
    
    % Roll
    phi_ref = state_matrix_X(1, ts / dt: te / dt);
    p_ref = state_matrix_X(4, ts / dt: te / dt);
    v_ref = state_matrix_X(11, ts / dt: te / dt);
    
    % Pitch
    theta_ref = state_matrix_X(2,  ts / dt: te / dt);
    q_ref = state_matrix_X(5, ts / dt: te / dt);
    u_ref = state_matrix_X(10, ts / dt: te / dt);
    
    % Yaw
    %psi_ref = state_matrix_X(3,ts / dt: te / dt);
    r_ref = state_matrix_X(6, ts / dt: te / dt);

    full_state_ref = [phi_ref;theta_ref;p_ref;q_ref;u_ref;v_ref];

    
    figure(seed); set(gcf, 'Color', 'w'); %set(gcf, 'Position', [500 , 800 , 1200 , 900])

    if mode_type == 0 % roll mode
        
        state_out_ref = [phi_ref; p_ref ; v_ref];
        state_out_gen = lsim(sys, Input(ts/dt:te/dt), Time_gen, state_out_ref(:,1));


        sgtitle(strcat("Roll Mode System",title_input), 'FontWeight', 'bold', 'FontSize', 16);
    
        subplot(3,1,1)
        plot(Time_gen, state_out_ref(3,:).', 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, state_out_gen(:,3), 'r--', 'Linewidth', 1.5); 
        legend("ground truth", "DMDc estimate", 'FontSize', 10);
        title("Linear Velocity (v)", 'FontSize', 12);
        ylabel('v (m/s)', 'FontSize', 11);

        subplot(3,1,2)
        plot(Time_gen, state_out_ref(1,:), 'b', 'Linewidth', 1.5); hold on; grid on;
        plot(Time_gen, state_out_gen(:,1), 'r--', 'Linewidth', 1.5);
        title("Roll Angle (φ)", 'FontSize', 12);
        ylabel('φ (°)', 'FontSize', 11);

        subplot(3,1,3)
        plot(Time_gen, state_out_ref(2,:), 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, state_out_gen(:,2), 'r--', 'Linewidth', 1.5); 
        title("Angular Velocity (p)", 'FontSize', 12);
        xlabel('Time (s)', 'FontSize', 11);
        ylabel('p (°/s)', 'FontSize', 11);


    elseif mode_type == 1 % pitch mode

        state_out_ref = [theta_ref; q_ref ; u_ref];
        state_out_gen = lsim(sys, Input(ts/dt:te/dt), Time_gen, state_out_ref(:,1));


        sgtitle(strcat("Pitch Mode System",title_input), 'FontWeight', 'bold', 'FontSize', 16);

        subplot(3,1,1)
        plot(Time_gen, state_out_ref(3,:), 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, state_out_gen(:,3), 'r--', 'Linewidth', 1.5); 
        legend("ground truth", "DMDc estimate", 'FontSize', 10);
        title("Linear Velocity (u)", 'FontSize', 12);
        ylabel('u (m/s)', 'FontSize', 11);
        
        subplot(3,1,2)
        plot(Time_gen, rad2deg(state_out_ref(1,:)), 'b', 'Linewidth', 1.5); hold on; grid on;
        plot(Time_gen, rad2deg(state_out_gen(:,1)), 'r--', 'Linewidth', 1.5);
        title("Pitch Angle (θ)", 'FontSize', 12);
        ylabel('θ (°)', 'FontSize', 11);
        
        subplot(3,1,3)
        plot(Time_gen, rad2deg(state_out_ref(2,:)), 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, rad2deg(state_out_gen(:,2)), 'r--', 'Linewidth', 1.5); 
        title("Angular Velocity (q)", 'FontSize', 12);
        xlabel('Time (s)', 'FontSize', 11);
        ylabel('q (°/s)', 'FontSize', 11);
        

    elseif mode_type == 2 % heave mode

        state_out_ref = w_ref;
        state_out_gen = lsim(sys, Input(ts/dt:te/dt), Time_gen, state_out_ref(:,1));

        sgtitle(strcat("Heave Mode System",title_input), 'FontWeight', 'bold', 'FontSize', 16);
    
        subplot(1,1,1)
        plot(Time_gen, state_out_ref, 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, state_out_gen(:), 'r--', 'Linewidth', 1.5); 
        legend("ground truth", "DMDc estimate", 'FontSize', 10);
        title("Linear Velocity (w)", 'FontSize', 12);
        xlabel('Time (s)', 'FontSize', 11);
        ylabel('w (m/s)', 'FontSize', 11);
                

    elseif mode_type == 3 % yaw mode 

        state_out_ref = [r_ref]; % psi_ref ( put in first pos)
        state_out_gen = lsim(sys, Input(ts/dt:te/dt), Time_gen, state_out_ref(:,1));

        sgtitle(strcat("Yaw Mode System",title_input), 'FontWeight', 'bold', 'FontSize', 16);

        % subplot(2,1,1)
        % plot(Time_gen, rad2deg(state_out_ref(1,:)), 'b', 'Linewidth', 1.5); hold on; grid on;
        % plot(Time_gen, rad2deg(state_out_gen(:,1)), 'r--', 'Linewidth', 1.5);
        % legend("ground truth", "DMDc estimate", 'FontSize', 10);
        % title("Yaw Angle (ψ)", 'FontSize', 12);
        % ylabel('ψ (°)', 'FontSize', 11);

        subplot(1,1,1)
        plot(Time_gen, rad2deg(state_out_ref(1,:)), 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, rad2deg(state_out_gen(:,1)), 'r--', 'Linewidth', 1.5); 
        title("Angular Velocity (r)", 'FontSize', 12);
        xlabel('Time (s)', 'FontSize', 11);
        ylabel('r (°/s)', 'FontSize', 11);

    else

        state_out_ref = full_state_ref;
        state_out_gen = lsim(sys, Input(:,ts/dt:te/dt), Time_gen, state_out_ref(:,1));

        sgtitle(strcat("Full System",title_input), 'FontWeight', 'bold', 'FontSize', 16);

        subplot(6,1,1)
        plot(Time_gen, state_out_ref(1,:), 'b', 'Linewidth', 1.5); hold on; grid on;
        plot(Time_gen, state_out_gen(:,1), 'r--', 'Linewidth', 1.5);
        title("Roll Angle (φ)", 'FontSize', 12);
        ylabel('φ (°)', 'FontSize', 11);

        subplot(6,1,2)
        plot(Time_gen, rad2deg(state_out_ref(2,:)), 'b', 'Linewidth', 1.5); hold on; grid on;
        plot(Time_gen, rad2deg(state_out_gen(:,2)), 'r--', 'Linewidth', 1.5);
        title("Pitch Angle (θ)", 'FontSize', 12);
        ylabel('θ (°)', 'FontSize', 11);

        subplot(6,1,3)
        plot(Time_gen, state_out_ref(5,:), 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, state_out_gen(:,5), 'r--', 'Linewidth', 1.5); 
        title("Linear Velocity (u)", 'FontSize', 12);
        ylabel('u (m/s)', 'FontSize', 11);
        
        subplot(6,1,4)
        plot(Time_gen, state_out_ref(6,:).', 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, state_out_gen(:,6), 'r--', 'Linewidth', 1.5); 
        title("Linear Velocity (v)", 'FontSize', 12);
        ylabel('v (m/s)', 'FontSize', 11);
        
        subplot(6,1,5)
        plot(Time_gen, state_out_ref(3,:), 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, state_out_gen(:,3), 'r--', 'Linewidth', 1.5); 
        title("Angular Velocity (p)", 'FontSize', 12);
        ylabel('p (°/s)', 'FontSize', 11);
       
        subplot(6,1,6)
        plot(Time_gen, rad2deg(state_out_ref(4,:)), 'b', 'Linewidth', 1.5);hold on; grid on;
        plot(Time_gen, rad2deg(state_out_gen(:,4)), 'r--', 'Linewidth', 1.5); 
        title("Angular Velocity (q)", 'FontSize', 12);
        ylabel('q (°/s)', 'FontSize', 11);


    end

    Plot = mode_type;

end

%% DMDc Function
function sys = DMDC_alg(inputs, states, dt)
    n = size(states,1); % Number of states
    l = size(inputs,1); % Number of inputs
    X = states(:, 1:end-1);
    X_shift = states(:, 2:end);
    gamma = inputs(:, 1:end-1);
    Omega = [X; gamma];
    [u, s, v] = svd(Omega, 'econ');
    u_s = u(1:n, :);
    u_c = u(n+1:end, :);
    Ad = X_shift * v * pinv(s) * u_s';
    Bd = X_shift * v * pinv(s) * u_c';
    sys = ss(Ad, Bd, eye(n), 0, dt);
end

function filtData = filterData(data,fs,fc,order)
% Remove non-finite values (NaN, Inf) from the data
data = data(isfinite(data));
[b,a] = butter(order,fc/(fs/2));
filtData = filtfilt(b,a,data);
end
