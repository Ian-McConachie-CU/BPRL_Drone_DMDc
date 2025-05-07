clear all;
close all;
clc;

%% DATA PRE-PROCESSING AND DATA EXTRATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% control variables
% KEEP THIS SECTION AS IS UNLESS FREQ CHANGES OR YOU WANT TO RUN DMDC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 1/40;

dmdc = 0; %% set to 1 to run and plot dmdc, set to 0 to plot input interval
gen = 1;

%%% mode,  0 = roll , 1 = pitch , 2 = heave , 3 = yaw , other = full state

%%%%%%%% defines the interval of interest 
ts_search = 70; % used to find the input to the modes
te_search = 235;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% THIS FILE ONLY ANALYZES THE LATERAL DYNAMICS OF A QUADROTYOR SO IGNORE MODES 0-3 and 5 AND CHANGE MODE 4 TO THE TIME INTERVAL OF INTEREST TO YOU
% NOTE : Modes 0 -3 were originally used for deriving system dynamics using DMDC (this script still contains the DMDC algorithms but they are commented out for now)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%%%%%%%%% roll + gen
% mode = 0;
% ts = 94;
% te = 100; 
% ts_gen = 188;
% te_gen = 198;

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

%%%%%%%% lon
 mode = 4;
 ts = 143; %125; %145; 
 te = 147; %130; %151;
 ts_gen = 195;
 te_gen = 205;

 % % %%%%%%%%%% lat
 % mode = 5;
 % ts = 200; %165; %170; %180; 
 % te = 248; %195; %190; %184;
 % ts_gen = 195; 
 % te_gen = 200;

input_shift = -0.004;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% THIS SECTION IS HOW YOU IMPORT FLIGHT DATA FOR MODEL ANALYSIS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot_true = 1;
use_IMU_for_pqr_not_mocap = 1;
% [time_import,inputs_import,RCIN_import,states_import] = data_import("SID_combined.bin",'5-1-flight_SID_combined.csv',[5,400], 1/40 , 17.45 , plot_true , use_IMU_for_pqr_not_mocap);
% [time_import,inputs_import,RCIN_import,states_import] = data_import2("drone_data_2.bin",'mocap_2.csv', dt , 33.6, [ts_search,te_search]); %[ts_search,te_search]
[time_import,inputs_import,RCIN_import,states_import] = data_import("5-6-SID-roll2.bin",'5-6-SID-roll2.csv',[5,400], 1/40 , 19.44 , plot_true , use_IMU_for_pqr_not_mocap);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EXTRACT DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% p = states_import(:,1);
% v = states_import(:,2);
% phi = states_import(:,3);

thrust = inputs_import(:,1) ;
roll_torque = inputs_import(:,2);
pitch_torque = inputs_import(:,3);
yaw_torque = inputs_import(:,4);


throttle = RCIN_import(:,1); 
roll_input = RCIN_import(:,2); 
pitch_input = RCIN_import(:,3); 
yaw_input = RCIN_import(:,4);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT DATA PROCESSING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

shift_spot = 100 / dt;

throttle = throttle - throttle(shift_spot);
roll_input = roll_input - roll_input(shift_spot);
pitch_input = pitch_input - pitch_input(shift_spot);
yaw_input = yaw_input - yaw_input(shift_spot);

thrust = thrust - thrust(shift_spot) + input_shift;
% roll_torque = roll_torque - roll_torque(shift_spot);
% pitch_torque = pitch_torque - pitch_torque(shift_spot);
% yaw_torque = yaw_torque - yaw_torque(shift_spot);


%% DEFINE SYSTEM DYNAMICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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


%% Extract Relevant Data for DMDc or System Identification
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% This section is used for running DMDC
xTime = [0: dt: te-ts];

xOutput_roll = [phi_dmdc; p_dmdc ; v_dmdc]; 
xOutput_pitch = [theta_dmdc; q_dmdc ; u_dmdc]; 
xOutput_heave = [ w_dmdc]; 
xOutput_yaw = [r_dmdc]; 
xOutput_roll_pitch = [phi_dmdc; theta_dmdc;p_dmdc;q_dmdc;u_dmdc;v_dmdc]; 
xOutput_lon = [q_dmdc; u_dmdc; w_dmdc; theta_dmdc];
xOutput_lat = [p_dmdc; r_dmdc; v_dmdc; phi_dmdc];

%% For Generalization Purposes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time_start = ts_gen/dt;
time_end = te_gen/dt;

phi_dmdc_gen = state_matrix_X(1, time_start:time_end);
theta_dmdc_gen = state_matrix_X(2, time_start:time_end);
psi_dmdc_gen = state_matrix_X(3, time_start:time_end);
p_dmdc_gen = state_matrix_X(4, time_start:time_end);
q_dmdc_gen = state_matrix_X(5, time_start:time_end);
r_dmdc_gen = state_matrix_X(6, time_start:time_end);
u_dmdc_gen = state_matrix_X(10, time_start:time_end);
v_dmdc_gen = state_matrix_X(11, time_start:time_end);
w_dmdc_gen = state_matrix_X(12, time_start:time_end);

M1_dmdc_gen = actuator_matrix(1, time_start:time_end);
M2_dmdc_gen = actuator_matrix(2, time_start:time_end);
M3_dmdc_gen = actuator_matrix(3, time_start:time_end);
M4_dmdc_gen = actuator_matrix(4, time_start:time_end);

%% For System Identification Toolbox - To verify DMDc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: THIS SECTION DEFINES THE RELEVANT INPUTS AND OUTPUTS FOR THE LATERAL DYNAMICS OF A QUADROTOR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define 4 state Longitudinal inputs and outputs
output_lon = [q_dmdc; u_dmdc; w_dmdc; theta_dmdc]';
input_lon = [M3_dmdc; M1_dmdc]';

% Define 3 state Longitudinal inputs and outputs with heave (w)
output_lon_3 = [q_dmdc; u_dmdc; theta_dmdc]';
input_lon_3 = [M3_dmdc]'; % heave input removed 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If the state data is noisy, filter without introducing a time delay and re-define outputs and inputs
% (note: the sgolay filter doesn't introduce a time delay)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filter state q (longitudinal)
q_dmdc_filt = sgolayfilt(q_dmdc, 3, 21);
figure;
plot(q_dmdc_filt)
hold on
plot(q_dmdc, 'r--')

% Filter state u (longitudinal)
u_dmdc_filt = sgolayfilt(u_dmdc, 3, 21);
figure;
plot(u_dmdc_filt)
hold on
plot(u_dmdc, 'r--')

% Filter state w (longitudinal)
w_dmdc_filt = sgolayfilt(w_dmdc, 3, 21);
figure;
plot(w_dmdc_filt)
hold on
plot(w_dmdc, 'r--')

% Filter state theta (longitudinal)
theta_dmdc_filt = sgolayfilt(theta_dmdc, 3, 21);
figure;
plot(theta_dmdc_filt)
hold on
plot(theta_dmdc, 'r--')

% Filtered 4 state longitudinal outputs
output_lon_filt = [q_dmdc_filt; u_dmdc_filt; w_dmdc_filt; theta_dmdc_filt]';

% Filtered 3 state longitudinal outputs
output_lon_filt3 = [q_dmdc_filt; u_dmdc_filt; theta_dmdc_filt]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: THIS STEP IS FOR GENERALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generalization for SYSID (check that the models generalize well)
output_lon_gen = [q_dmdc_gen; u_dmdc_gen; w_dmdc_gen; theta_dmdc_gen]';
output_lat_gen = [p_dmdc_gen; r_dmdc_gen; v_dmdc_gen; phi_dmdc_gen]';

input_lon_gen = [M3_dmdc_gen; M1_dmdc_gen]';
input_lat_gen = [M2_dmdc_gen; M4_dmdc_gen]';

%% Longitudinal 4 state dynamics (constrained A/B)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Define initial state-space matrices (placeholders for estimation)
% A = A_lon_bab; %ss_lon.A; %rand(4);
% B = B_lon_bab; %ss_lon.B; %rand(4,2);
% C = ss_lon2.C;
% D = zeros(4,2);
% K = ss_lon2.K;
% x0 = zeros(4,1);
% Ts = 0; % Ts = dt, discrete-time system
% 
% % Create idss model
% sys = idss(A, B, C, D, K, 'Ts', Ts);
% 
% % A matrix constraints
% sys.Structure.A.Free = true(4); % Allow estimation for all elements
% sys.Structure.A.Value = A; % Retain initial values
% 
% % Fix the last row of A to [1 0 0 0]
% sys.Structure.A.Free(4,:) = [false false false false]; 
% sys.A(4,:) = [1 0 0 0];
% 
% % Fix the last column of A to [0; -9.8; 0; 0]
% sys.Structure.A.Free(:,4) = [false; false; false; false];
% sys.A(:,4) = [0; -9.8; 0; 0];
% 
% % % B matrix constraints
% sys.Structure.B.Free = true(4,2); % Allow estimation
% sys.Structure.B.Value = B; % Retain initial values
% 
% % Fix last row of B to [0 0]
% sys.Structure.B.Free(4,:) = [false false];
% sys.B(4,:) = [0 0];
% 
% sys.Structure.B.Free(1,2) = false;
% sys.B(1,2) = 0;
% 
% sys.Structure.B.Free(2,2) = false;
% sys.B(2,2) = 0;
% 
% % Estimate the structured state-space model
% 
% % Define estimation options
% opt = ssestOptions;
% opt.InitializeMethod = 'n4sid';
% opt.InitialState = 'estimate';  % Initial condition can't be zero (drone hover)
% opt.Display = 'on';  % Show iteration progress
% %opt.SearchMethod = 'lm';  % Levenberg-Marquardt for better convergence (good for nonlin problems)
% opt.SearchMethod = 'lsqnonlin';  % good for nonlin, least squares problems
% opt.SearchOptions.MaxIterations = 1000;  % Increase iterations for better estimates
% %opt.SearchOptions.Tolerance = 1e-6;  % Define a convergence threshold (use when opt.SearchMethod = 'lm')
% 
% data = iddata(output_lon, input_lon, dt);
% estimated_sys = ssest(data, sys, opt);
% 
% % Display estimated parameters
% disp('Estimated A matrix:');
% disp(estimated_sys.A);
% 
% disp('Estimated B matrix:');
% disp(estimated_sys.B);
% 
% disp('Estimated C matrix:');
% disp(estimated_sys.C);
% 
% % plot results against original data for comparison
% % Extract estimated A and B matrices
% A_est = estimated_sys.A;
% B_est = estimated_sys.B;
% C_est = estimated_sys.C;
% D_est = estimated_sys.D;

%% Longitudinal 3 state dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define initial state-space matrices (placeholders for estimation)
% THIS A,B,C,D,K PROVIDE AN INITIAL GUESS FOR IDSS TO BUILD A MODEL OFF OF

% NOTE: Run the systemidentification toolbox by typing
% 'systemIdentification' into the command window. import input and output
% data into the gui and use the estimation tool to estimated a ss model.
% Once you have the toolbox's model, import that into the workspace and
% that can act as your initial guess as well (use the K from the toolbox
% for K's initial guess)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define initial state-space matrices (placeholders for estimation)
A = ss_lon1.A;
B = ss_lon1.B;
C = eye(3); % 3x3 identity matrix
D = zeros(3,1); % 3x1 zeroes matrix
K = ss_lon1.K;
Ts = 0; % Ts = dt, discrete-time system

% Create idss model
sys = idss(A, B, C, D, K, 'Ts', Ts);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: FOR THIS SECTION, IF YOU WANT A MODEL WHERE C,D IS CONSTAINED TO
% IDENTITY, COMMENT OUT 'STRUCTURED A AND B'. IF YOU WANT A MODEL WHERE
% A,B,C,D ARE ALL CONSTRAINED, COMMENT OUT 'UNSTRUCTURED' SECTION.

% Unstructured
sys.Structure.A.Free = true(3);
sys.Structure.B.Free = true(3,1);
% C matrix constraints
sys.Structure.C.Free = false(3);
sys.C = eye(3);
% D matrix constraints
sys.Structure.D.Free = false(size(D)); 
sys.D = zeros(size(D));

% % Structured A and B
% % A matrix constraints
% sys.Structure.A.Free = true(3); % Allow estimation for all elements
% sys.Structure.A.Value = A; % Retain initial values
% 
% % Fix the last row of A to [1 0 0]
% sys.Structure.A.Free(3,:) = [false false false]; 
% sys.A(3,:) = [1 0 0];
% 
% % Fix the last column of A to [0; -9.8; 0]
% sys.Structure.A.Free(:,3) = [false; false; false];
% sys.A(:,3) = [0; -9.8; 0];
% 
% % % B matrix constraints
% sys.Structure.B.Free = true(3,1); % Allow estimation
% sys.Structure.B.Value = B; % Retain initial values
% 
% % Fix last row of B to [0]
% sys.Structure.B.Free(3,:) = false;
% sys.B(3,:) = 0;
% 
% % C matrix constraints
% sys.Structure.C.Free = false(3);
% sys.C = eye(3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Estimate the structured state-space model
% Define estimation options
% These options can help build a better model than what the toolbox's auto-options are
opt = ssestOptions;
opt.InitializeMethod = 'n4sid';  
opt.InitialState = 'estimate';  % Initial condition can't be zero (drone hover)?
opt.Display = 'on';  % Show iteration progress
opt.SearchMethod = 'lm';  % Levenberg-Marquardt for better convergence %'gn', 'gna', 'lm', 'grad', 'lsqnonlin', 'fmincon'
%opt.SearchMethod = 'lsqnonlin';  % good for nonlin, least squares problems
opt.SearchOptions.MaxIterations = 10000;  % Increase iterations for better estimates
%opt.SearchOptions.Tolerance = 1e-6;  % Define a convergence threshold

% % add options to help with ill-conditioning (high cond #)
% R = 1e-4 * eye(15);  % A small positive value on the diagonal (identity matrix)
% opt.Regularization = struct('Lambda', 1e-1, 'R', R, 'Nominal', 'zero'); %1e-4

data = iddata(output_lon_filt3, input_lon_3, dt);
estimated_sys = ssest(data, sys, opt);

% Display estimated parameters
disp('Estimated A matrix:');
disp(estimated_sys.A);

disp('Estimated B matrix:');
disp(estimated_sys.B);

disp('Estimated C matrix:');
disp(estimated_sys.C);

disp('Estimated D matrix:');
disp(estimated_sys.D);

% plot results against original data for comparison
% Extract estimated A and B matrices
A_est = estimated_sys.A;
B_est = estimated_sys.B;
C_est = estimated_sys.C;
D_est = estimated_sys.D;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% THE NEXT SECTIONS ARE USED FOR MODEL ANALYSIS AND COMPARISON
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Bode plot of roll input to (u)

% C to extract second state (u)
C_u = [0 1 0]; % u is the second state
D_u = 0;

% Create new SISO system from pitch input to u
sys_u = ss(A_est, B_est, C_u, D_u, Ts);

% Bode plot
figure;
margin(sys_u);
grid on;
title('Bode Plot: Pitch Input to state u');
xlabel('Frequency (rad/s)');
ylabel('Magnitude / Phase');

%% Bode plot from roll input to (q)
% C to extract first state (q)
C_q = [1 0 0]; % q is the first state
D_q = 0;

% Create SISO system for v
sys_q = ss(A_est, B_est, C_q, D_q, Ts);

% Bode plot
figure;
margin(sys_q);
grid on;
title('Bode Plot: Pitch Input to state q');
xlabel('Frequency (rad/s)');
ylabel('Magnitude / Phase');

%% Bode plot from roll input to theta
% C to extract third state (theta)
C_theta = [0 0 1]; % theta is the third state
D_theta = 0;

% Create SISO system for phi
sys_theta = ss(A_est, B_est, C_theta, D_theta, Ts);

% Bode plot
figure;
margin(sys_theta);
grid on;
title('Bode Plot: Pitch Input to \theta');
xlabel('Frequency (rad/s)');
ylabel('Magnitude / Phase');

%% observability and controllability
obsv_matrix = obsv(A_est, C_est);
ctrb_matrix = ctrb(A_est, B_est);
rank_obsv = rank(obsv_matrix);
rank_ctrb = rank(ctrb_matrix);
disp(['Observability rank: ', num2str(rank_obsv)]);
disp(['Controllability rank: ', num2str(rank_ctrb)]);

%% Check Condition # of A
cond_A = cond(A_est);
disp(['Condition number of A: ', num2str(cond_A)]);

%% Eigenvalue comparison of models
% Flight data 5/1/25
eigs_all = {
    [-2.803 + 0j, 2.091 + 2.11j, 2.091 - 2.11j];       % Model 1
    [-0.0724 + 2.376j, -0.0724 - 2.376j, -5.635 + 0j];     % Model 2
    [-0.218 + 0.7266j, -0.218 - 0.7266j, 0.6375 + 0j];  % Model 3
    [0.5106 + 1.8692j, 0.5106 - 1.8692j, -1.0537 + 0j];  % Model 4
    [0.9287 + 0.4978j, 0.9287 - 0.4978j, -0.367 + 0j]  % Model 5
};

figure;
hold on; grid on;
xlabel('Real Part'); ylabel('Imaginary Part');
title('Eigenvalue Model Comparison');

colors = lines(length(eigs_all));
for i = 1:length(eigs_all)
    plot(real(eigs_all{i}), imag(eigs_all{i}), 'o', ...
        'Color', colors(i,:), ...
        'MarkerFaceColor', colors(i,:), ...  % Fill color
        'MarkerSize', 8, ...
        'DisplayName', ['Model ' num2str(i)]);
end
xline(0, 'k', 'LineWidth', 1);  % y-axis
yline(0, 'k', 'LineWidth', 1);  % x-axis
legend('show');
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% THIS SECTION IS ONLY RELEVANT FOR RUNNING DMDC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
elseif mode == 4
    xOutput = xOutput_lon;
    xInputs_act = [M3_dmdc; M1_dmdc];
    xInput_gen = [actuator_matrix(3, :); actuator_matrix(1, :)];
elseif mode == 5
    xOutput = xOutput_lat;
    xInputs_act = [M2_dmdc; M4_dmdc];
    xInput_gen = [actuator_matrix(2, :); actuator_matrix(4, :)];
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