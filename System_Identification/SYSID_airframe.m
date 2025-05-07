clear all;
close all;
clc;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% control variables

dt = 1/40;

dmdc = 0; %% set to 1 to run and plot dmdc, set to 0 to plot input interval
gen = 1;

%%% mode,  0 = roll , 1 = pitch , 2 = heave , 3 = yaw , other = full state

%%%%%%%% defines the interval of interest 
ts_search = 70; % used to find the input to the modes
te_search = 235;

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

%%%%%%%%% lon
 % mode = 4;
 % ts = 143; %125; %145; 
 % te = 147; %130; %151;
 % ts_gen = 195;
 % te_gen = 205;

 % %%%%%%%%%% lat
 mode = 5;
 ts = 335; %165; %170; %180; 
 te = 370; %195; %190; %184;
 ts_gen = 195; 
 te_gen = 200;

% %%%%%%%%%%% roll+pitch + gen
% mode = 6;
% ts = 94;
% te = 100; 
% ts_gen = 188;
% te_gen = 198;

input_shift = -0.004;

plot_true = 1;
use_IMU_for_pqr_not_mocap = 1;
% [time_import,inputs_import,RCIN_import,states_import] = data_import("SID_combined.bin",'5-1-flight_SID_combined.csv',[5,400], 1/40 , 17.45 , plot_true , use_IMU_for_pqr_not_mocap);
% [time_import,inputs_import,RCIN_import,states_import] = data_import2("drone_data_2.bin",'mocap_2.csv', dt , 33.6, [ts_search,te_search]); %[ts_search,te_search]
[time_import,inputs_import,RCIN_import,states_import] = data_import("5-6-SID-roll2.bin",'5-6-SID-roll2.csv',[5,400], 1/40 , 19.44 , plot_true , use_IMU_for_pqr_not_mocap);


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


%% Extract Relevant Data for DMDc or System Identification
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
xOutput_lon = [q_dmdc; u_dmdc; w_dmdc; theta_dmdc];
xOutput_lat = [p_dmdc; r_dmdc; v_dmdc; phi_dmdc];

% xInputs_act = [M1_dmdc;M2_dmdc;M3_dmdc;M4_dmdc];

%% For Generalization Purposes
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

% Define Lateral and Longitudinal inputs and outputs
output_lon = [q_dmdc; u_dmdc; w_dmdc; theta_dmdc]';
output_lat = [p_dmdc; r_dmdc; v_dmdc; phi_dmdc]';

input_lon = [M3_dmdc; M1_dmdc]';
input_lat = [M2_dmdc; M4_dmdc]';

% Three state input and outputs with heave (w) and yaw (r) removed 
output_lon_3 = [q_dmdc; u_dmdc; theta_dmdc]';
output_lat_3 = [p_dmdc; v_dmdc; phi_dmdc]';
output_lat_greg = [v_dmdc; p_dmdc; phi_dmdc]';

input_lon_3 = [M3_dmdc]'; % heave input removed 
input_lat_3 = [M2_dmdc]'; % yaw input removed 
input_lat_greg = [M2_dmdc]';

% If the data is noisy, filter without introducing a time delay and re-define relevant outputs
% (note: the sgolay filter doesn't introduce a time delay)
% Filter state r (lateral)
r_dmdc_filt = sgolayfilt(r_dmdc, 3, 21);
figure;
plot(r_dmdc_filt)
hold on
plot(r_dmdc, 'r--')

% Filter state p (lateral)
p_dmdc_filt = sgolayfilt(p_dmdc, 3, 21);
figure;
plot(p_dmdc_filt)
hold on
plot(p_dmdc, 'r--')

% Filter state v (lateral)
v_dmdc_filt = sgolayfilt(v_dmdc, 3, 21);
figure;
plot(v_dmdc_filt)
hold on
plot(v_dmdc, 'r--')

% Filter state phi (lateral)
phi_dmdc_filt = sgolayfilt(phi_dmdc, 3, 21);
figure;
plot(phi_dmdc_filt)
hold on
plot(phi_dmdc, 'r--')

% Filtered Lateral Outputs
output_lat_filt = [p_dmdc_filt; r_dmdc_filt; v_dmdc_filt; phi_dmdc_filt]';

% Filtered p for 3 state lateral output
output_lat_3_filtp = [p_dmdc_filt; v_dmdc; phi_dmdc]';
output_lat_gregfilt = [v_dmdc_filt; p_dmdc_filt; phi_dmdc_filt]';

% Check r is actually the correct data (ddt of psi)
figure;
psi_ddt = (psi_dmdc(2:end) - psi_dmdc(1:end-1))/dt;
plot(psi_ddt)
hold on
plot(r_dmdc, 'r--')

% Generalization for SYSID (check that the models generalize well)
output_lon_gen = [q_dmdc_gen; u_dmdc_gen; w_dmdc_gen; theta_dmdc_gen]';
output_lat_gen = [p_dmdc_gen; r_dmdc_gen; v_dmdc_gen; phi_dmdc_gen]';

input_lon_gen = [M3_dmdc_gen; M1_dmdc_gen]';
input_lat_gen = [M2_dmdc_gen; M4_dmdc_gen]';

%% Babcock (This paper provided estimates for the lateral and longitudinal dynamics of our specific quadrotor)
W_0 = 0;
g = 9.8;
theta_0 = 0;
U_0 = 0;
M_col = 0;
X_col = 0;
L_ped = 0;
Y_ped = 0;

A_lon_bab = [0.9 10.8 26.6 0;
    -0.08051-W_0 -0.25 0 -g*cos(theta_0);
    U_0 0.3558 0.7 -g*sin(theta_0);
    1 0 0 0];

% delta_lon, delta_col (throttle)
B_lon_bab = [149.8 M_col;
    -7.2 X_col;
    -5.7 -13.8;
    0 0];

% Lateral dynamics (p, r, v, phi)
A_lat_bab = [0 -21.6 -6.7 0;
    0.06 -3.8 0 0;
    0.11*W_0 -U_0 -0.18 g*cos(theta_0);
    1 0 0 0];

% delta_lat, delta_ped
B_lat_bab = [153.4 L_ped;
    -4.0 31.1;
    7.4 Y_ped;
    0 0]; 

%% Use idss to set structural constraints on SYSID linear models 
%% Longitudinal Dynamics
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
% 
% % Plot the output comparison
% y = lsim(c2d(ss(A_est, B_est, C_est , D_est), dt), input_lon, 0:dt:5);
% figure
% plot(0:dt:5, y(:,1))
% hold on
% plot(0:dt:5, q_dmdc, 'r')

%% Lateral dynamics
% % Define initial state-space matrices (placeholders for estimation)
% A = ss_lat2.A; %A_lat_bab; %rand(4);
% B = ss_lat2.B; %B_lat_bab; %rand(4,2);
% C = eye(4); %ss_lat2.C;
% D = zeros(4,2);
% K = ss_lat2.K;
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
% % Fix the last column of A to [0; 0; 9.8; 0]
% sys.Structure.A.Free(:,4) = [false; false; false; false];
% sys.A(:,4) = [0; 0; 9.8; 0];
% 
% % % B matrix constraints
% sys.Structure.B.Free = true(4,2); % Allow estimation
% sys.Structure.B.Value = B; % Retain initial values
% 
% % Fix last row of B to [0 0]
% sys.Structure.B.Free(4,:) = [false false];
% sys.B(4,:) = [0 0];
% 
% % % C matrix constraints
% sys.Structure.C.Free = false(4);
% sys.C = eye(4);
% 
% % Estimate the structured state-space model
% 
% % Define estimation options
% opt = ssestOptions;
% opt.InitializeMethod = 'n4sid';  
% opt.InitialState = 'estimate';  % Initial condition can't be zero (drone hover)?
% opt.Display = 'on';  % Show iteration progress
% %opt.SearchMethod = 'lm';  % Levenberg-Marquardt for better convergence
% opt.SearchMethod = 'lsqnonlin';  % good for nonlin, least squares problems
% opt.SearchOptions.MaxIterations = 1000;  % Increase iterations for better estimates
% %opt.SearchOptions.Tolerance = 1e-6;  % Define a convergence threshold
% 
% data = iddata(output_lat, input_lat, dt);
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
% 
% % % Plot the output comparison
% % y = lsim(c2d(ss(A_est, B_est, C_est , D_est), dt), input_lat, 0:dt:5);
% % figure
% % plot(0:dt:5, y(:,1))
% % hold on
% % plot(0:dt:5, p_dmdc, 'r')

%% Lateral 3 state - constrained dynamics
% Define initial state-space matrices (placeholders for estimation)
A = [-0.821 -0.437 9.8; -2.52 2.20 0; 0 1 0];
B = [0.000205; 0.0184; 0];
C = eye(3); %[0 1 0; 0 0 1];
D = zeros(3,1);
K = ss_greg_lat2.K; %[845.4 3065];
Ts = 0; % Ts = dt, discrete-time system

% Create idss model
sys = idss(A, B, C, D, K, 'Ts', Ts);

% % Unstructured
% sys.Structure.A.Free = true(3);
% sys.Structure.B.Free = true(3,1);
% % C matrix constraints
% sys.Structure.C.Free = false(3);
% sys.C = eye(3);
% % D matrix constraints
% sys.Structure.D.Free = false(size(D)); 
% sys.D = zeros(size(D));

% Structured A and B
% A matrix constraints
sys.Structure.A.Free = true(3); % Allow estimation for all elements
sys.Structure.A.Value = A; % Retain initial values

% Fix the last row of A to [0 1 0]
sys.Structure.A.Free(3,:) = [false false false]; 
sys.A(3,:) = [0 1 0];

% Fix the last column of A to [9.8; 0; 0]
sys.Structure.A.Free(:,3) = [false; false; false];
sys.A(:,3) = [9.8; 0; 0];eig(A_est)

% % B matrix constraints
sys.Structure.B.Free = true(3,1); % Allow estimation
sys.Structure.B.Value = B; % Retain initial values

% Fix last row of B to [0]
sys.Structure.B.Free(3,:) = false;
sys.B(3,:) = 0;

% C matrix constraints
sys.Structure.C.Free = false(3);
sys.C = eye(3);

% Estimate the structured state-space model
% Define estimation options
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

data = iddata(output_lat_gregfilt, input_lat_greg, dt);

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

%% Bode plot of roll input to roll rate (p)

% C to extract second state (p)
C_p = [0 1 0]; % p is the second state
D_p = 0;

% Create new SISO system from roll input to p
sys_p = ss(A_est, B_est, C_p, D_p, Ts);

% Greg's lateral model for comparison
sys_greg_p = ss(A,B,C_p,D_p,Ts);

% [GM1, PM1] = margin(sys_p) % computes the gain and phase margins
% [GM2, PM2] = margin(sys_greg)

% Bode plot
figure;
bode(sys_p); hold on;
bode(sys_greg_p);
grid on;
title('Bode Plot: Roll Input to Roll Rate p');
xlabel('Frequency (rad/s)');
ylabel('Magnitude / Phase');
legend('Model 1', 'Greg');

%% Bode plot from roll input to velocity (v)
% C to extract first state (v)
C_v = [1 0 0]; % v is the first state
D_v = 0;

% Create SISO system for v
sys_v = ss(A_est, B_est, C_v, D_v, Ts);

% Greg's model
sys_greg_v = ss(A,B,C_v,D_v,Ts);

% Bode plot
figure;
bode(sys_v); hold on;
bode(sys_greg_v);
grid on;
title('Bode Plot: Roll Input to Velocity v');
xlabel('Frequency (rad/s)');
ylabel('Magnitude / Phase');
legend('Model 1', 'Greg');

%% Bode plot from roll input to phi
% C to extract third state (phi)
C_phi = [0 0 1]; % phi is the third state
D_phi = 0;

% Create SISO system for phi
sys_phi = ss(A_est, B_est, C_phi, D_phi, Ts);

% Greg's model
sys_greg_phi = ss(A,B,C_phi,D_phi,Ts);

% Bode plot
figure;
bode(sys_phi); hold on;
bode(sys_greg_phi);
grid on;
title('Bode Plot: Roll Input to \phi');
xlabel('Frequency (rad/s)');
ylabel('Magnitude / Phase');
legend('Model 1', 'Greg');

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
% eigs_all = {
%     [-2.803 + 0j, 2.091 + 2.11j, 2.091 - 2.11j];       % Model 1
%     [-0.0724 + 2.376j, -0.0724 - 2.376j, -5.635 + 0j];     % Model 2
%     [-0.218 + 0.7266j, -0.218 - 0.7266j, 0.6375 + 0j];  % Model 3
%     [0.5106 + 1.8692j, 0.5106 - 1.8692j, -1.0537 + 0j];  % Model 4
%     [0.9287 + 0.4978j, 0.9287 - 0.4978j, -0.367 + 0j]  % Model 5
% };

% Flight data 5/6/25
eigs_all = {
    [0.1643 + 1.5416j, 0.1643 - 1.5416j, -2.6757 + 0j];  % Model 1
    [  -0.2428 + 0.4939j, -0.2428 - 0.4939j, 0.5509 + 0j];     % Model 2
    [-2.803 + 0j, 2.091 + 2.11j, 2.091 - 2.11j];  % greg
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


%%
% 
% % Plot the output comparison
% 
% ssD = c2d(ss(A_est, B_est, C_est , D_est), dt, 'foh')
% y = lsim(ssD, input_lat_3, 0:dt:4);
% figure
% plot(0:dt:4, y(:,1))
% hold on
% plot(0:dt:4, p_dmdc, 'r')

%% Longitudinal 3 state - constrained dynamics
% % Define initial state-space matrices (placeholders for estimation)
% A = ss_lon.A; %[-0.821 -0.437 9.8; -2.52 2.20 0; 0 1 0];
% B = ss_lon.B; %[0.000205; 0.0184; 0];
% C = eye(3); %ss_lon.C;
% D = zeros(3,1);
% K = ss_lon.K; %[845.4 3065];
% Ts = 0; % Ts = dt, discrete-time system
% 
% % Create idss model
% sys = idss(A, B, C, D, K, 'Ts', Ts);
% 
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
% % % Fix the last row of C to [0 0 1]
% % sys.Structure.C.Free(2,:) = [false false false]; 
% % sys.C(3,:) = [0 0 1];
% % 
% % % Fix the first row of C to [1; 0; 0]
% % sys.Structure.C.Free(1,:) = [false; false; false];
% % sys.C(:,3) = [1; 0; 0];
% 
% % Estimate the structured state-space model
% 
% % Define estimation options
% opt = ssestOptions;
% opt.InitializeMethod = 'n4sid';  
% opt.InitialState = 'estimate';  % Initial condition can't be zero (drone hover)?
% opt.Display = 'on';  % Show iteration progress
% %opt.SearchMethod = 'lm';  % Levenberg-Marquardt for better convergence
% opt.SearchMethod = 'lsqnonlin';  % good for nonlin, least squares problems
% opt.SearchOptions.MaxIterations = 1000;  % Increase iterations for better estimates
% %opt.SearchOptions.Tolerance = 1e-6;  % Define a convergence threshold
% 
% data = iddata(output_lat_3, input_lat_3, dt);
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
% 
% % Plot the output comparison
% y = lsim(c2d(ss(A_est, B_est, C_est , D_est), dt), input_lat_3, 0:dt:4);
% figure
% plot(0:dt:4, y(:,1))
% hold on
% plot(0:dt:4, q_dmdc, 'r')


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
% Remove non-finite values (NaN, Inf) from the data
data = data(isfinite(data));
[b,a] = butter(order,fc/(fs/2));
filtData = filtfilt(b,a,data);
end
