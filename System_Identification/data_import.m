
function [time,inputs,RCIN,states] = data_import(flight_filename,mocap_filename, time_range , common_time_step , pad_time, plot_flag , IMU_pqr_flag)

% plot_flag = 1; %: set to 1 to plot, set to 0 to disable plot

% this is the range of data that is plotted to help align the data streams 
ts_search = 40;
te_search = 50;

start = time_range(1);
stop = time_range(2);
dt = common_time_step; %sec 
chop_pad = pad_time;

% start = 5;
% stop = 260;
% dt = 1/40; %sec 
% chop_pad = 30;

globalTime = [start: dt: stop].';
timeLength = size(globalTime);

%% Read Mocap Data from CSV
% mocapData = readtable('4_28_flight_2.csv', 'NumHeaderLines', 5);
% mocapData = readtable('04_28_flight_1.csv', 'NumHeaderLines', 5);
mocapData = readtable(mocap_filename, 'NumHeaderLines', 5);

%% Read in flight data
% this creates the table structures from the flight bin file

% arduObj = ardupilotreader("Flight_2.bin");
% arduObj = ardupilotreader("Flight_1.bin");
arduObj = ardupilotreader(flight_filename);

RCIN_data_raw = readMessages(arduObj,'MessageName',{'RCIN'});
RCIN_Data = RCIN_data_raw.MsgData{1,1};

control_data_raw = readMessages(arduObj,'MessageName',{'CTUN'});
control_data = control_data_raw.MsgData{1,1};

IMU_raw = readMessages(arduObj,'MessageName',{'IMU'});
IMU_Data = IMU_raw.MsgData{1,1};


%% Extract mocap position, rotation, and time
% this step pulls the data from the mocap csv

mocapTime = mocapData.Var2;

pos_x =  mocapData.Position_1;
pos_y = mocapData.Position_1;
pos_z = mocapData.Position_2;

roll = deg2rad(mocapData.Rotation);
pitch = deg2rad(mocapData.Rotation_1);
yaw = deg2rad(mocapData.Rotation_2);

% pos_x =  -mocapData.Position_1;
% pos_y = mocapData.Position;
% pos_z = mocapData.Position_2;
% 
% roll = -deg2rad(mocapData.Rotation_1);
% pitch = deg2rad(mocapData.Rotation);
% yaw = deg2rad(mocapData.Rotation_2);

%% Extract flight inputs and IMU data
% this step pulls out the data from the flight bin file

% RC input data ( this is the stick position from the pilot )
time_RCIN = HMS_to_sec( timetable2table( RCIN_Data(:,2) ) );

throttle_in_table = timetable2table(  RCIN_Data(:,2)  );
throttle_in = table2array(  throttle_in_table(:,2)  );

roll_stick_table = timetable2table(  RCIN_Data(:,3)  );
roll_stick = table2array(  roll_stick_table(:,2)  );

pitch_stick_table = timetable2table(  RCIN_Data(:,4)  );
pitch_stick = table2array(  pitch_stick_table(:,2)  );

yaw_stick_table = timetable2table(  RCIN_Data(:,5)  );
yaw_stick = table2array(  yaw_stick_table(:,2)  );


% control inputs, u ( theres are the inputs into the drone-mixer [plant])
control_time = HMS_to_sec( timetable2table( control_data(:,4)) );

thrust_input_table = timetable2table( control_data(:,4)  );
thrust_input = table2array( thrust_input_table(:,2)  );

roll_input_table = timetable2table(  control_data(:,5)  );
roll_input = table2array(  roll_input_table(:,2)  );

pitch_input_table = timetable2table( control_data(:,6)  );
pitch_input = table2array(  pitch_input_table(:,2)  );

yaw_input_table = timetable2table( control_data(:,7)  );
yaw_input = table2array(  yaw_input_table(:,2)  );


% IMU data 
time_IMU = HMS_to_sec( timetable2table(  IMU_Data(:,3)  ) );

IMU_x_table = timetable2table(  IMU_Data(:,3)  );
IMU_x = table2array(  IMU_x_table(:,2)  );

IMU_y_table = timetable2table(  IMU_Data(:,4)  );
IMU_y = table2array(  IMU_y_table(:,2)  );

IMU_z_talbe = timetable2table(  IMU_Data(:,5)  );
IMU_z = table2array(  IMU_z_talbe(:,2)  );


%% Interpolate data
% This step aligns all data with a common 40Hz global time 

% Interpolate position data
position_xyz = interp1(mocapTime, [pos_x,pos_y,pos_z], globalTime, 'linear');

% Interpolate orientation data (Euler angles)
rotation_rpy = unwrap(interp1(mocapTime, [roll,pitch,yaw], globalTime, 'linear'));

% Interpolate inputs thrust_input
thrust_input = interp1(control_time, thrust_input, globalTime, 'linear');
roll_input = interp1(control_time, roll_input, globalTime, 'linear');
pitch_input = interp1(control_time, pitch_input, globalTime, 'linear');
yaw_input = interp1(control_time, yaw_input, globalTime, 'linear');

% Interpolate IMU
IMU_pqr = interp1(time_IMU, [IMU_x,IMU_y,IMU_z], globalTime, 'linear');

%% chop or pad data
% This is the time shift that aligns the mocap data with the drone data.
% Typically the mocap is stated first so the data is chopped down to allign
% with the drone flight data

shift_val = int32(chop_pad/dt);

% chop
position_xyz = cat(1, position_xyz(shift_val:end-1,:) , zeros(shift_val,3));
rotation_rpy = cat(1, rotation_rpy(shift_val:end-1,:) , zeros(shift_val,3));

%pad
% position_xyz = cat(1, zeros(shift_val,3) , position_xyz(1:end-shift_val,:) );
% rotation_rpy = cat(1 , zeros(shift_val,3) , rotation_rpy(1:end-shift_val,:) );


%% differentiate pos and rot
velo_xyz = [zeros(1, 3); diff(position_xyz) ./ dt]; % Linear velocity (vx, vy, vz)
euler_rate = [zeros(1, 3); diff(rotation_rpy) ./ dt]; % Angular velocity (roll rate p, pitch rate q, yaw rate r)


%% convert inertial to body
velo_body_xyz = zeros(timeLength(1), 3);
body_rate_pqr = zeros(timeLength(1), 3);

for i = 1:timeLength(1)

    R = RotationMatrix321(rotation_rpy(i,:));
    R1 = RotationAngularRatesToPQR(rotation_rpy(i,:));

    velo_body_xyz(i,1:3) = R * velo_xyz(i, :).';

    body_rate_pqr(i,1:3) = R1 * euler_rate(i, :).';

end


%% plot the IMU and mocap to allight the data 
if plot_flag == 1 % plot mode
    
    %%%% plot IMU and mocap PQR
    figure(100); set(gcf, 'Color', 'w');
    subplot(3,1,1);  
    plot(globalTime, body_rate_pqr(:,1),'r'); hold on; grid on;
    plot(globalTime, IMU_pqr(:,1),'b'); hold on; grid on;
    ylabel('p', 'FontWeight', 'bold'); 
    xlim([ts_search,te_search]);
    
    subplot(3,1,2); 
    plot(globalTime, body_rate_pqr(:,2),'r'); hold on; grid on;
    plot(globalTime, IMU_pqr(:,2),'b');  hold on; grid on;
    ylabel('q', 'FontWeight', 'bold');
    xlim([ts_search,te_search]);
    
    subplot(3,1,3); 
    plot(globalTime, body_rate_pqr(:,3),'r'); hold on; grid on;
    plot(globalTime, IMU_pqr(:,3),'b');  hold on; grid on;
    ylabel('r', 'FontWeight', 'bold');
    xlabel('time', 'FontWeight', 'bold'); 
    xlim([ts_search,te_search]);
    legend('mocap','IMU');
    

    % %%% plot the control inputs
    % figure(101); set(gcf, 'Color', 'w');
    % 
    % subplot(4,1,1);  
    % plot(globalTime, thrust_input,'r'); hold on; grid on;
    % ylabel('thrust input', 'FontWeight', 'bold'); 
    % 
    % subplot(4,1,2); 
    % plot(globalTime, roll_input,'r'); hold on; grid on;
    % ylabel('roll input', 'FontWeight', 'bold');
    % 
    % subplot(4,1,3); 
    % plot(globalTime, pitch_input,'r'); hold on; grid on;
    % ylabel('pitch input', 'FontWeight', 'bold');   
    % 
    % subplot(4,1,4); 
    % plot(globalTime, yaw_input,'r'); hold on; grid on;
    % ylabel('yaw input', 'FontWeight', 'bold');   
    % xlabel('time', 'FontWeight', 'bold'); 
    % 
    % %%% xyz plot
    % figure(102); set(gcf, 'Color', 'w');
    % 
    % subplot(3,1,1);  
    % plot(globalTime, position_xyz(:,1),'r'); hold on; grid on;
    % ylabel('X (m)', 'FontWeight', 'bold'); 
    % 
    % subplot(3,1,2); 
    % plot(globalTime, position_xyz(:,2),'g'); hold on; grid on;
    % ylabel('Y (m)', 'FontWeight', 'bold');
    % 
    % subplot(3,1,3); 
    % plot(globalTime, position_xyz(:,3),'b'); hold on; grid on;
    % ylabel('Z (m)', 'FontWeight', 'bold');
    % xlabel('time', 'FontWeight', 'bold'); 
    % 
    % %%% xyz velo plot
    % figure(103); set(gcf, 'Color', 'w');
    % 
    % subplot(3,1,1);  
    % plot(globalTime, velo_body_xyz(:,1),'r'); hold on; grid on;
    % ylabel('u (m/s)', 'FontWeight', 'bold'); 
    % 
    % subplot(3,1,2); 
    % plot(globalTime, velo_body_xyz(:,2),'g'); hold on; grid on;
    % ylabel('v (m/s)', 'FontWeight', 'bold');
    % 
    % subplot(3,1,3); 
    % plot(globalTime, velo_body_xyz(:,3),'b'); hold on; grid on;
    % ylabel('w (m/s)', 'FontWeight', 'bold');
    % xlabel('time', 'FontWeight', 'bold'); 
    % 
    % %%% roll , pitch , yaw
    % figure(104); set(gcf, 'Color', 'w');
    % 
    % subplot(3,1,1);  
    % plot(globalTime, rad2deg(rotation_rpy(:,1)),'r'); hold on; grid on;
    % ylabel('roll (°)', 'FontWeight', 'bold'); 
    % 
    % subplot(3,1,2); 
    % plot(globalTime, rad2deg(rotation_rpy(:,2)),'g'); hold on; grid on;
    % ylabel('pitch (°)', 'FontWeight', 'bold');
    % 
    % subplot(3,1,3); 
    % plot(globalTime, rad2deg(rotation_rpy(:,3)),'b'); hold on; grid on;
    % ylabel('yaw (°)', 'FontWeight', 'bold');
    % xlabel('time', 'FontWeight', 'bold'); 
    % 


    %%% roll axis 
    figure(104); set(gcf, 'Color', 'w');
    
    subplot(4,1,1);  
    plot(globalTime, rad2deg(body_rate_pqr(:,1)),'r'); hold on; grid on;
    plot(globalTime, rad2deg(IMU_pqr(:,1)),'b'); hold on; grid on;
    ylabel('p (°/s)', 'FontWeight', 'bold'); 
    xlim([200,250]);
    legend('mocap','IMU')

    subplot(4,1,2);  
    plot(globalTime, rad2deg(rotation_rpy(:,1)),'r'); hold on; grid on;
    ylabel('roll (°)', 'FontWeight', 'bold'); 
    xlim([200,250]);

    subplot(4,1,3); 
    plot(globalTime, velo_body_xyz(:,2),'r'); hold on; grid on;
    ylabel('v (m/s)', 'FontWeight', 'bold');
    xlim([200,250]);

    subplot(4,1,4); 
    plot(globalTime, roll_input,'b'); hold on; grid on;
    ylabel('roll input', 'FontWeight', 'bold');
    xlabel('time', 'FontWeight', 'bold'); 
    xlim([200,250]);

end

time = globalTime;
inputs = [thrust_input, roll_input,pitch_input,yaw_input];

if IMU_pqr_flag == 1
    states = [position_xyz , IMU_pqr , body_rate_pqr , velo_body_xyz];
else
    states = [position_xyz , rotation_rpy , body_rate_pqr , velo_body_xyz];
end

RCIN = [time_RCIN , throttle_in , roll_stick , pitch_stick , yaw_stick];

end

% This function converts the time structure of the flight controller to sec
function time = HMS_to_sec(table)

    time_HMS = table2array(table(:,1));
    [Y, M, D, H, MN, S] = datevec(time_HMS);
    time_offset = H*3600+MN*60+S;
    time = time_offset-time_offset(1);
end

function R = RotationMatrix321(euler_angles)
    % calculates the 3-2-1 rotation matrix (R) given Euler angles [phi; theta; psi]
    % phi   = roll angle (rotation about x-axis)
    % theta = pitch angle (rotation about y-axis)
    % psi   = yaw angle (rotation about z-axis)

    % Extract Euler angles
    phi = euler_angles(1);
    theta = euler_angles(2);
    psi = euler_angles(3);

    % Calculate rotation matrix
    R_phi = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    R_theta = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    R_psi = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];

    % Combine results
    R = R_phi * R_theta * R_psi;
end

function R1 = RotationAngularRatesToPQR(euler_angles)
    % Extract Euler angles
    phi = euler_angles(1);   % Roll angle
    theta = euler_angles(2); % Pitch angle

    R1 = [
            1, 0, -sin(theta);
            0, cos(phi), sin(phi) * cos(theta);
            0, -sin(phi), cos(phi) * cos(theta)
        ];
end