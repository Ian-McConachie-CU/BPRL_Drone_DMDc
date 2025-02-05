% clear all;
% close all;
% clc;



function [time,inputs,RCIN,states] = data_import(flight_filename,mocap_filename, common_time_step , pad_time)


start = 1; %sec
stop = 320; %sec
% dt = 1/40; %sec
dt = common_time_step; %sec 

% chop_pad = 1.45; %sec
chop_pad = pad_time;

globalTime = [start: dt: stop].';
timeLength = size(globalTime);

%% Read Mocap Data from CSV
% mocapData = readtable('mocap_1.csv', 'NumHeaderLines', 5);
mocapData = readtable(mocap_filename, 'NumHeaderLines', 5);

%% Read in flight data
arduObj = ardupilotreader("drone_data_1.bin");
%arduObj = ardupilotreader(flight_filename);

RCIN_data_raw = readMessages(arduObj,'MessageName',{'RCIN'});
RCIN_Data = RCIN_data_raw.MsgData{1,1};

PIDR_raw = readMessages(arduObj,'MessageName',{'PIDR'});
PIDR_data = PIDR_raw.MsgData{1,1};

PIDP_raw = readMessages(arduObj,'MessageName',{'PIDP'});
PIDP_data = PIDP_raw.MsgData{1,1};

PIDY_raw = readMessages(arduObj,'MessageName',{'PIDY'});
PIDY_data = PIDY_raw.MsgData{1,1};

throttle_out_raw = readMessages(arduObj,'MessageName',{'CTUN'});
throttle_out_data = throttle_out_raw.MsgData{1,1};

IMU_raw = readMessages(arduObj,'MessageName',{'IMU'});
IMU_Data = IMU_raw.MsgData{1,1};


%% Extract mocap position, rotation, and time
mocapTime = mocapData.Var2;

pos_x = mocapData.Position;
pos_y = mocapData.Position_1;
pos_z = mocapData.Position_2;

roll = deg2rad(mocapData.Rotation);
pitch = deg2rad(mocapData.Rotation_1);
yaw = deg2rad(mocapData.Rotation_2);

%% Extract flight inputs and IMU data
throttle_in_table = timetable2table(  RCIN_Data(:,2)  );
throttle_in = table2array(  throttle_in_table(:,2)  );

roll_stick_table = timetable2table(  RCIN_Data(:,3)  );
roll_stick = table2array(  roll_stick_table(:,2)  );

pitch_stick_table = timetable2table(  RCIN_Data(:,4)  );
pitch_stick = table2array(  pitch_stick_table(:,2)  );

yaw_stick_table = timetable2table(  RCIN_Data(:,5)  );
yaw_stick = table2array(  yaw_stick_table(:,2)  );

time_RCIN = HMS_to_sec( timetable2table( RCIN_Data(:,2) ) );


throttle_out_table = timetable2table( throttle_out_data(:,4)  );
throttle_out = table2array(  throttle_out_table(:,2)  );
time_throttle = HMS_to_sec( timetable2table( throttle_out_data(:,4)) );

PIDR_table = timetable2table(  PIDR_data(:,2)  );
PIDR = table2array(  PIDR_table(:,2)  );
time_PIDR = HMS_to_sec( timetable2table( PIDR_data(:,2) ) );

PIDP_table = timetable2table( PIDP_data(:,2)  );
PIDP = table2array(  PIDP_table(:,2)  );
time_PIDP = HMS_to_sec( timetable2table( PIDP_data(:,2) ) );

PIDY_table = timetable2table( PIDY_data(:,2)  );
PIDY = table2array(  PIDY_table(:,2)  );
time_PIDY = HMS_to_sec( timetable2table( PIDY_data(:,2) ) );

IMU_x_table = timetable2table(  IMU_Data(:,3)  );
IMU_x = table2array(  IMU_x_table(:,2)  );

IMU_y_table = timetable2table(  IMU_Data(:,4)  );
IMU_y = table2array(  IMU_y_table(:,2)  );

IMU_z_talbe = timetable2table(  IMU_Data(:,5)  );
IMU_z = table2array(  IMU_z_talbe(:,2)  );

time_IMU = HMS_to_sec( timetable2table(  IMU_Data(:,3)  ) );


% Interpolate position data
position_xyz = interp1(mocapTime, [pos_x,pos_y,pos_z], globalTime, 'linear');

% Interpolate orientation data (Euler angles)
rotation_rpy = unwrap(interp1(mocapTime, [roll,pitch,yaw], globalTime, 'linear'));

% Interpolate inputs 
thrust_input = interp1(time_throttle, throttle_out, globalTime, 'linear');
roll_input = interp1(time_PIDR, PIDR, globalTime, 'linear');
pitch_input = interp1(time_PIDP, PIDP, globalTime, 'linear');
yaw_input = interp1(time_PIDY, PIDY, globalTime, 'linear');

% Interpolate RCIN
RCIN_trpy = interp1(time_RCIN, [throttle_in,roll_stick,pitch_stick,yaw_stick], globalTime, 'linear');

% Interpolate IMU
IMU_pqr = interp1(time_IMU, [IMU_x,IMU_y,IMU_z], globalTime, 'linear');

%% chop or pad data

shift_val = int32(chop_pad/dt);

% chop
% position_xyz = cat(1, position_xyz(chop_pad/dt:end,:) , zeros(chop_pad/dt,3));
% 
% rotation_rpy = cat(1, rotation_rpy(chop_pad/dt:end,:) , zeros(chop_pad/dt,3));

%pad
position_xyz = cat(1, zeros(shift_val,3) , position_xyz(1:end-shift_val,:) );

rotation_rpy = cat(1 , zeros(shift_val,3) , rotation_rpy(1:end-shift_val,:) );


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
ts_search = 50;
te_search = 100;

figure(100); set(gcf, 'Color', 'w');

% subplot(3,1,1);  
% plot(globalTime, body_rate_pqr(:,1),'r'); hold on; grid on;
% plot(globalTime, IMU_pqr(:,1),'b'); hold on; grid on;
% ylabel('p', 'FontWeight', 'bold'); 
% xlim([ts_search,te_search]);
% 
% subplot(3,1,2); 
% plot(globalTime, body_rate_pqr(:,2),'r'); hold on; grid on;
% plot(globalTime, IMU_pqr(:,2),'b');  hold on; grid on;
% ylabel('q', 'FontWeight', 'bold');
% xlim([ts_search,te_search]);
% 
% subplot(3,1,3); 
% plot(globalTime, body_rate_pqr(:,3),'r'); hold on; grid on;
% plot(globalTime, IMU_pqr(:,3),'b');  hold on; grid on;
% ylabel('r', 'FontWeight', 'bold');   
% xlim([ts_search,te_search]);


time = globalTime;
inputs = [thrust_input,roll_input,pitch_input,yaw_input];
RCIN = RCIN_trpy;
states = [position_xyz,rotation_rpy,body_rate_pqr,velo_body_xyz];

end

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