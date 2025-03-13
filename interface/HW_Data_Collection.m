clc,clear;
% NB: to set up data collection, reset Arduino (button press) or unplug USB port and run this script before the serial
% monitor populates on Arduino.
sysid_array = [];
combined_data = [];
startTime = tic;
i=1;

% Set up communication with both Arduinos
% MotorDualArduino.ino = motor arduino code = COM5 (right port on demo computer, change if needed)
% FrequencyScheduler.ino = sensor arduino code = COM6 (left port on demo computer, change if needed)
% HW_back_and_forth.m = MATLAB-in-the-loop code = on computer
A_motor = serialport('COM5', 115200,"Timeout",15);
A_sensor = serialport('COM6', 115200,"Timeout",15);

% Parameters for chirp signal
f_end = 120; % End frequency (20 Hz) 
f_start = 6; % Start frequency (0.1 Hz) 
T = 10; % time at this frequency 
fs = 1000/2; % Sampling frequency (1000/2 samples per second, 0.02 cycles/second, 50 Hz)
A = 9; % Amplitude of the signal
offset = A + 3; % Offset in the y direction to prevent sticking friction 
t = linspace(0, T, T*fs); 

% Chirp signal
chirp_signal = A*sin(2*pi*((f_start-f_end)/T)*(0.5).*t.*t + f_start*t) + offset;      
while toc(startTime) <500    % Read serial lines to get IMU values
    serial_data = readline(A_sensor);
    % Split incoming serial lines by commas
    split_data = strsplit(serial_data,",");
    hw_command = chirp_signal(i);
    writeline(A_motor, "<" + hw_command + ">")
    encoder_data = readline(A_motor);
    combined_data = [split_data, hw_command];
    % Reform data collection array with split data
    sysid_array = [sysid_array; combined_data];
    i = i+1;
end