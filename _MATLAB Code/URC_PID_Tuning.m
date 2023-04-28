% URC PID Position Algorithm Tuning

clear; clc; close all;

% Motion Limits
min_angle = -90;
max_angle = 90;

% pulses per rev of motor output shaft * gear ratio
%units_per_rev = 1482.6 * 2.5;
units_per_rev = 1;

% Native Units of Joint
safe_max_speed = 45 / 60 * units_per_rev;
min_unit = min_angle * units_per_rev;
max_unit = max_angle * units_per_rev;

% PID Gains
Kp = 1.2;
Ki = 10;
Kd = 10;

% PID Loop variables
integral = 0;
lastError = 0;

% Time Parameters
n = 50;
loops_per_sec = 50;
dT = 1 / loops_per_sec;

% Initialize vectors for plotting
time = zeros(n,1);
pos = zeros(n,1);
cmdPos = zeros(n,1);
vel = zeros(n,1);
time = zeros(n,1);

% Get user input to determine initial conditions
pos(1) = input("INITIAL POSITION (In degrees): ") / 360 * units_per_rev;
cmdPos(1) = input("INITIAL POSITION (In degrees): ") / 360 * units_per_rev;
vel(1) = input("INITIAL VELOCITY (In degrees/sec): ") / 360 * units_per_rev;

for i = 1:n
    % Calculate difference between desired and current position
    error = cmdPos(i) - pos(i);
    % Calculate area under position curve
    integral = integral + (error + lastError) / 2 * dT / 1000.0;   % Riemann sum integral
    % Calculate derivate of position difference
    dError = (error - lastError) / dT / 1000.0;   % derivative
    % Store position difference for next loop
    lastError = error;
    % Do PID calculation
    PID = (Kp * error) + (Ki * integral) + (Kd * dError);

    % Limit PID output to +100% or -100%
    if abs(PID) > 100
        PID = 100 * PID/abs(PID);
    end
    
    % Calculate new position and velocity
    pos(i+1) = pos(i) + vel(i)*dT;
    
    vel(i+1) = PID/100 * safe_max_speed;
    
    cmdPos(i+1) = cmdPos(i);
    time(i+1) = i/loops_per_sec;
end

% Create window to display graphs
figure;

% Display position vs time
subplot(2,1,1);
plot(time,pos,'--',time,cmdPos);
title("POSITION");
legend("ACTUAL","TARGET");
grid on;

% Display velocity vs time
subplot(2,1,2);
plot(time,vel,"r");
title("VELOCITY");
grid on;
