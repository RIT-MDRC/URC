clear; clc; close all;

% URC Motor Position Control Algorithm Test
% NOTE: All units are in degrees!!

%% CONSTANTS AND LOOP VARIABLES

% Motor Parameters
vel_Max = 4;
vel_Min = 0.5;
accel = 0.5;

% What is the acceptable error?
delta_pos_CloseEnough = 0.2;

% Number of loops
n = 5000;

% Loops per seconds
loops_per_sec = 100;

% State Variables
pos = zeros(n,1);
vel = zeros(n,1);
%loopCase = zeros(n,1);
time = zeros(n,1);

%% GET USER INPUT

% Initial Conditions
vel(1) = input("INITIAL VELOCITY (In degrees/sec): ");
% Calculate direction of initial speed
initial_direction = vel(1) / abs(vel(1));
% Limit input to maximum speed
if (abs(vel(1)) > vel_Max)
    vel(1) = vel_Max * initial_direction;
    disp(" !!! INITIAL SPEED REDUCED TO MAX OF " + string(initial_direction*vel_Max) + " !!!");
end
pos(1) = input("INITIAL POSITION (In degrees): ");

% Desired Final Position
pos_Final = input("TARGET POSITION (In degrees): ");

%% CATCH STAYING STILL, OVERSHOT, AND LIMITED MAX SPEED EDGE CASES

% Determine direction from initial to final pos (defaults to initial vel direction when final pos is same as initial pos)
travel_direction = initial_direction;
if (pos_Final ~= pos(1))
    travel_direction = (pos_Final - pos(1)) / abs(pos_Final - pos(1));
end

% Change in position required for motor to accelerate to and decelerate from max velocity
delta_pos_MaxVel = ( 2*vel_Max^2 - vel(1)^2 ) / (2*accel);

% Change in position required to deccelerate from max velocity (with correction factor)
delta_pos_MaxDeccel = (vel_Max)^2 / (2*accel);

% Total distance from initial and final positions
delta_pos_Total = abs(pos_Final - pos(1));

% Total distance required to stop if max speed not limited
delta_pos_Deccel = (delta_pos_Total + vel(1)^2/(2*accel) ) / 2;

% Total distance required to stop from initial speed
delta_pos_InstantDeccel = vel(1)^2 / (2*accel);

% Default to max speed and no overshot
delta_pos_SafeStop = delta_pos_MaxDeccel;
OVERSHOOTING = false;

% Is motor already in final position and sitting still?
if (vel(1) == 0 && pos_Final == pos(1))
    % Initial pos = final pos and initial velocity = 0
    delta_pos_SafeStop = 0;
    disp(" !!! EDGE CASE: Motor is already in position and sitting still !!!")

% Will motor overshoot position because the initial velocity is too high?
% Only edge case if initial velocity is in travel direction
elseif (delta_pos_Total < delta_pos_InstantDeccel)
    % Stopping distance will be half of overshot distance
    delta_pos_SafeStop = (delta_pos_InstantDeccel - delta_pos_Total) / 2;
    % Invert direction so motor immediately deccelerates
    % If initial vel is opposite to travel direction, direction will be inverted once motor has returned to initial position (i.e. inside loop)
    STARTED_OPPOSITE = true;
    if initial_direction == travel_direction
        travel_direction = -1 * travel_direction;
        STARTED_OPPOSITE = false;
    end
    % Set OVERSHOOTING flag so stopping distance is only detected coming back
    OVERSHOOTING = true;
    disp(" !!! EDGE CASE : Initial velocity is too high, Calculating correction for overshot !!!" + newline)

% Does motor has enough distance to travel to speed up to and down from maximum speed
elseif (delta_pos_Total < delta_pos_MaxVel)
    % Stopping distance from max possible speed will be used
    delta_pos_SafeStop = delta_pos_Deccel;
    disp("!!! EDGE CASE : Cannot reach max speed, Calculating new speed limit !!!" + newline)
end

%% LOOP

for i = 1:n
  % MOVE
  if (abs(vel(i)) >= vel_Min)
    pos(i+1) = pos(i) + vel(i)/loops_per_sec;
  else
    pos(i+1) = pos(i);
  end
  
  % Is it close enough to final position?
  %if pos(i) - pos_Final < 0.5
      % Is is close enough to start slowing down?
      if abs(pos_Final - pos(i)) > delta_pos_SafeStop
          % Apply acceleration
          vel(i+1) = vel(i) + accel/loops_per_sec*travel_direction;
          % Cap speed at MAX speed
          if (abs(vel(i+1)) > vel_Max)
              vel(i+1) = vel_Max*travel_direction;
          end
          % If in the overshot edge case but initial velocity is opposite
          % to travel direction, direction needs to be flipped once motor
          % returns to initial position so it begins deccelerating
          if OVERSHOOTING && STARTED_OPPOSITE && initial_direction == -travel_direction && (pos(i) - pos(1))*travel_direction > 0
              travel_direction = -1 * travel_direction;
          end
      % Is this the overshot edge case? Are we overshooting? Are we still going the wrong way?
      elseif OVERSHOOTING && abs(pos_Final - pos(i)) <= delta_pos_SafeStop && vel(i)/abs(vel(i)) ~= travel_direction
          vel(i+1) = vel(i) + accel/loops_per_sec*travel_direction;
      % Are we within acceptable error?
      %{
      elseif abs(pos_Final - pos(i)) < delta_pos_CloseEnough
          loopCase(i+1) = -1;
          OVERSHOOTING = false; % Otherwise, motor will keep trying to correct and begin oscilating
          vel(i+1) = 0;
      %}
      % Slowing down
      else
          % Apply acceleration in reverse
          vel(i+1) = vel(i) - accel/loops_per_sec*travel_direction;
          % Keep motor from overshooting and reversing direction
          if (vel(i+1)*travel_direction < 0)
              vel(i+1) = 0;
          end
          OVERSHOOTING = false; % Otherwise, motor will keep trying to correct and begin oscilating
      end
  %else
  %    vel(i+1) = 0;
  %end
  
% Increment Timer
time(i+1) = i/loops_per_sec;
end

%% DISPLAY RESULTS

% Calculate error between target and algorithm result
error = pos_Final - pos(n+1);
disp("DISTANCE FROM END POSITION TO TARGET: " + string(error));

% Create a line to represent target position
target = pos_Final * ones(n+1,1);

% Create window to display graphs
figure;

% Display position vs time
subplot(2,1,1);
plot(time,target,'--',time,pos);
title("POSITION");
legend("ACTUAL","TARGET");
grid on;

% Display velocity vs time
subplot(2,1,2);
plot(time,vel,"r");
title("VELOCITY");
grid on;