clear 
close all 

RPM2RADS = 2*pi/60;

% Motor Parameters
motor_voltage = 110; % Vpp, voltage applied to coils
motor_resistance = 0.0178; % Ohms
motor_constant_kT = 0.2225; % Nm/A 
motor_constant_kV = 1 / (48.1 * RPM2RADS); % convert to V/rad/s
motor_speed_limit = 8000 * RPM2RADS;
motor_power_limit = 40e3; % DC supply power (40 kW) (half of FS 80kW rule)
motor_current_limit = 200; % Peak motor phase current (Arms)
Fx_max = 1100; % from plots of the magic formula for the OUR tyres

% Vehicle Parameters
n_driven_wheels = 2; 
wheel_diameter = 0.522; % meters
vehicle_mass = 300; % kg
driver_mass = 75; % kg
gearbox_ratio = 5; % reduction ratio
CdA = 0.5;

% simulation parameters
distance = 75; % event distance in meters
time_step = 0.01; % Time step in seconds

% Run the simulation
[time_vec, position_vec, velocity_vec, acceleration_vec, wheel_force_vec, motor_torque_vec, motor_speed_vec,motor_current_vec,electrical_power_vec] = ...
    simulate_motor_torque_race_with_output(motor_voltage, motor_resistance, ...
    motor_constant_kT, motor_constant_kV,motor_speed_limit, ...
    CdA, wheel_diameter, Fx_max, vehicle_mass+driver_mass, gearbox_ratio,...
    n_driven_wheels, motor_power_limit, motor_current_limit, distance, time_step);

fprintf('Time to complete the race: %.2f seconds\n', time_vec(end));

% Plot results
figure;
subplot(3,2,1);
plot(time_vec, position_vec, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs. Time');
yticks(0:25:75)
grid on;

subplot(3,2,2);
plot(time_vec, velocity_vec*3.6, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (km/h)');
title('Velocity vs. Time');
grid on;

subplot(3,2,3);
yyaxis left
plot(time_vec, acceleration_vec, 'LineWidth', 1.5);
ylabel('Acceleration (m/s^2)');
yyaxis right
plot(time_vec, wheel_force_vec, 'LineWidth', 1.5);
ylabel('Tyre force (N)');
xlabel('Time (s)');
title('Acceleration vs. Time');
grid on;

subplot(3,2,4);
yyaxis left
plot(time_vec, motor_torque_vec, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Motor Torque (Nm)');
yyaxis right
plot(time_vec, motor_current_vec, 'LineWidth', 1.5);
ylabel('Motor Current (A)');
title('Motor Torque vs. Time');
grid on;

% phase motor current in Arms
subplot(3,2,5);
plot(time_vec, motor_speed_vec, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Motor speed (rad/s)');
title('Motor speed vs. Time');
grid on;

subplot(3,2,6);
plot(time_vec, electrical_power_vec/1000, 'LineWidth', 1.5);
xlabel('Position (m)');
ylabel('Power (kW)');
title('Power vs. Time');
grid on;

savefig("plots")
