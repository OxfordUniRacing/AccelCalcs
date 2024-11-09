function [time_vec, position_vec, velocity_vec, acceleration_vec, wheel_force_vec, motor_torque_vec, motor_speed_vec, motor_current_vec,electrical_power_vec] = simulate_motor_torque_race_with_output(...
    motor_voltage, motor_resistance, motor_constant_kT, motor_constant_kV, motor_speed_limit, CdA, wheel_diameter, Fx_max, ...
    total_mass, gearbox_ratio, power_limit, current_limit, distance, time_step)
    % Simulates a vehicle drag race with motor dynamics and gearbox effects,
    % storing results for plotting, with a motor current limit.
    %
    % Inputs:
    %   motor_voltage   - Voltage supplied to each motor (V)
    %   motor_resistance - Motor resistance (Ohms)
    %   motor_constant  - Torque/Back-EMF constant (Nm/A or V/rad/s)
    %   CdA             - coefficient of drag * area (m^2) 
    %   wheel_diameter  - Diameter of the wheels (meters)
    %   total_mass      - Vehicle and driver mass (kg)
    %   gearbox_ratio   - Gearbox reduction ratio
    %   power_limit     - Maximum motor power (W)
    %   current_lim
    % it   - Peak motor current (A)
    %   distance        - Drag race distance (meters)
    %   time_step       - Simulation time step (seconds)
    %
    % Outputs:
    %   time_vec        - Time vector (s)
    %   position_vec    - Position vector (m)
    %   velocity_vec    - Velocity vector (m/s)
    %   acceleration_vec - Acceleration vector (m/s^2)
    %   motor_torque_vec - Motor torque vector (Nm)
    %   motor_current_vec - Motor current vector (A)

    addpath("MFeval\MFeval")

    % Initial conditions
    velocity = 0; % m/s
    position = 0; % m
    time = 0; % seconds

    % Wheel radius and effective gearbox reduction
    wheel_radius = wheel_diameter / 2;
    omega_base = motor_voltage/motor_constant_kV * 0.95;


    % Initialize result storage
    time_vec = [];
    position_vec = [];
    velocity_vec = [];
    acceleration_vec = [];
    motor_torque_vec = [];
    motor_current_vec = [];
    motor_speed_vec = [];
    electrical_power_vec = [];
    wheel_force_vec = [];

    % Simulation loop
    while position < distance
        % Motor speed in rad/s
        motor_speed = velocity / wheel_radius * gearbox_ratio;
        
        % enforce motor max speed
        motor_speed = min(motor_speed,motor_speed_limit);

        % Adjust motor constant (k_t and k_v) for field weakening
        if motor_speed > omega_base
            motor_constant_kT_fw = motor_constant_kT * omega_base/ motor_speed;
            motor_constant_kV_fw= motor_constant_kV * omega_base/ motor_speed;
        else
            motor_constant_kT_fw = motor_constant_kT;
            motor_constant_kV_fw= motor_constant_kV;
        end

        % Back EMF (V_back = k_e * omega)
        back_emf = motor_constant_kV_fw * motor_speed;

        % Voltage available for current
        voltage_available = motor_voltage - back_emf;

        % Motor current (I = V / R)
        motor_current = voltage_available / motor_resistance;

        % Limit current to peak rating
        motor_current = min(motor_current, current_limit);

        % Electrical power
        electrical_power = motor_voltage * motor_current;

        % Limit motor power if necessary
        if electrical_power > power_limit
            motor_current = power_limit / motor_voltage; % P = V*I
            electrical_power = motor_voltage * motor_current;
        end

        % Torque at each motor (T = k_t * I)
        motor_torque = motor_constant_kT_fw * motor_current;

        % Total torque at wheels (2 motors and gearbox amplification)
        wheel_torque = motor_torque * gearbox_ratio;

        % Force at the wheels (F = T / r)
        wheel_force = wheel_torque / wheel_radius;
        
        % limit to max force that can be applied through a single tyre
        wheel_force = min(wheel_force, Fx_max);

        % Drag Fd = 0.5 * CdA * V^2
        Fd = 0.5 * CdA * velocity^2;

        % Acceleration (a = F / m) 
        acceleration = (wheel_force * 2 - Fd) / total_mass;

        % Update velocity and position
        velocity = velocity + acceleration * time_step;
        
        %check not exceeding motor max speed
        if velocity / wheel_radius * gearbox_ratio > motor_speed_limit
            velocity = motor_speed_limit * wheel_radius / gearbox_ratio;
        end

        position = position + velocity * time_step;

        % Store results
        time_vec(end+1) = time; %#ok<*AGROW>
        position_vec(end+1) = position;
        velocity_vec(end+1) = velocity;
        acceleration_vec(end+1) = acceleration;
        motor_torque_vec(end+1) = motor_torque;
        motor_speed_vec(end+1) = motor_speed;
        motor_current_vec(end+1) = motor_current;
        electrical_power_vec(end+1) = electrical_power;
        wheel_force_vec(end+1) = wheel_force;
        % Increment time
        time = time + time_step;
    end
end