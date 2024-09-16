% Define the initial and final quaternions (viewpoints or object orientations)
q_start = quaternion([1 0 0], 'euler', 'XYZ', 'frame');  % Identity rotation
q_end = quaternion([pi/2 pi/4 0], 'euler', 'XYZ', 'frame');  % Rotation by 90 degrees on X and 45 degrees on Y

% Number of interpolation steps
n_steps = 100;

% SLERP interpolation
t = linspace(0, 1, n_steps);
interp_quats = slerp(q_start, q_end, t);  % Perform SLERP

% Define object to rotate (e.g., a 3D arrow or cube)
[x, y, z] = sphere;  % Simple sphere to represent the object
arrow_length = 2;

% Create figure and plot
figure;
hold on;

% Plot initial and final orientations for reference
plot3([0, arrow_length], [0, 0], [0, 0], 'r', 'LineWidth', 2);  % X-axis in red for initial orientation
plot3([0, 0], [0, arrow_length], [0, 0], 'g', 'LineWidth', 2);  % Y-axis in green for initial orientation
plot3([0, 0], [0, 0], [0, arrow_length], 'b', 'LineWidth', 2);  % Z-axis in blue for initial orientation

% Loop through interpolated quaternions and plot their orientations
for i = 1:n_steps
    % Extract rotation matrix from quaternion
    rotm = rotmat(interp_quats(i), 'frame');
    
    % Apply rotation to the object (e.g., the axes or the sphere)
    x_rot = rotm(1,1)*x + rotm(1,2)*y + rotm(1,3)*z;
    y_rot = rotm(2,1)*x + rotm(2,2)*y + rotm(2,3)*z;
    z_rot = rotm(3,1)*x + rotm(3,2)*y + rotm(3,3)*z;
    
    % Plot rotated object (sphere here)
    surf(x_rot, y_rot, z_rot, 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    
    % Plot intermediate orientation axes
    plot3([0, rotm(1,1)*arrow_length], [0, rotm(2,1)*arrow_length], [0, rotm(3,1)*arrow_length], 'r');
    plot3([0, rotm(1,2)*arrow_length], [0, rotm(2,2)*arrow_length], [0, rotm(3,2)*arrow_length], 'g');
    plot3([0, rotm(1,3)*arrow_length], [0, rotm(2,3)*arrow_length], [0, rotm(3,3)*arrow_length], 'b');
    
    % Pause for animation effect
    pause(0.05);
end

% Labels and grid
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
title('Smooth Transitions Between Viewpoints Using SLERP');
hold off;