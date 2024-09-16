% Define start and end quaternion using Euler angles
q_start = quaternion([0 0 0],'eulerd','ZYX','frame'); % Arm down
q_end = quaternion([90 0 0],'eulerd','ZYX','frame');  % Arm up (rotation around Y-axis)

% Interpolate between quaternions
n_steps = 100;
t = linspace(0, 1, n_steps);
interp_quats = slerp(q_start, q_end, t);

% Create figure for the plot
figure;
hold on;
axis equal;

% Plot the arm at each interpolated quaternion
for i = 1:n_steps
    arm_dir = rotatepoint(interp_quats(i), [1 0 0]); % Assume arm is 1 unit long
    quiver3(0, 0, 0, arm_dir(1), arm_dir(2), arm_dir(3), 'b'); % Plot as quiver
end

% Highlight start and end positions
start_dir = rotatepoint(q_start, [1 0 0]);
end_dir = rotatepoint(q_end, [1 0 0]);
quiver3(0, 0, 0, start_dir(1), start_dir(2), start_dir(3), 'r', 'LineWidth', 2);
quiver3(0, 0, 0, end_dir(1), end_dir(2), end_dir(3), 'g', 'LineWidth', 2);

title('Quaternion Interpolation for Arm Swing');
xlabel('X');
ylabel('Y');
zlabel('Z');