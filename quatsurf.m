% Create a figure window
figure;
hold on;
axis equal;
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Quaternion-based Object Manipulation in Virtual Reality');

% Define a 3D object: Cube vertices
cubeVertices = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];

% Define faces of the cube
cubeFaces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

% Plot the initial cube in 3D
patch('Vertices', cubeVertices, 'Faces', cubeFaces, 'FaceColor', 'red', 'FaceAlpha', 0.2);

% Set up the 3D view
view(3);  % Set the plot to 3D view
axis vis3d;  % Keep the aspect ratio of 3D axes equal when rotating
rotate3d on;  % Enable 3D rotation of the plot

% Define two quaternion rotations (using Aerospace Toolbox)
quat_start = quaternion(1, 0, 0, 0);  % Identity quaternion
quat_end = quaternion(cos(pi/8), sin(pi/8), 0, 0);  % 45-degree rotation around the X-axis

% Define number of steps for the transition
n_steps = 10;
interp_quats = quaternion.empty(n_steps, 0);  % Initialize quaternion array

% Interpolate between start and end quaternion using SLERP
for i = 1:n_steps
    t = (i-1) / (n_steps-1);
    % SLERP interpolation
    interp_quats(i) = slerp(quat_start, quat_end, t); 
end

% Plot the object manipulation and the quaternion surface

% Use a fixed point on the cube for visualization
fixedPoint = cubeVertices(1, :);  % Choose the first vertex of the cube

% Collect transformed points
transformedPoints = zeros(n_steps, 3);

for i = 1:n_steps
    % Convert quaternion to rotation matrix
    rotationMatrix = rotmat(interp_quats(i), 'frame');
    % Apply rotation to the fixed point
    transformedPoints(i,:) = (rotationMatrix * fixedPoint')';
end

% Plot the transformation surface using the motion of the fixed point
plot3(transformedPoints(:,1), transformedPoints(:,2), transformedPoints(:,3), 'b-', 'LineWidth', 2);
scatter3(transformedPoints(:,1), transformedPoints(:,2), transformedPoints(:,3), 50, 'b', 'filled');

% Plot the final cube at its rotated position in 3D
rotationMatrixFinal = rotmat(quat_end, 'frame');
finalCubeVertices = (rotationMatrixFinal * cubeVertices')';
patch('Vertices', finalCubeVertices, 'Faces', cubeFaces, 'FaceColor', 'green', 'FaceAlpha', 0.2);

% Set 3D limits to avoid clipping
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);

% Display
legend('Initial Cube', 'Rotational Path', 'Final Cube');
hold off;

