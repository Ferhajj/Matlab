% Define the file name for the GIF (optional)
gif_filename = 'stickman_arm_rotation.gif';

% Number of interpolation steps
n_steps = 100;

% Define the stickman's body in 3D (head, torso, arms, legs)
stickman_body = [
    0, 0, 1;    % Head top
    0, 0, 0;    % Head bottom (neck)
    0, -1, 0;   % Torso bottom
    -0.5, -2, 0; % Left leg
    0.5, -2, 0;  % Right leg
    -1, 0.5, 0;  % Left arm start (shoulder)
    -1.5, 1.5, 0; % Left arm end (hand)
    1, 0.5, 0;   % Right arm start (shoulder)
    1.5, 1.5, 0; % Right arm end (hand)
];

% Quaternions for the initial and final orientations of the right arm
q_start = quaternion([0 0 0], 'euler', 'XYZ', 'frame');  % No rotation
q_end = quaternion([pi/4 pi/6 pi/3], 'euler', 'XYZ', 'frame');  % Rotation in 3D

% SLERP interpolation
t = linspace(0, 1, n_steps);
interp_quats = slerp(q_start, q_end, t);  % Perform SLERP interpolation

% Initialize figure
figure_handle = figure;
hold on;

% Loop through the frames for interpolation
for i = 1:n_steps
    clf;  % Clear the figure for each frame

    % Plot the stickman body (head, torso, legs)
    plot3([stickman_body(1,1), stickman_body(2,1)], [stickman_body(1,2), stickman_body(2,2)], [stickman_body(1,3), stickman_body(2,3)], 'k', 'LineWidth', 2); % Head
    plot3([stickman_body(2,1), stickman_body(3,1)], [stickman_body(2,2), stickman_body(3,2)], [stickman_body(2,3), stickman_body(3,3)], 'k', 'LineWidth', 2); % Torso
    plot3([stickman_body(3,1), stickman_body(4,1)], [stickman_body(3,2), stickman_body(4,2)], [stickman_body(3,3), stickman_body(4,3)], 'k', 'LineWidth', 2); % Left leg
    plot3([stickman_body(3,1), stickman_body(5,1)], [stickman_body(3,2), stickman_body(5,2)], [stickman_body(3,3), stickman_body(5,3)], 'k', 'LineWidth', 2); % Right leg
    
    % Rotate the right arm using SLERP
    right_arm_start = stickman_body(8,:);  % Shoulder joint (fixed)
    right_arm_end = stickman_body(9,:);    % Hand (end of the arm)
    
    % Convert the arm vector to 3D and apply rotation
    right_arm_vector_3d = right_arm_end - right_arm_start;  % 3D arm vector (X, Y, Z)

    % Extract the rotation matrix from the quaternion
    rotm = rotmat(interp_quats(i), 'frame');
    
    % Apply rotation to the right arm
    rotated_arm_3d = (rotm * right_arm_vector_3d')';  % Rotate the arm vector (now in 3D)
    
    % New arm end position after rotation
    rotated_arm_end = right_arm_start + rotated_arm_3d;  % 3D arm end
    
    % Plot the left arm and right arm (rotating)
    plot3([stickman_body(6,1), stickman_body(7,1)], [stickman_body(6,2), stickman_body(7,2)], [stickman_body(6,3), stickman_body(7,3)], 'k', 'LineWidth', 2); % Left arm (static)
    plot3([right_arm_start(1), rotated_arm_end(1)], [right_arm_start(2), rotated_arm_end(2)], [right_arm_start(3), rotated_arm_end(3)], 'k', 'LineWidth', 2); % Right arm (rotating)

    % Axis settings
    axis([-2.5 2.5 -2.5 2.5 -2.5 2.5]);
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    view(3);  % Set the 3D view
    
    % Capture the current frame for the GIF
    frame = getframe(figure_handle);  % Use figure handle to capture the frame
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write to the GIF file
    if i == 1
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
    
    % Pause for animation effect
    pause(0.05);
end

hold off;