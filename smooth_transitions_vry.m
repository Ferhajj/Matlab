% Define the file name for the GIF
gif_filename = 'smooth_transitions_vr_3d.gif';

% Define the initial and final quaternions (viewpoints or object orientations)
q_start = quaternion([1 0 0], 'euler', 'XYZ', 'frame');  % Initial orientation
q_end = quaternion([pi/2 pi/4 pi/6], 'euler', 'XYZ', 'frame');  % Final orientation

% Number of interpolation steps
n_steps = 100;

% SLERP interpolation
t = linspace(0, 1, n_steps);
interp_quats = slerp(q_start, q_end, t);  % Perform SLERP interpolation

% Define object to rotate (e.g., a 3D cube)
cube_points = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1]; % Cube vertices
cube_faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8]; % Cube faces

% Create figure for 3D visualization
figure;
hold on;

% Plot initial orientation of the object (red)
patch('Vertices', cube_points, 'Faces', cube_faces, 'FaceColor', 'r', 'FaceAlpha', 0.3);

% Loop through interpolated quaternions and plot object orientations
for i = 1:n_steps
    % Extract rotation matrix from quaternion
    rotm = rotmat(interp_quats(i), 'frame');
    
    % Apply rotation to the object (rotate cube points)
    rotated_cube = (rotm * cube_points')';
    
    % Plot the rotated cube at each interpolated step
    h = patch('Vertices', rotated_cube, 'Faces', cube_faces, 'FaceColor', 'b', 'FaceAlpha', 0.1);
    
    % Capture the current frame and convert it to an image for the GIF
    frame = getframe(gcf);
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
    
    % Delete previous plot to create a smooth animation
    if i < n_steps
        delete(h);
    end
end

% Plot the final orientation of the object (green)
rotm_final = rotmat(q_end, 'frame');
rotated_cube_final = (rotm_final * cube_points')';
patch('Vertices', rotated_cube_final, 'Faces', cube_faces, 'FaceColor', 'g', 'FaceAlpha', 0.3);

% Customize axes, labels, and view for better visualization
axis([-2 2 -2 2 -2 2]);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Smooth 3D Transitions Between Viewpoints or Objects');
grid on;
view(3);
hold off;
