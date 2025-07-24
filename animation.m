N = size(out.pos.signals.values, 1);
safe_dist = 0.3;
safe_dist_z = 0.25;

%% Map
figure;
axis equal;
axis([0 10 0 10 0 4]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(20, 70);
grid on;
hold on;


% Wall colors
floorColor = [0.8 0.8 0.8];     
wallColor = [0.9 0.9 0.9];      
ceilingColor = [0.9 0.9 0.9];   

% External walls
floor = fill3([0 10 10 0], [0 0 10 10], [0 0 0 0], floorColor);
southWall = fill3([0 10 10 0], [0 0 0 0], [0 0 4 4], wallColor,'EdgeColor', 'none');        % Y = 0
northWall = fill3([0 10 10 0], [10 10 10 10], [0 0 4 4], wallColor);                        % Y = 10
westWall = fill3([0 0 0 0], [0 10 10 0], [0 0 4 4], wallColor);                             % X = 0
eastWall = fill3([10 10 10 10], [0 10 10 0], [0 0 4 4], wallColor,'EdgeColor', 'none');     % X = 10
ceiling = fill3([0 10 10 0], [0 0 10 10], [4 4 4 4], ceilingColor,'EdgeColor', 'none');


set(ceiling, 'FaceAlpha', 0.1);     
set(southWall, 'FaceAlpha', 0.1);
set(eastWall, 'FaceAlpha', 0.1);

% The south and east walls, as well as the ceiling, are made
% transparent to ease the visibility of the animation.

% Internal walls
wall1 = fill3([0.06 3 3 0.06], [3 3 3 3], [0.08 0.08 4 4], wallColor);
wall2 = fill3([3 3 3 3], [3 2 2 3], [0.08 0.08 4 4], wallColor);
wall3 = fill3([9.94 7 7 9.94], [7 7 7 7], [0.08 0.08 4 4], wallColor);
wall4 = fill3([7 7 7 7], [7 8 8 7], [0.08 0.08 4 4], wallColor);


% Opening

wall5_left = fill3([0 4 4 0], [5 5 5 5], [0.08 0.08 4 4], wallColor, 'EdgeColor', 'none');
fill3([0 4 4 0], [5 5 5 5], [4 4 4 4], wallColor, 'EdgeColor', 'k');
fill3([0 4 4 0], [5 5 5 5], [0.08 0.08 0.08 0.08], wallColor, 'EdgeColor', 'k');

wall5_right = fill3([6 10 10 6], [5 5 5 5], [0.08 0.08 4 4], wallColor, 'EdgeColor', 'none');
fill3([6 10 10 6], [5 5 5 5], [4 4 4 4], wallColor, 'EdgeColor', 'k');
fill3([6 10 10 6], [5 5 5 5], [0.08 0.08 0.08 0.08], wallColor, 'EdgeColor', 'k');

wall5_top = fill3([4 6 6 4], [5 5 5 5], [3 3 4 4], wallColor, 'EdgeColor', 'none');
fill3([4 6 6 4], [5 5 5 5], [4 4 4 4], wallColor, 'EdgeColor', 'k'); 
fill3([4 6 6 4], [5 5 5 5], [3 3 3 3], wallColor, 'EdgeColor', 'k');


wall5_bot = fill3([4 6 6 4], [5 5 5 5], [0.08 0.08 2 2], wallColor, 'EdgeColor', 'none');
fill3([4 6 6 4], [5 5 5 5], [2 2 2 2], wallColor, 'EdgeColor', 'k');  
fill3([4 6 6 4], [5 5 5 5], [0.08 0.08 0.08 0.08], wallColor, 'EdgeColor', 'k');


% Target position
targetPos = [9.0, 9.0, 1];

% Length of the cross arms
L = 0.5;

% Plot target position
plot3([targetPos(1)-L, targetPos(1)+L], ...
      [targetPos(2),   targetPos(2)], ...
      [targetPos(3),   targetPos(3)], 'r-', 'LineWidth', 3);

plot3([targetPos(1),   targetPos(1)], ...
      [targetPos(2)-L, targetPos(2)+L], ...
      [targetPos(3),   targetPos(3)], 'r-', 'LineWidth', 3);


% Load quadrotor STL
model = stlread('quadrotor_3.stl');
spawnPos = [2, 2, 1.5];
dronePatch = patch('Faces', model.ConnectivityList, ...
                   'Vertices', model.Points - mean(model.Points) + spawnPos, ...
                   'FaceColor', [0.3 0.5 1], 'EdgeColor', 'k');


% Plot trajectory
% plot3(px, py, pz, 'k--', 'LineWidth', 2); % Path
% plot3(px(end), py(end), pz(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End point
% plot3(waypoints(2:end-1,1), waypoints(2:end-1,2), waypoints(2:end-1,3), ...
%     'ro', 'LineWidth', 1, 'MarkerSize', 6, 'MarkerFaceColor', 'r');



%% Animation
pause(3);
for i = 1:N
    pos = out.pos.signals.values(i, :);
    q = squeeze(out.q.signals.values(:,1,i));
    R = quat2rotm(q');

    % STL rotation and translation
    rotated_pts = (R * (model.Points - mean(model.Points))')';
    set(dronePatch, 'Vertices', rotated_pts + pos);

    x = pos(1); y = pos(2); z = pos(3);

    safe_dist = 0.5;
    safe_dist_z = 0.25;

    too_close = false;
    collision = false;
    
    % Safety check in case of collision
    if x <= 0 || x >= 10 || y <= 0 || y >= 10 || z <= 0 || z >= 4
        collision = true;
    end

     if (x >= 0 && x <= 3 && y >= 2.72 && y <= 3.28 && z >= 0 && z <= 4) || ...       % Wall 1
        (x >= 2.72 && x <= 3.28 && y >= 2 && y <= 3 && z >= 0 && z <= 4) || ...       % Wall 2
        (x >= 7 && x <= 10 && y >= 6.72 && y <= 7.28 && z >= 0 && z <= 4) || ...      % Wall 3
        (x >= 6.72 && x <= 7.28 && y >= 7 && y <= 8 && z >= 0 && z <= 4) || ...       % Wall 4
        (x >= 0 && x <= 4 && y >= 4.72 && y <= 5.28 && z >= 0 && z <= 4) || ...       % Wall 5_left
        (x >= 6 && x <= 10 && y >= 4.72 && y <= 5.28 && z >= 0 && z <= 4) || ...      % Wall 5_right
        (x >= 4 && x <= 6 && y >= 4.72 && y <= 5.28 && z >= 0 && z <= 2) || ...       % Wall 5_bot
        (x >= 4 && x <= 6 && y >= 4.72 && y <= 5.28 && z >= 3 && z <= 4)              % Wall 5_top
        collision = true;
    end

    % Safety checks
    if x < safe_dist || x > 10 - safe_dist || ...
       y < safe_dist || y > 10 - safe_dist || ...
       z < safe_dist_z || z > 4 - safe_dist_z
        too_close = true;
    end
    
    % --- wall1
    if x >= 0.06 - safe_dist && x <= 3 + safe_dist && ...
       y >= 3 - safe_dist && y <= 3 + safe_dist && ...
       z >= 0.08 - safe_dist && z <= 4 + safe_dist
        too_close = true;
    end
    
    % --- wall2
    if x >= 3 - safe_dist && x <= 3 + safe_dist && ...
       y >= 2 - safe_dist && y <= 3 + safe_dist && ...
       z >= 0.08 - safe_dist && z <= 4 + safe_dist
        too_close = true;
    end
    
    % --- wall3
    if x >= 7 - safe_dist && x <= 9.94 + safe_dist && ...
       y >= 7 - safe_dist && y <= 7 + safe_dist && ...
       z >= 0.08 - safe_dist && z <= 4 + safe_dist
        too_close = true;
    end
    
    % --- wall4
    if x >= 7 - safe_dist && x <= 7 + safe_dist && ...
       y >= 7 - safe_dist && y <= 8 + safe_dist && ...
       z >= 0.08 - safe_dist && z <= 4 + safe_dist
        too_close = true;
    end
    
    % --- wall5_left
    if x >= 0 - safe_dist && x <= 4 + safe_dist && ...
       y >= 5 - safe_dist && y <= 5 + safe_dist && ...
       z >= 0.08 - safe_dist && z <= 4 + safe_dist
        too_close = true;
    end
    
    % --- wall5_right
    if x >= 6 - safe_dist && x <= 10 + safe_dist && ...
       y >= 5 - safe_dist && y <= 5 + safe_dist && ...
       z >= 0.08 - safe_dist && z <= 4 + safe_dist
        too_close = true;
    end
    
    % --- wall5_bot
    if x >= 4 - safe_dist && x <= 6 + safe_dist && ...
       y >= 5 - safe_dist && y <= 5 + safe_dist && ...
       z >= 0.08 - safe_dist && z <= 2 + safe_dist
        too_close = true;
    end
    
    % --- wall5_top
    if x >= 4 - safe_dist && x <= 6 + safe_dist && ...
       y >= 5 - safe_dist && y <= 5 + safe_dist && ...
       z >= 3 && z <= 4 + safe_dist
        too_close = true;
    end
    
     if collision
        set(dronePatch, 'FaceColor', 'r', 'EdgeColor', 'none');
        title(' COLLISION DETECTED — Simulation interrupted!', 'Color', 'r', 'FontSize', 14);
        disp([' Collision at timestep ', num2str(i), ' | Position: [', num2str(pos), ']']);
        break;
        
     elseif too_close
        set(dronePatch, 'FaceColor', 'r', 'EdgeColor', 'none');
        
     else
        set(dronePatch, 'FaceColor', [0.3 0.5 1], 'EdgeColor', 'k');
     end

    pause(Ts);
        end
