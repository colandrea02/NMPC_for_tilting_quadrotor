%% TIME SETTINGS
Ts = 0.025;
ttot = 20;       % Trajectory duration
tdead = 5;       % Hover duration at target
t_final = ttot + tdead;
t1 = linspace(0, ttot, round(ttot/Ts));
t2 = linspace(Ts, tdead, round(tdead/Ts));
t = [t1, ttot + t2];


%% WAYPOINT TRAJECTORY 
waypoints = [
    2, 2, 1.5;          % Start
    4.3, 2.5, 0.35;
    5.0, 6.0, 2.8;      % Center of winfow
    6.5, 8.5, 3.75;     % Exit of window
    9.0, 9.0, 1;        % Target
];


waypoint_times = linspace(0, ttot, size(waypoints, 1));

%% SPLINE INTERPOLATION OF TRAJECTORY
px = spline(waypoint_times, waypoints(:,1)', t1);
py = spline(waypoint_times, waypoints(:,2)', t1);
pz = spline(waypoint_times, waypoints(:,3)', t1);


px = [px, repmat(px(end), 1, length(t2))];
py = [py, repmat(py(end), 1, length(t2))];
pz = [pz, repmat(pz(end), 1, length(t2))];


window_size = 15;           
filter_range = [781, 840];  
iterations = 4;             

% Setup
window_half = floor(window_size / 2);
filter_start =  filter_range(1);
filter_end = filter_range(2);



 px_ext = [repmat(px(1), 1, window_half), px, repmat(px(end), 1, window_half)];
 py_ext = [repmat(py(1), 1, window_half), py, repmat(py(end), 1, window_half)];
 pz_ext = [repmat(pz(1), 1, window_half), pz, repmat(pz(end), 1, window_half)];


% Applying filter
for iter = 1:iterations
    px_temp = px_ext; py_temp = py_ext; pz_temp = pz_ext;
    for i = filter_start:filter_end
        idx = i + window_half;
        
        
        if i <= filter_start + 10
            weight = (i - filter_start) / 10; 
        elseif i >= filter_end - 10
            weight = (filter_end - i) / 10;   
        else
            weight = 1;                       
        end
        
        
        filtered_x = mean(px_temp(idx-window_half:idx+window_half));
        filtered_y = mean(py_temp(idx-window_half:idx+window_half));
        filtered_z = mean(pz_temp(idx-window_half:idx+window_half));
        
        px_ext(idx) = weight * filtered_x + (1-weight) * px_temp(idx);
        py_ext(idx) = weight * filtered_y + (1-weight) * py_temp(idx);
        pz_ext(idx) = weight * filtered_z + (1-weight) * pz_temp(idx);
    end
end


px = px_ext(window_half+1:end-window_half);
py = py_ext(window_half+1:end-window_half);
pz = pz_ext(window_half+1:end-window_half);

% Derivatives
dot_px = gradient(px, Ts);   ddot_px = gradient(dot_px, Ts);
dot_py = gradient(py, Ts);   ddot_py = gradient(dot_py, Ts);
dot_pz = gradient(pz, Ts);   ddot_pz = gradient(dot_pz, Ts);



%% TRAJECTORY STRUCTURES FOR SIMULINK
csi_d = [px; py; pz];
dot_csi_d = [dot_px; dot_py; dot_pz];
ddot_csi_d = [ddot_px; ddot_py; ddot_pz];


% Constant orientation 
Rbdes = repmat(eye(3), [1 1 length(t)]);
omegabb_des = zeros(3, length(t));
dot_omegabb_des = zeros(3, length(t));


%% INITIAL CONDITIONS
pos_0 = waypoints(1,:);
lin_vel_0 = [0 0 0];
w_bb_0 = [0 0 0];
alpha0=deg2rad([0 0 0 0]);

R0=eye(3);
q_0=rotm2quat(R0);

x0 = [pos_0'; lin_vel_0'; q_0'; w_bb_0'];