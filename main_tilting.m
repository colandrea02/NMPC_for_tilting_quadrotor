clear
clc

% Reference trajectory
ref_trajectory

% Parameters
params.mass = 1.32;
params.Ib   = diag([0.0154 0.0154 0.0263]);
params.cf   = 1.65776e-5;
params.cm   = 2.1792e-7;
params.l    = 0.45;
Simulink.Bus.createObject(params);


disturbance_type = 0;          

% 0 default configuration without dusturbance
% set disturbance_type=1 to simulate the system with wind-like disturbance
% set disturbance_type=2 to simulate the system with a stochastic noise

%% Controller

nx = 13; % 3 pos, 3 vel, 4 quaternion, 3 omega
nu = 6;  % 3 f, 3 tau
ny = 13; % 3 pos, 3 vel, 4 quaternion, 3 omega

controller = nlmpc(nx, ny, nu);
controller.Ts = Ts;
controller.PredictionHorizon = 5;
controller.ControlHorizon = 5;
controller.Model.StateFcn = @UAV_model;
controller.Model.IsContinuousTime = true;
controller.Model.OutputFcn = @outputFunction;
controller.Model.NumberOfParameters = 5;

controller.States(1).Min = 0;  controller.States(1).Max = 10;  % x
controller.States(2).Min = 0;  controller.States(2).Max = 10;  % y
controller.States(3).Min = 0;  controller.States(3).Max = 4;   % z


Q = [40 40 10 10 10 10 15 15 15 15 5 5 5];
R = 0.1 * ones(1,6);
R_rate = 0.1 * ones(1,6);


controller.Weights.OutputVariables = Q;
controller.Weights.ManipulatedVariables = R;
controller.Weights.ManipulatedVariablesRate = R_rate;

g=9.81*[0 0 1]';
u_0 = [-params.mass*g; 0; 0; 0];

out = sim("NMPC.slx");
animation

%plot_all


%% Functions

% Update function
function updateDroneSTL(currentPos, currentQuat, stl_vertices, droneBody)
    rotated_pts = rotatepoint(currentQuat, stl_vertices);
    set(droneBody, 'Vertices', rotated_pts + currentPos);
end

% Dynamics
function xdot = UAV_model(x, u, mass, Ib, cf, cm, l)

 pos     = x(1:3);
 vel     = x(4:6);
 q       = x(7:10);
 omega   = x(11:13);


 M = [mass*eye(3) zeros(3,3);
       zeros(3,3) Ib];

 C = [-mass*skew(omega), zeros(3,3);
      zeros(3,3) skew(Ib*omega)];

 g_vec = - mass*9.81*[0 0 1 0 0 0]';


 dyn= M\(C * [vel; omega] + g_vec + u );
 
 lin_acc=dyn(1:3);
 ang_acc=dyn(4:6);
 dot_q = 0.5*quatmultiply(q', [0; omega]')'; 

 xdot = [vel;lin_acc; dot_q; ang_acc];

end

% Output function
function y = outputFunction(x, u, mass, Ib, cf, cm, l)

    y = x(1:13);
end

% Skew operator
function S = skew(v)
    if(numel(v)~= 1)
    S = [0 -v(3) v(2); 
         v(3) 0 -v(1);
        -v(2) v(1) 0];
    else
    S = zeros(3);
    end
end