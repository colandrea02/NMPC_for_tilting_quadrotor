
outdir = fullfile(pwd, 'img');
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

%% position
posfig = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.p_des.time, out.p_des.signals.values(:,1), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.pos.time, out.pos.signals.values(:,1), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
title('UAV position components','Interpreter','latex','FontSize',14);
ylabel('$p_x$ [m]','Interpreter','latex','FontSize',13);
legend('$p_{x,ref}$','$p_x$','Interpreter','latex','Location','southeast','FontSize',12);
axis([0 t_final -0.05 10.05]);
grid on;

nexttile;
plot(out.p_des.time, out.p_des.signals.values(:,2), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.pos.time, out.pos.signals.values(:,2), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
ylabel('$p_y$ [m]','Interpreter','latex','FontSize',13);
legend('$p_{y,ref}$','$p_y$','Interpreter','latex','Location','southeast','FontSize',12);
axis([0 t_final -0.05 10.05]);
grid on;

nexttile;
plot(out.p_des.time, out.p_des.signals.values(:,3), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.pos.time, out.pos.signals.values(:,3), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
xlabel('Time [s]','Interpreter','latex','FontSize',13);
ylabel('$p_z$ [m]','Interpreter','latex','FontSize',13);
legend('$p_{z,ref}$','$p_z$','Interpreter','latex','Location','northeast','FontSize',12);
axis([0 t_final -0.05 4.05]);
grid on;


%% z
zfig=figure;
plot(out.p_des.time, out.p_des.signals.values(:,3), 'LineWidth', 1, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.pos.time, out.pos.signals.values(:,3), 'LineWidth', 1.2, 'Color', 'k', 'LineStyle', '-');
xlabel('Time [s]','Interpreter','latex','FontSize',13);
ylabel('$p_z$ [m]','Interpreter','latex','FontSize',13);
legend('$p_{z,ref}$','$p_z$','Interpreter','latex','Location','northeast','FontSize',12);
title('z-axis tracking','Interpreter','latex','FontSize',14);
axis([0 t_final -0.05 4.05]);
grid on;


%% velocity
velfig = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.v_des.time, out.v_des.signals.values(:,1), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.vel.time, out.vel.signals.values(:,1), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
title('UAV velocity components','Interpreter','latex','FontSize',14);
ylabel('$v_{x}$ [m/s]','Interpreter','latex','FontSize',13);
legend('$v_{x,ref}$','$v_{x}$','Interpreter','latex','Location','northeast','FontSize',9.5);
grid on;

nexttile;
plot(out.v_des.time, out.v_des.signals.values(:,2), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.vel.time, out.vel.signals.values(:,2), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
ylabel('$v_{y}$ [m/s]','Interpreter','latex','FontSize',13);
legend('$v_{y,ref}$','$v_{y}$','Interpreter','latex','Location','northeast','FontSize',9.5);
grid on;

nexttile;
plot(out.v_des.time, out.v_des.signals.values(:,3), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.vel.time, out.vel.signals.values(:,3), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
xlabel('Time [s]','Interpreter','latex','FontSize',13);
ylabel('$v_{z}$ [m/s]','Interpreter','latex','FontSize',13);
legend('$v_{z,ref}$','$v_{z}$','Interpreter','latex','Location','southeast','FontSize',9.5);
grid on;



%% orientation
q_des = squeeze(out.q_des.signals.values)';
q = squeeze(out.q.signals.values)';
eta_des = quat2eul(q_des, "XYZ");
eta = quat2eul(q, "XYZ");
etafig = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.q_des.time, eta_des(:,1), 'LineWidth', 1.5, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.q.time, eta(:,1), 'LineWidth', 1.3, 'Color', 'k');
title('RPY Angles','Interpreter','latex','FontSize',14);
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',13);
legend('$\phi_{ref}$','$\phi$','Interpreter','latex','Location','best','FontSize',10);
grid on;

nexttile;
plot(out.q_des.time, eta_des(:,2), 'LineWidth', 1.5, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.q.time, eta(:,2), 'LineWidth', 1.3, 'Color', 'k');
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',13);
legend('$\theta_{ref}$','$\theta$','Interpreter','latex','Location','best','FontSize',10);
axis([0 t_final 0 1.2e-2])
grid on;

nexttile;
plot(out.q_des.time, eta_des(:,3), 'LineWidth', 1.5, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.q.time, eta(:,3), 'LineWidth', 1.3, 'Color', 'k');
xlabel('Time [s]','Interpreter','latex','FontSize',13);
ylabel('$\psi$ [rad]','Interpreter','latex','FontSize',13);
legend('$\psi_{ref}$','$\psi_z$','Interpreter','latex','Location','best','FontSize',10);
axis([0 t_final -2e-4 1.2e-4])
grid on;



%% angular velocity
angvelfig = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.om_des.time, out.om_des.signals.values(:,1), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.omega.time, out.omega.signals.values(:,1), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
title('UAV Angular Velocity Components','Interpreter','latex','FontSize',14);
ylabel('$\omega_{x}$ [rad/s]','Interpreter','latex','FontSize',13);
legend('$\omega_{x,ref}$','$\omega_{x}$','Interpreter','latex','Location','northeast','FontSize',9.5);
axis([0 t_final -5e-3 4e-3])
grid on;

nexttile;
plot(out.om_des.time, out.om_des.signals.values(:,2), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.omega.time, out.omega.signals.values(:,2), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
ylabel('$\omega_{y}$ [rad/s]','Interpreter','latex','FontSize',13);
legend('$\omega_{y,ref}$','$\omega_{y}$','Interpreter','latex','Location','northeast','FontSize',9.5);
axis([0 t_final -4e-3 8e-3])
grid on;

nexttile;
plot(out.om_des.time, out.om_des.signals.values(:,3), 'LineWidth', 2, 'Color', 'r', 'LineStyle','--');
hold on;
plot(out.omega.time, out.omega.signals.values(:,3), 'LineWidth', 1.3, 'Color', 'k', 'LineStyle', '-');
xlabel('Time [s]','Interpreter','latex','FontSize',13);
ylabel('$\omega_{z}$ [rad/s]','Interpreter','latex','FontSize',13);
legend('$\omega_{z,ref}$','$\omega_{z}$','Interpreter','latex','Location','southeast','FontSize',9.5);
axis([0 t_final -3.5e-3 2e-3])
grid on;



%% Errors
err_norm = vecnorm(squeeze(out.err_p.signals.values)', 2, 2);
err_vel_norm = vecnorm(squeeze(out.err_v.signals.values)', 2, 2);
err_angvel_norm = vecnorm(squeeze(out.err_om.signals.values)', 2, 2);


errnorm = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
plot(out.err_p.time, err_norm, 'LineWidth', 1.3, 'Color', 'k')
title('Position error norm', 'Interpreter','latex', 'FontSize', 14)
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 13)
ylabel("$\|e_p\|$[m]", 'Interpreter','latex', 'FontSize', 13)
grid on;



verrnorm = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
plot(out.err_v.time, err_vel_norm, 'LineWidth', 1.3, 'Color', 'k');
title('Velocity error norm', 'Interpreter','latex', 'FontSize', 14)
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 13)
ylabel("$\|e_v\|$[m/s]", 'Interpreter','latex', 'FontSize', 13)
grid on;



oerrnorm = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
err_q = squeeze(out.err_q.signals.values)';
err_eta = quat2eul(err_q, "XYZ");
err_eta_norm = vecnorm(err_eta, 2, 2);
hold on
plot(out.q.time, err_eta_norm, 'LineWidth', 1.5, 'Color', 'k');      %phi
hold off
title('Orientation error norm', 'Interpreter','latex', 'FontSize', 14)
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 13)
ylabel("$\|e_\eta\|$[rad]", 'Interpreter','latex', 'FontSize', 13)
grid on;



voerrnorm = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
plot(out.err_om.time, err_angvel_norm, 'LineWidth', 1.3, 'Color', 'k');
title('Angular velocity error norm', 'Interpreter','latex', 'FontSize', 14)
xlabel('Time [s]', 'Interpreter','latex', 'FontSize', 13)
ylabel("$\|e_{\omega}\|$[rad/s]", 'Interpreter','latex', 'FontSize', 13)
grid on;



%% Actuator commands

uw = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.uw.time, out.uw.signals.values(:,1), 'LineWidth', 1.5, 'Color', 'k');
title('$u_{\omega,1}$','Interpreter','latex','FontSize',13);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$[rad^2/s^2]$','Interpreter','latex','FontSize',12);
grid on;

nexttile;
plot(out.uw.time, out.uw.signals.values(:,2), 'LineWidth', 1.5, 'Color', 'k');
title('$u_{\omega,2}$','Interpreter','latex','FontSize',13);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$[rad^2/s^2]$','Interpreter','latex','FontSize',12);
grid on;

nexttile;
plot(out.uw.time, out.uw.signals.values(:,3), 'LineWidth', 1.5, 'Color', 'k');
title('$u_{\omega,3}$','Interpreter','latex','FontSize',13);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$[rad^2/s^2]$','Interpreter','latex','FontSize',12);
grid on;

nexttile;
plot(out.uw.time, out.uw.signals.values(:,4), 'LineWidth', 1.5, 'Color', 'k');
title('$u_{\omega,4}$','Interpreter','latex','FontSize',13);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$[rad^2/s^2]$','Interpreter','latex','FontSize',12);
grid on;

tl = gca;
title(tl.Parent,'Spinning Propellers Velocities','Interpreter','latex','FontSize',15);




%% alpha
alpha_unwrapped = unwrap(out.alpha.signals.values);
alphaFig = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.alpha.time, alpha_unwrapped(:,1), 'LineWidth', 1.5, 'Color', 'k');
title('$\alpha_1$','Interpreter','latex','FontSize',13);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('[rad]','Interpreter','latex','FontSize',12);
grid on;

nexttile;
plot(out.alpha.time, alpha_unwrapped(:,2), 'LineWidth', 1.5, 'Color', 'k');
title('$\alpha_2$','Interpreter','latex','FontSize',13);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('[rad]','Interpreter','latex','FontSize',12);
grid on;

nexttile;
plot(out.alpha.time, alpha_unwrapped(:,3), 'LineWidth', 1.5, 'Color', 'k');
title('$\alpha_3$','Interpreter','latex','FontSize',13);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('[rad]','Interpreter','latex','FontSize',12);
grid on;

nexttile;
plot(out.alpha.time, alpha_unwrapped(:,4), 'LineWidth', 1.5, 'Color', 'k');
title('$\alpha_4$','Interpreter','latex','FontSize',13);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('[rad]','Interpreter','latex','FontSize',12);
grid on;

tl = gca;
title(tl.Parent,'Tilting Angle Commands','Interpreter','latex','FontSize',15);



%% Control inputs

ffig = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.f_opt.time, squeeze(out.f_opt.signals.values(1,1,:)), ...
    'LineWidth', 2, 'Color', 'r', 'LineStyle', '--');
hold on;
plot(out.f_b.time, out.f_b.signals.values(:,1), 'LineWidth', 1.3, 'Color', 'k');
title('Force Components','Interpreter','latex','FontSize',14);
ylabel('$f_{x}$ [N]','Interpreter','latex','FontSize',13);
legend('$f_{x}^*$','$f_{x}$','Interpreter','latex','Location','northeast','FontSize',10);
axis([0 t_final -2 2])
grid on;

nexttile;
plot(out.f_opt.time, squeeze(out.f_opt.signals.values(2,1,:)), ...
    'LineWidth', 2, 'Color', 'r', 'LineStyle', '--');
hold on;
plot(out.f_b.time, out.f_b.signals.values(:,2), 'LineWidth', 1.3, 'Color', 'k');
ylabel('$f_{y}$ [N]','Interpreter','latex','FontSize',13);
legend('$f_{y}^*$','$f_{y}$','Interpreter','latex','Location','southeast','FontSize',10);
axis([0 t_final -4 4])
grid on;

nexttile;
plot(out.f_opt.time, squeeze(out.f_opt.signals.values(3,1,:)), ...
    'LineWidth', 2, 'Color', 'r', 'LineStyle', '--');
hold on;
plot(out.f_b.time, out.f_b.signals.values(:,3), 'LineWidth', 1.3, 'Color', 'k');
xlabel('Time [s]','Interpreter','latex','FontSize',13);
ylabel('$f_{z}$ [N]','Interpreter','latex','FontSize',13);
legend('$f_{z}^*$','$f_{z}$','Interpreter','latex','Location','southeast','FontSize',10);
axis([0 t_final 0 20])
grid on;



taufig = figure('Units','normalized','OuterPosition',[0.2 0.2 0.4 0.6]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.tau_opt.time, squeeze(out.tau_opt.signals.values(1,1,:)), ...
    'LineWidth', 2, 'Color', 'r', 'LineStyle', '--');
hold on;
plot(out.tau_b.time, out.tau_b.signals.values(:,1), 'LineWidth', 1.3, 'Color', 'k');
title('Torque Components','Interpreter','latex','FontSize',14);
ylabel('$\tau_{x}$ [Nm]','Interpreter','latex','FontSize',13);
legend('$\tau_{x}^*$','$\tau_{x}$','Interpreter','latex','Location','northeast','FontSize',12);
grid on;

nexttile;
plot(out.tau_opt.time, squeeze(out.tau_opt.signals.values(2,1,:)), ...
    'LineWidth', 2, 'Color', 'r', 'LineStyle', '--');
hold on;
plot(out.tau_b.time, out.tau_b.signals.values(:,2), 'LineWidth', 1.3, 'Color', 'k');
ylabel('$\tau_{y}$ [Nm]','Interpreter','latex','FontSize',13);
legend('$\tau_{y}^*$','$\tau_{y}$','Interpreter','latex','Location','northeast','FontSize',12);
grid on;

nexttile;
plot(out.tau_opt.time, squeeze(out.tau_opt.signals.values(3,1,:)), ...
    'LineWidth', 2, 'Color', 'r', 'LineStyle', '--');
hold on;
plot(out.tau_b.time, out.tau_b.signals.values(:,3), 'LineWidth', 1.3, 'Color', 'k');
xlabel('Time [s]','Interpreter','latex','FontSize',13);
ylabel('$\tau_{z}$ [Nm]','Interpreter','latex','FontSize',13);
legend('$\tau_{z}^*$','$\tau_{z}$','Interpreter','latex','Location','southeast','FontSize',12);
axis([0 t_final -3e-4 3e-4])
grid on;

%% save

% saveas(posfig, fullfile(outdir, 'posfig.eps') , 'epsc')
% saveas(zfig, fullfile(outdir, 'z.eps') , 'epsc')
% saveas(velfig, fullfile(outdir, 'velfig.eps') , 'epsc')
% saveas(etafig, fullfile(outdir, 'etafig.eps') , 'epsc')
% saveas(angvelfig, fullfile(outdir, 'angvel.eps') , 'epsc')
% saveas(errnorm, fullfile(outdir, 'errnorm.eps') , 'epsc')
% saveas(verrnorm, fullfile(outdir, 'verrnorm.eps') , 'epsc')
% saveas(oerrnorm, fullfile(outdir, 'oerrnorm.eps') , 'epsc')
% saveas(voerrnorm, fullfile(outdir, 'voerrnorm.eps') , 'epsc')
% saveas(uw, fullfile(outdir, 'uw_d.eps') , 'epsc')
% saveas(alphaFig, fullfile(outdir, 'alphaFig.eps') , 'epsc')
% saveas(ffig, fullfile(outdir, 'ffig.eps') , 'epsc')
% saveas(taufig, fullfile(outdir, 'taufig.eps') , 'epsc')

