function crazyflie_visualization(sim, cfg)
%CRAZYFLIE_VISUALIZATION 绘制仿真曲线并播放三维动画。
%
% 输入 sim 为 crazyflie_ctbr_simulation() 返回的日志结构，cfg 为
% crazyflie_parameters() 返回的参数结构。本文件不参与控制和动力学计算。

if cfg.visualization.plot
    plotSummary(sim);
end

if cfg.visualization.animate
    animateVehicle(sim, cfg);
end
end

function plotSummary(sim)
% 绘制轨迹、误差、姿态和 CTBR 指令。
figure('Name', 'Crazyflie CTBR 仿真结果', 'Color', 'w');

subplot(2, 2, 1);
plot3(sim.position(1, :), sim.position(2, :), -sim.position(3, :), ...
    'LineWidth', 1.5, 'Color', [0.00, 0.35, 0.75]);
hold on;
plot3(sim.targetPosition(1, :), sim.targetPosition(2, :), ...
    -sim.targetPosition(3, :), '--', 'LineWidth', 1.2, ...
    'Color', [0.85, 0.25, 0.10]);
plot3(sim.position(1, 1), sim.position(2, 1), -sim.position(3, 1), ...
    'o', 'MarkerFaceColor', [0.00, 0.35, 0.75], 'Color', 'none');
plot3(sim.targetPosition(1, end), sim.targetPosition(2, end), ...
    -sim.targetPosition(3, end), 'x', 'LineWidth', 2, ...
    'Color', [0.85, 0.25, 0.10]);
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
zlabel('altitude = -z (m)');
title('位置轨迹');
legend('实际轨迹', '目标轨迹', '起点', '目标点', 'Location', 'best');

subplot(2, 2, 2);
plot(sim.time, sim.positionError', 'LineWidth', 1.1);
grid on;
xlabel('时间 (s)');
ylabel('位置误差 (m)');
title('位置误差');
legend('e_x', 'e_y', 'e_z', 'Location', 'best');

subplot(2, 2, 3);
actualRpy = zeros(3, numel(sim.time));
targetRpy = zeros(3, numel(sim.time));
for k = 1:numel(sim.time)
    actualRpy(:, k) = rotmToRpy(sim.rotation(:, :, k));
    targetRpy(:, k) = rotmToRpy(sim.targetRotation(:, :, k));
end
plot(sim.time, rad2deg(actualRpy'), 'LineWidth', 1.1);
hold on;
plot(sim.time, rad2deg(targetRpy'), '--', 'LineWidth', 1.0);
grid on;
xlabel('时间 (s)');
ylabel('姿态角 (deg)');
title('姿态跟踪');
legend('roll', 'pitch', 'yaw', 'roll target', 'pitch target', ...
    'yaw target', 'Location', 'best');

subplot(2, 2, 4);
yyaxis left;
plot(sim.time, rad2deg(sim.computedBodyRate'), '--', 'LineWidth', 0.9);
hold on;
plot(sim.time, sim.bodyRateCommandDeg', 'LineWidth', 1.1);
ylabel('机体角速度指令 (deg/s)');
yyaxis right;
plot(sim.time, sim.thrustPercentage, 'k', 'LineWidth', 1.3);
ylabel('推力指令 (%)');
grid on;
xlabel('时间 (s)');
title(sprintf('CTBR 输出 (Omega_c: %s)', char(sim.omegaCMethod)));
legend('Omega_c roll', 'Omega_c pitch', 'Omega_c yaw', ...
    'command roll', 'command pitch', 'command yaw', 'thrust', ...
    'Location', 'best');
end

function animateVehicle(sim, cfg)
% 播放简化四旋翼动画：两条机臂、机体坐标轴和运动轨迹。
animationFigure = figure('Name', 'Crazyflie CTBR 三维动画', 'Color', 'w');
animationAxes = axes(animationFigure);
hold(animationAxes, 'on');
grid(animationAxes, 'on');
axis(animationAxes, 'equal');
xlabel(animationAxes, 'x (m)');
ylabel(animationAxes, 'y (m)');
zlabel(animationAxes, 'altitude = -z (m)');
title(animationAxes, 'Crazyflie 2.1 Brushless CTBR 仿真');

% 用实际轨迹和目标轨迹共同确定固定坐标范围，避免动画抖动。
allPoints = [sim.position, sim.targetPosition];
limitsMin = min(allPoints, [], 2) - cfg.visualization.axisPadding;
limitsMax = max(allPoints, [], 2) + cfg.visualization.axisPadding;
xlim(animationAxes, [limitsMin(1), limitsMax(1)]);
ylim(animationAxes, [limitsMin(2), limitsMax(2)]);
zlim(animationAxes, [-limitsMax(3), -limitsMin(3)]);
view(animationAxes, 35, 25);

plot3(animationAxes, sim.targetPosition(1, :), ...
    sim.targetPosition(2, :), -sim.targetPosition(3, :), '--', ...
    'Color', [0.85, 0.25, 0.10], 'LineWidth', 1.0);
goalMarker = plot3(animationAxes, sim.targetPosition(1, end), ...
    sim.targetPosition(2, end), -sim.targetPosition(3, end), ...
    'x', 'Color', [0.85, 0.25, 0.10], 'LineWidth', 2);
trajectoryLine = animatedline(animationAxes, ...
    'Color', [0.00, 0.35, 0.75], 'LineWidth', 1.3);

armLength = cfg.visualization.quadrotorArmLength;
bodyLine = plot3(animationAxes, nan, nan, nan, 'k-', 'LineWidth', 2);
axisLines = gobjects(3, 1);
axisColors = [0.85, 0.15, 0.10; 0.10, 0.55, 0.15; 0.10, 0.25, 0.75];
for i = 1:3
    axisLines(i) = plot3(animationAxes, nan, nan, nan, '-', ...
        'Color', axisColors(i, :), 'LineWidth', 1.5);
end
motorMarkers = plot3(animationAxes, nan, nan, nan, 'ko', ...
    'MarkerFaceColor', [0.15, 0.15, 0.15], 'MarkerSize', 4);

videoWriter = [];
if cfg.visualization.saveVideo
    videoWriter = VideoWriter(cfg.visualization.videoFile, 'MPEG-4');
    videoWriter.FrameRate = max(1, round(1 / (cfg.simulation.dt * ...
        cfg.visualization.animationStride)));
    open(videoWriter);
end

for k = 1:cfg.visualization.animationStride:numel(sim.time)
    R = sim.rotation(:, :, k);
    p = sim.position(:, k);
    % 动画纵轴向上显示，因此把纸面坐标的 z 取反。
    p(3) = -p(3);
    addpoints(trajectoryLine, p(1), p(2), p(3));

    % 两条机臂在机体 b1-b2 平面内，采用 X 形布局示意。
    armPoints = R(:, 1:2) * ...
        [armLength, 0, -armLength, 0; ...
         0, armLength, 0, -armLength] + p;
    set(bodyLine, 'XData', armPoints(1, [1, 3, 2, 4]), ...
        'YData', armPoints(2, [1, 3, 2, 4]), ...
        'ZData', armPoints(3, [1, 3, 2, 4]));
    set(motorMarkers, 'XData', armPoints(1, :), ...
        'YData', armPoints(2, :), 'ZData', armPoints(3, :));

    % 绘制机体坐标轴：红 b1，绿 b2，蓝 b3。
    for i = 1:3
        endpoint = p + armLength * R(:, i);
        set(axisLines(i), 'XData', [p(1), endpoint(1)], ...
            'YData', [p(2), endpoint(2)], ...
            'ZData', [p(3), endpoint(3)]);
    end

    set(goalMarker, 'XData', sim.targetPosition(1, k), ...
        'YData', sim.targetPosition(2, k), ...
        'ZData', -sim.targetPosition(3, k));
    title(animationAxes, sprintf('t = %.2f s, 推力 = %.1f%%', ...
        sim.time(k), sim.thrustPercentage(k)));
    drawnow;

    if ~isempty(videoWriter)
        writeVideo(videoWriter, getframe(animationFigure));
    end
end

if ~isempty(videoWriter)
    close(videoWriter);
end
end

function rpy = rotmToRpy(R)
% 将旋转矩阵转换为 ZYX 顺序的 [roll; pitch; yaw]。
pitch = asin(clamp(-R(3, 1), -1, 1));
if abs(cos(pitch)) > 1e-8
    roll = atan2(R(3, 2), R(3, 3));
    yaw = atan2(R(2, 1), R(1, 1));
else
    roll = 0;
    yaw = atan2(-R(1, 2), R(2, 2));
end
rpy = [roll; pitch; yaw];
end

function value = clamp(value, lowerBound, upperBound)
% 标量限幅。
value = min(max(value, lowerBound), upperBound);
end
