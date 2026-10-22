function formation_three_quadrotor_visualization(sim, cfg)
%FORMATION_THREE_QUADROTOR_VISUALIZATION 三机编队曲线和简化动画。
%
% 绘图坐标把惯性系 z 轴取反，显示为通常的“高度向上”方向；控制器和
% 动力学内部仍严格使用 V2 的 z 轴向下约定。

if cfg.visualization.plot
    plotSummary(sim, cfg);
end
if cfg.visualization.animate
    animateVehicles(sim, cfg);
end
end

function plotSummary(sim, cfg)
%PLOTSUMMARY 位置、误差和 CTBR 输出摘要。
N = size(sim.position, 2);
colors = lines(N);
figure('Name', '三机编队 CTBR 仿真结果', 'Color', 'w');

subplot(2, 2, 1);
hold on;
for i = 1:N
    x = squeeze(sim.position(1, i, :));
    y = squeeze(sim.position(2, i, :));
    z = -squeeze(sim.position(3, i, :));
    plot3(x, y, z, 'LineWidth', 1.4, 'Color', colors(i, :));
    xTarget = squeeze(sim.targetPosition(1, i, :));
    yTarget = squeeze(sim.targetPosition(2, i, :));
    zTarget = -squeeze(sim.targetPosition(3, i, :));
    plot3(xTarget, yTarget, zTarget, '--', 'LineWidth', 0.9, ...
        'Color', colors(i, :) * 0.65 + 0.35);
end
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
zlabel('altitude = -z (m)');
title(sprintf('三机轨迹 (%s)', char(cfg.formation.type)));
legend(arrayfun(@(i) sprintf('飞机 %d', i), 1:N, 'UniformOutput', false), ...
    'Location', 'best');

subplot(2, 2, 2);
hold on;
for i = 1:N
    plot(sim.time, squeeze(sim.formationErrorNorm(i, :)), ...
        'LineWidth', 1.1, 'Color', colors(i, :));
end
grid on;
xlabel('时间 (s)');
ylabel('相对编队误差范数 (m)');
title('位移型误差');
legend(arrayfun(@(i) sprintf('飞机 %d', i), 1:N, 'UniformOutput', false), ...
    'Location', 'best');

subplot(2, 2, 3);
hold on;
for i = 1:N
    plot(sim.time, squeeze(sim.leaderErrorNorm(i, :)), ...
        'LineWidth', 1.1, 'Color', colors(i, :));
end
grid on;
xlabel('时间 (s)');
ylabel('误差范数 (m)');
title('绝对编队/领导参考误差');
legend(arrayfun(@(i) sprintf('飞机 %d', i), 1:N, 'UniformOutput', false), ...
    'Location', 'best');

subplot(2, 2, 4);
hold on;
for i = 1:N
    plot(sim.time, squeeze(sim.thrustPercentage(i, :)), ...
        'LineWidth', 1.1, 'Color', colors(i, :));
end
grid on;
xlabel('时间 (s)');
ylabel('推力指令 (%)');
title(sprintf('CTBR 推力 (%s)', char(sim.omegaCMethod)));
legend(arrayfun(@(i) sprintf('飞机 %d', i), 1:N, 'UniformOutput', false), ...
    'Location', 'best');
end

function animateVehicles(sim, cfg)
%ANIMATEVEHICLES 播放三架四旋翼的简化三维动画。
N = size(sim.position, 2);
colors = lines(N);
animationFigure = figure('Name', '三机编队 CTBR 三维动画', 'Color', 'w');
animationAxes = axes(animationFigure);
hold(animationAxes, 'on');
grid(animationAxes, 'on');
axis(animationAxes, 'equal');
xlabel(animationAxes, 'x (m)');
ylabel(animationAxes, 'y (m)');
zlabel(animationAxes, 'altitude = -z (m)');
title(animationAxes, '三机编队 8 字飞行');

allPoints = [reshape(sim.position, 3, []), reshape(sim.targetPosition, 3, [])];
limitsMin = min(allPoints, [], 2) - cfg.visualization.axisPadding;
limitsMax = max(allPoints, [], 2) + cfg.visualization.axisPadding;
xlim(animationAxes, [limitsMin(1), limitsMax(1)]);
ylim(animationAxes, [limitsMin(2), limitsMax(2)]);
zlim(animationAxes, [-limitsMax(3), -limitsMin(3)]);
view(animationAxes, 35, 25);

trajectoryLine = gobjects(N, 1);
bodyLine = gobjects(N, 1);
axisLines = gobjects(3, N);
motorMarkers = gobjects(N, 1);
for i = 1:N
    trajectoryLine(i) = animatedline(animationAxes, ...
        'Color', colors(i, :), 'LineWidth', 1.2);
    bodyLine(i) = plot3(animationAxes, nan, nan, nan, '-', ...
        'Color', colors(i, :), 'LineWidth', 2.0);
    motorMarkers(i) = plot3(animationAxes, nan, nan, nan, 'ko', ...
        'MarkerFaceColor', colors(i, :), 'MarkerSize', 4);
    for axisIndex = 1:3
        axisColors = [0.85, 0.15, 0.10; 0.10, 0.55, 0.15; 0.10, 0.25, 0.75];
        axisLines(axisIndex, i) = plot3(animationAxes, nan, nan, nan, '-', ...
            'Color', axisColors(axisIndex, :), 'LineWidth', 1.3);
    end
end

videoWriter = [];
if cfg.visualization.saveVideo
    videoWriter = VideoWriter(cfg.visualization.videoFile, 'MPEG-4');
    videoWriter.FrameRate = max(1, round(1 / (cfg.simulation.dt * ...
        cfg.visualization.animationStride)));
    open(videoWriter);
end

armLength = cfg.visualization.quadrotorArmLength;
% 绘图把惯性系 z 轴反向显示为高度轴，因此姿态矩阵也要做相同
% 的坐标变换，否则机体轴线会与位置坐标使用不同的坐标系。
displayTransform = diag([1, 1, -1]);
for k = 1:cfg.visualization.animationStride:numel(sim.time)
    for i = 1:N
        R = sim.rotation(:, :, i, k);
        p = sim.position(:, i, k);
        p(3) = -p(3);
        displayRotation = displayTransform * R;
        addpoints(trajectoryLine(i), p(1), p(2), p(3));

        armPoints = displayRotation(:, 1:2) * ...
            [armLength, 0, -armLength, 0; ...
             0, armLength, 0, -armLength] + p;
        % 用 NaN 把两条机臂分开，避免绘图线在机臂交点之间额外连线。
        armLinePoints = [armPoints(:, 1), armPoints(:, 3), nan(3, 1), ...
                         armPoints(:, 2), armPoints(:, 4)];
        set(bodyLine(i), 'XData', armLinePoints(1, :), ...
            'YData', armLinePoints(2, :), ...
            'ZData', armLinePoints(3, :));
        set(motorMarkers(i), 'XData', armPoints(1, :), ...
            'YData', armPoints(2, :), 'ZData', armPoints(3, :));

        for axisIndex = 1:3
            endpoint = p + armLength * displayRotation(:, axisIndex);
            set(axisLines(axisIndex, i), ...
                'XData', [p(1), endpoint(1)], ...
                'YData', [p(2), endpoint(2)], ...
                'ZData', [p(3), endpoint(3)]);
        end
    end
    title(animationAxes, sprintf('t = %.2f s, 最大推力 = %.1f%%', ...
        sim.time(k), max(sim.thrustPercentage(:, k))));
    drawnow;
    if ~isempty(videoWriter)
        writeVideo(videoWriter, getframe(animationFigure));
    end
end

if ~isempty(videoWriter)
    close(videoWriter);
end
end
