function cfg = crazyflie_parameters(userCfg)
%CRAZYFLIE_PARAMETERS Crazyflie 仿真的集中参数文件。
%
% 用法：
%   cfg = crazyflie_parameters();
%   cfg = crazyflie_parameters(userCfg);
%
% userCfg 中给出的字段会覆盖这里的默认值。例如：
%   userCfg.simulation.duration = 20;
%   userCfg.visualization.animate = false;
%
% 坐标系沿用论文约定：惯性系 e3 指向重力方向，z 轴向下，推力为
% -f*R*e3。R 是“机体系到惯性系”的旋转矩阵。

if nargin < 1 || isempty(userCfg)
    userCfg = struct();
end

% 仿真时间和离散步长。
cfg.simulation = struct(...
    'duration', 12.0, ...              % 仿真总时间，单位 s
    'dt', 0.002, ...                   % 仿真步长，单位 s
    'maxBodyRate', [8; 8; 6]);         % 仿真中角速度安全上限，单位 rad/s

% Crazyflie 2.1 Brushless 的近似物理参数。
% 惯量和最大推力应在真实平台辨识后替换。
cfg.vehicle = struct(...
    'mass', 0.037, ...                 % 质量，单位 kg
    'gravity', 9.81, ...               % 重力加速度，单位 m/s^2
    'inertia', diag([1.8e-5, 1.8e-5, 3.0e-5]), ... % 机体惯性矩阵 J
    'maxTotalThrust', 0.75, ...        % 四个电机总最大推力，单位 N
    'thrustTimeConstant', 0.035, ...  % 推力一阶响应时间常数，单位 s
    'externalForce', zeros(3, 1));     % 外部扰动力，单位 N

% 几何位置外环增益。
cfg.positionController = struct(...
    'kx', diag([0.75, 0.75, 0.95]), ...
    'kv', diag([0.45, 0.45, 0.55]), ...
    'ki', diag([0.05, 0.05, 0.08]), ...
    'c1', 0.35, ...                    % 位置积分器中的交叉项系数
    'integralLimit', [0.8; 0.8; 0.8], ...
    'forceNormEpsilon', 1e-7, ...      % ||A|| 的数值保护阈值
    'headingProjectionEpsilon', 1e-6); % 航向投影的数值保护阈值

% CTBR 姿态外环增益。
% 该环输出期望机体角速度，而不是论文中的直接力矩 M。
cfg.attitudeController = struct(...
    'kr', [3.0; 3.0; 2.0], ...         % 姿态误差到角速度的增益
    'useIntegral', false, ...          % 是否启用姿态外环积分
    'ki', [0.15; 0.15; 0.10], ...      % 姿态外环积分增益
    'integralLimit', [0.5; 0.5; 0.5], ...
    'maxBodyRateCommand', [4.5; 4.5; 3.5], ... % 角速度指令上限，单位 rad/s
    'omegaCMethod', 'analytic'); % 'log_difference' 或 'analytic'

% 解析计算 Omega_c 时使用的数值保护参数。
% analytic 模式需要 desired.jerk；若未提供则默认使用零 jerk。
cfg.omegaC = struct(...
    'forceNormEpsilon', 1e-7, ...       % ||A|| 的数值保护阈值
    'headingProjectionEpsilon', 1e-6);  % b1d 投影的数值保护阈值

% 仿真用的 Crazyflie 内部角速度控制器。
% 真实飞行时，这一环由 Crazyflie 固件和机载陀螺仪完成。
cfg.rateController = struct(...
    'kp', [10; 10; 7], ...             % 角速度比例增益
    'ki', [3; 3; 2], ...               % 角速度积分增益
    'integralLimit', [1.5; 1.5; 1.0], ...
    'maxMoment', [0.003; 0.003; 0.0015]); % 力矩上限，单位 N*m

% 初始状态。
cfg.initial = struct();
cfg.initial.position = [0.35; -0.25; 0.20]; % 位置，z 轴向下，单位 m
cfg.initial.velocity = zeros(3, 1);         % 惯性系速度，单位 m/s
cfg.initial.bodyRate = zeros(3, 1);         % 机体系角速度，单位 rad/s
cfg.initial.rpy = deg2rad([8; -6; -20]);   % [roll; pitch; yaw]，单位 rad
cfg.initial.R = rpyToRotm(cfg.initial.rpy);
cfg.initial.thrustNewton = cfg.vehicle.mass * cfg.vehicle.gravity;

% 静态目标状态。位置模式中目标姿态主要提供航向参考。
cfg.target = struct();
cfg.target.position = [0; 0; -0.45];
cfg.target.velocity = zeros(3, 1);
cfg.target.acceleration = zeros(3, 1);
cfg.target.rpy = deg2rad([0; 0; 30]);
cfg.target.R = rpyToRotm(cfg.target.rpy);

% 如果不为空，该函数至少返回 [position, velocity, acceleration, rotation]。
% 解析 Omega_c 模式还可扩展第五个输出 jerk 和第六个输出 rotationDot：
%   [x, v, a, R, jerk, Rdot] = myReference(t)
% 例如：cfg.referenceFcn = @myReference;
cfg.referenceFcn = [];

% 可视化参数。
cfg.visualization = struct(...
    'plot', true, ...                  % 是否绘制结果曲线
    'animate', true, ...               % 是否播放三维动画
    'animationStride', 10, ...         % 每隔多少个仿真步刷新一次动画
    'quadrotorArmLength', 0.09, ...    % 动画中的机臂长度，单位 m
    'axisPadding', 0.25, ...           % 三维坐标轴边界留白，单位 m
    'saveVideo', false, ...            % 是否保存动画视频
    'videoFile', 'crazyflie_ctbr_simulation.mp4');

% 用用户参数覆盖默认参数，并保持嵌套结构的其他字段不变。
cfg = mergeStruct(cfg, userCfg);

% 用户只修改 rpy 时，自动重新生成旋转矩阵。
if ~isfield(userCfg, 'initial') || ~isfield(userCfg.initial, 'R')
    cfg.initial.R = rpyToRotm(cfg.initial.rpy);
end
if ~isfield(userCfg, 'target') || ~isfield(userCfg.target, 'R')
    cfg.target.R = rpyToRotm(cfg.target.rpy);
end
end

function R = rpyToRotm(rpy)
% ZYX 欧拉角顺序：先 yaw，再 pitch，再 roll。
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);
R = [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr; ...
     sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr; ...
     -sp,     cp * sr,                  cp * cr];
end

function out = mergeStruct(base, override)
% 递归合并结构体，便于只覆盖少数参数。
out = base;
fields = fieldnames(override);
for i = 1:numel(fields)
    name = fields{i};
    if isstruct(override.(name)) && isfield(base, name) && ...
            isstruct(base.(name))
        out.(name) = mergeStruct(base.(name), override.(name));
    else
        out.(name) = override.(name);
    end
end
end
