function sim = crazyflie_ctbr_simulation(userCfg)
%CRAZYFLIE_CTBR_SIMULATION Crazyflie 2.1 Brushless CTBR 仿真主程序。
%
% 用法：
%   sim = crazyflie_ctbr_simulation();
%   cfg.simulation.duration = 20;
%   sim = crazyflie_ctbr_simulation(cfg);
%
% 本文件只负责：
%   1. 初始化状态和日志；
%   2. 调用几何 CTBR 控制器；
%   3. 模拟推力执行器和角速度内环；
%   4. 积分六自由度刚体动力学；
%   5. 调用独立的可视化文件。

if nargin < 1 || isempty(userCfg)
    userCfg = struct();
end

cfg = crazyflie_parameters(userCfg);
dt = cfg.simulation.dt;
nSteps = floor(cfg.simulation.duration / dt) + 1;
time = (0:nSteps - 1) * dt;

% 分配日志数组。
sim = struct();
sim.time = time;
sim.position = zeros(3, nSteps);
sim.velocity = zeros(3, nSteps);
sim.rotation = zeros(3, 3, nSteps);
sim.bodyRate = zeros(3, nSteps);
sim.targetPosition = zeros(3, nSteps);
sim.targetRotation = zeros(3, 3, nSteps);
sim.computedRotation = zeros(3, 3, nSteps);
sim.positionError = zeros(3, nSteps);
sim.velocityError = zeros(3, nSteps);
sim.attitudeError = zeros(3, nSteps);
sim.computedBodyRate = zeros(3, nSteps);
sim.bodyRateCommand = zeros(3, nSteps);
sim.bodyRateCommandDeg = zeros(3, nSteps);
sim.thrustNewtonCommand = zeros(1, nSteps);
sim.thrustNewtonActual = zeros(1, nSteps);
sim.thrustPercentage = zeros(1, nSteps);
sim.innerMoment = zeros(3, nSteps);
sim.desiredForce = zeros(3, nSteps);
sim.omegaCMethod = char(cfg.attitudeController.omegaCMethod);

% 初始状态。
position = cfg.initial.position(:);
velocity = cfg.initial.velocity(:);
rotation = projectSO3(cfg.initial.R);
bodyRate = cfg.initial.bodyRate(:);
actualThrust = cfg.initial.thrustNewton;

% 控制器的跨周期状态。
memory = struct();
memory.positionIntegral = zeros(3, 1);
memory.attitudeIntegral = zeros(3, 1);
memory.previousComputedRotation = [];
memory.previousB1 = [];
memory.rateIntegral = zeros(3, 1);
lastCommand = emptyCommand();

for k = 1:nSteps
    t = time(k);
    desired = referenceState(t, cfg);
    % 当前动力学加速度供解析 Omega_c 模式计算 A_dot 使用。
    e3 = [0; 0; 1];
    acceleration = cfg.vehicle.gravity * e3 ...
        - (actualThrust / cfg.vehicle.mass) * (rotation * e3) ...
        + cfg.vehicle.externalForce / cfg.vehicle.mass;
    state = struct(...
        'position', position, ...
        'velocity', velocity, ...
        'acceleration', acceleration, ...
        'rotation', rotation, ...
        'bodyRate', bodyRate);

    % Python 几何外环对应这里的 CTBR 控制器调用。
    [command, memory] = crazyflie_ctbr_controller(...
        state, desired, memory, cfg);

    % 记录状态、目标和控制量。
    sim.position(:, k) = position;
    sim.velocity(:, k) = velocity;
    sim.rotation(:, :, k) = rotation;
    sim.bodyRate(:, k) = bodyRate;
    sim.targetPosition(:, k) = desired.position;
    sim.targetRotation(:, :, k) = desired.rotation;
    sim.computedRotation(:, :, k) = command.computedRotation;
    sim.positionError(:, k) = command.positionError;
    sim.velocityError(:, k) = command.velocityError;
    sim.attitudeError(:, k) = command.attitudeError;
    sim.computedBodyRate(:, k) = command.computedBodyRate;
    sim.bodyRateCommand(:, k) = command.bodyRateCommand;
    sim.bodyRateCommandDeg(:, k) = command.bodyRateCommandDeg;
    sim.thrustNewtonCommand(k) = command.thrustNewton;
    sim.thrustNewtonActual(k) = actualThrust;
    sim.thrustPercentage(k) = command.thrustPercentage;
    sim.desiredForce(:, k) = command.desiredForce;

    if k == nSteps
        break;
    end

    % -------- 模拟 Crazyflie 的推力执行器 --------
    % 真实 cflib 发送的是 0--100% 推力；仿真用理想线性映射还原为牛顿。
    requestedThrust = cfg.vehicle.maxTotalThrust * ...
        command.thrustPercentage / 100;
    thrustAlpha = min(1, dt / max(cfg.vehicle.thrustTimeConstant, eps));
    actualThrust = actualThrust + thrustAlpha * ...
        (requestedThrust - actualThrust);
    actualThrust = clamp(actualThrust, 0, cfg.vehicle.maxTotalThrust);

    % -------- 模拟 Crazyflie 内部角速度 PID --------
    % 真实系统中这里使用机载陀螺仪测量 bodyRate；仿真直接使用当前状态。
    rateError = command.bodyRateCommand - bodyRate;
    memory.rateIntegral = memory.rateIntegral + dt * rateError;
    memory.rateIntegral = clampVector(memory.rateIntegral, ...
        -cfg.rateController.integralLimit, ...
        cfg.rateController.integralLimit);
    commandedAngularAcceleration = ...
        cfg.rateController.kp .* rateError + ...
        cfg.rateController.ki .* memory.rateIntegral;
    moment = cfg.vehicle.inertia * commandedAngularAcceleration + ...
        cross(bodyRate, cfg.vehicle.inertia * bodyRate);
    moment = clampVector(moment, -cfg.rateController.maxMoment, ...
        cfg.rateController.maxMoment);
    sim.innerMoment(:, k) = moment;
    lastCommand.innerMoment = moment;

    % -------- 六自由度刚体动力学 --------
    bodyRateDot = cfg.vehicle.inertia \ ...
        (moment - cross(bodyRate, cfg.vehicle.inertia * bodyRate));

    % 半显式 Euler 积分：先更新速度，再用新速度更新位置。
    velocity = velocity + dt * acceleration;
    position = position + dt * velocity;
    bodyRate = bodyRate + dt * bodyRateDot;
    bodyRate = clampVector(bodyRate, -cfg.simulation.maxBodyRate, ...
        cfg.simulation.maxBodyRate);
    rotation = projectSO3(rotation * expSO3(bodyRate * dt));
end

sim.config = cfg;
sim.lastCommand = lastCommand;

% 仿真和绘图解耦：关闭两个开关即可只获得数值结果。
if cfg.visualization.plot || cfg.visualization.animate
    crazyflie_visualization(sim, cfg);
end
end

function desired = referenceState(t, cfg)
% 读取静态目标或用户提供的轨迹函数。
if isempty(cfg.referenceFcn)
    desired.position = cfg.target.position;
    desired.velocity = cfg.target.velocity;
    desired.acceleration = cfg.target.acceleration;
    desired.rotation = cfg.target.R;
    desired.jerk = zeros(3, 1);
    desired.rotationDot = zeros(3, 3);
else
    [desired.position, desired.velocity, desired.acceleration, ...
        desired.rotation, desired.jerk, desired.rotationDot] = ...
        callReferenceFunction(cfg.referenceFcn, t);
end
desired.position = desired.position(:);
desired.velocity = desired.velocity(:);
desired.acceleration = desired.acceleration(:);
desired.rotation = projectSO3(desired.rotation);
if ~isfield(desired, 'jerk') || isempty(desired.jerk)
    desired.jerk = zeros(3, 1);
end
desired.jerk = desired.jerk(:);
if ~isfield(desired, 'rotationDot') || isempty(desired.rotationDot)
    desired.rotationDot = zeros(3, 3);
end
end

function [position, velocity, acceleration, rotation, jerk, rotationDot] = ...
    callReferenceFunction(referenceFcn, t)
% 兼容旧的四输出轨迹函数，同时支持第五输出 jerk 和第六输出 R_dot。
numberOfOutputs = nargout(referenceFcn);
if numberOfOutputs >= 6
    [position, velocity, acceleration, rotation, jerk, rotationDot] = ...
        referenceFcn(t);
elseif numberOfOutputs == 5
    [position, velocity, acceleration, rotation, jerk] = referenceFcn(t);
    rotationDot = zeros(3, 3);
elseif numberOfOutputs == 4
    [position, velocity, acceleration, rotation] = referenceFcn(t);
    jerk = zeros(3, 1);
    rotationDot = zeros(3, 3);
else
    % 对匿名函数或 varargout 函数，尝试新接口，失败后回退到旧接口。
    try
        [position, velocity, acceleration, rotation, jerk, rotationDot] = ...
            referenceFcn(t);
    catch
        [position, velocity, acceleration, rotation] = referenceFcn(t);
        jerk = zeros(3, 1);
        rotationDot = zeros(3, 3);
    end
end
end

function command = emptyCommand()
% 初始化日志所需的控制器输出结构。
command = struct(...
    'positionError', zeros(3, 1), ...
    'velocityError', zeros(3, 1), ...
    'attitudeError', zeros(3, 1), ...
    'computedRotation', eye(3), ...
    'computedBodyRate', zeros(3, 1), ...
    'bodyRateCommand', zeros(3, 1), ...
    'bodyRateCommandDeg', zeros(3, 1), ...
    'bodyRateMeasurement', zeros(3, 1), ...
    'thrustNewton', 0, ...
    'thrustPercentage', 0, ...
    'desiredForce', zeros(3, 1), ...
    'omegaCMethod', 'log_difference', ...
    'innerMoment', zeros(3, 1));
end

function R = projectSO3(R)
% 将数值积分误差投影回合法的旋转矩阵。
[U, ~, V] = svd(R);
R = U * V';
if det(R) < 0
    U(:, 3) = -U(:, 3);
    R = U * V';
end
end

function R = expSO3(v)
% SO(3) 指数映射，用于离散更新姿态。
theta = norm(v);
if theta < 1e-10
    R = eye(3) + hatMap(v);
else
    K = hatMap(v / theta);
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * K * K;
end
end

function S = hatMap(v)
% 三维向量到反对称矩阵的 hat 映射。
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

function value = clamp(value, lowerBound, upperBound)
% 标量限幅。
value = min(max(value, lowerBound), upperBound);
end

function value = clampVector(value, lowerBound, upperBound)
% 向量逐元素限幅。
value = min(max(value, lowerBound), upperBound);
end
