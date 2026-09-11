function [command, memory] = crazyflie_ctbr_controller(...
    state, desired, memory, cfg)
%CRAZYFLIE_CTBR_CONTROLLER 几何位置外环和 CTBR 姿态外环。
%
% 输入：
%   state.position   当前惯性系位置，单位 m
%   state.velocity   当前惯性系速度，单位 m/s
%   state.rotation   当前姿态 R，机体系到惯性系
%   state.bodyRate   当前机体系角速度，单位 rad/s
%   desired.position / velocity / acceleration / rotation
%                      目标位置、速度、加速度和姿态
%   memory           跨采样周期保存的积分器和 Rc
%   cfg              crazyflie_parameters() 返回的参数
%
% 输出 command：
%   thrustPercentage  发给 Crazyflie 的集体推力百分比
%   bodyRateCommand   发给 Crazyflie 的期望机体角速度，单位 rad/s
%   bodyRateCommandDeg 发送接口使用的角速度，单位 deg/s
%
% 注意：这个文件不计算论文中的直接力矩 M。真实 Crazyflie 会在固件内部
% 根据 bodyRateCommand 和机载陀螺仪测量值计算力矩。

if nargin < 4
    error('crazyflie_ctbr_controller:NotEnoughInputs', ...
        '需要 state、desired、memory 和 cfg 四个输入。');
end

dt = cfg.simulation.dt;
e3 = [0; 0; 1];
R = projectSO3(state.rotation);

% ---------- 位置误差和位置积分器 ----------
ex = state.position(:) - desired.position(:);
ev = state.velocity(:) - desired.velocity(:);

positionIntegralRate = ev + cfg.positionController.c1 * ex;
memory.positionIntegral = memory.positionIntegral + dt * positionIntegralRate;
memory.positionIntegral = clampVector(memory.positionIntegral, ...
    -cfg.positionController.integralLimit, ...
    cfg.positionController.integralLimit);

% 论文中的期望惯性系合力 A。
satIntegral = clampVector(memory.positionIntegral, ...
    -cfg.positionController.integralLimit, ...
    cfg.positionController.integralLimit);
satIntegralRate = saturatedIntegralDerivative(memory.positionIntegral, ...
    positionIntegralRate, cfg.positionController.integralLimit);
A = -cfg.positionController.kx * ex ...
    - cfg.positionController.kv * ev ...
    - cfg.positionController.ki * satIntegral ...
    - cfg.vehicle.mass * cfg.vehicle.gravity * e3 ...
    + cfg.vehicle.mass * desired.acceleration(:);

% 解析模式需要 A_dot。没有提供当前加速度时，使用期望加速度作为
% 一个保守的回退值；仿真文件会显式传入当前动力学加速度。
if isfield(state, 'acceleration') && ~isempty(state.acceleration)
    currentAcceleration = state.acceleration(:);
else
    currentAcceleration = desired.acceleration(:);
end
if isfield(desired, 'jerk') && ~isempty(desired.jerk)
    desiredJerk = desired.jerk(:);
else
    desiredJerk = zeros(3, 1);
end
A_dot = -cfg.positionController.kx * ev ...
    - cfg.positionController.kv * (currentAcceleration - desired.acceleration(:)) ...
    - cfg.positionController.ki * satIntegralRate ...
    + cfg.vehicle.mass * desiredJerk;

% ---------- 从合力构造计算姿态 Rc ----------
forceNorm = norm(A);
if forceNorm < cfg.positionController.forceNormEpsilon
    % A 接近零时无法归一化，保持上一时刻的 b3c。
    if isempty(memory.previousComputedRotation)
        b3c = e3;
    else
        b3c = memory.previousComputedRotation(:, 3);
    end
else
    b3c = -A / forceNorm;
end

% 目标姿态只提供航向参考 b1d；滚转和俯仰由位置合力方向决定。
b1d = desired.rotation(:, 1);
b1Projection = (eye(3) - b3c * b3c') * b1d;
if norm(b1Projection) < cfg.positionController.headingProjectionEpsilon
    % 航向轴与 b3c 平行时，使用上一时刻航向或固定备用方向。
    if isempty(memory.previousB1)
        fallback = [1; 0; 0];
        b1Projection = (eye(3) - b3c * b3c') * fallback;
    else
        b1Projection = memory.previousB1;
    end
end
b1c = b1Projection / max(norm(b1Projection), eps);
b2c = cross(b3c, b1c);
b2c = b2c / max(norm(b2c), eps);
b1c = cross(b2c, b3c);
b1c = b1c / max(norm(b1c), eps);
Rc = projectSO3([b1c, b2c, b3c]);

% 计算期望姿态角速度。默认是离散 SO(3) 李群差分；analytic 模式则由
% Rc=[b1c,b2c,b3c] 的列向量导数计算 Rc' * Rc_dot。
omegaCMethod = lower(char(cfg.attitudeController.omegaCMethod));
switch omegaCMethod
    case {'log_difference', 'log', 'so3_log'}
        if isempty(memory.previousComputedRotation)
            computedBodyRate = zeros(3, 1);
        else
            relativeRotation = memory.previousComputedRotation' * Rc;
            computedBodyRate = so3Log(relativeRotation) / max(dt, eps);
        end
        omegaCMethod = 'log_difference';
    case {'analytic', 'derivative'}
        computedBodyRate = analyticOmegaC(Rc, A, A_dot, desired, cfg);
        omegaCMethod = 'analytic';
    otherwise
        error('crazyflie_ctbr_controller:UnknownOmegaCMethod', ...
            '未知 Omega_c 计算方式：%s。可选 log_difference 或 analytic。', ...
            cfg.attitudeController.omegaCMethod);
end
memory.previousComputedRotation = Rc;
memory.previousB1 = Rc(:, 1);

% ---------- 几何姿态误差和 CTBR 角速度指令 ----------
attitudeError = 0.5 * veeMap(Rc' * R - R' * Rc);
computedBodyRateCurrentFrame = R' * Rc * computedBodyRate;

if cfg.attitudeController.useIntegral
    memory.attitudeIntegral = memory.attitudeIntegral + dt * attitudeError;
    memory.attitudeIntegral = clampVector(memory.attitudeIntegral, ...
        -cfg.attitudeController.integralLimit, ...
        cfg.attitudeController.integralLimit);
else
    memory.attitudeIntegral = zeros(3, 1);
end

bodyRateCommand = computedBodyRateCurrentFrame ...
    - cfg.attitudeController.kr .* attitudeError ...
    - cfg.attitudeController.ki .* memory.attitudeIntegral;
bodyRateCommand = clampVector(bodyRateCommand, ...
    -cfg.attitudeController.maxBodyRateCommand, ...
    cfg.attitudeController.maxBodyRateCommand);

% ---------- 推力和 Crazyflie 百分比接口 ----------
% 这是论文意义下的有符号总推力。真实 CTBR 接口只能发送非负推力。
thrustNewton = -A' * R * e3;
thrustForMap = max(thrustNewton, 0);
thrustPercentage = 100 * thrustForMap / cfg.vehicle.maxTotalThrust;
thrustPercentage = clamp(thrustPercentage, 0, 100);

command = emptyCommand();
command.positionError = ex;
command.velocityError = ev;
command.attitudeError = attitudeError;
command.computedRotation = Rc;
command.computedBodyRate = computedBodyRate;
command.bodyRateCommand = bodyRateCommand;
command.bodyRateCommandDeg = rad2deg(bodyRateCommand);
command.bodyRateMeasurement = state.bodyRate(:);
command.omegaCMethod = omegaCMethod;
command.thrustNewton = thrustNewton;
command.thrustPercentage = thrustPercentage;
command.desiredForce = A;
end

function command = emptyCommand()
% 初始化输出结构，便于仿真文件统一记录数据。
command = struct(...
    'positionError', zeros(3, 1), ...
    'velocityError', zeros(3, 1), ...
    'attitudeError', zeros(3, 1), ...
    'computedRotation', eye(3), ...
    'computedBodyRate', zeros(3, 1), ...
    'bodyRateCommand', zeros(3, 1), ...
    'bodyRateCommandDeg', zeros(3, 1), ...
    'bodyRateMeasurement', zeros(3, 1), ...
    'omegaCMethod', 'log_difference', ...
    'thrustNewton', 0, ...
    'thrustPercentage', 0, ...
    'desiredForce', zeros(3, 1));
end

function R = projectSO3(R)
% 将数值误差造成的近似旋转矩阵投影回 SO(3)。
[U, ~, V] = svd(R);
R = U * V';
if det(R) < 0
    U(:, 3) = -U(:, 3);
    R = U * V';
end
end

function v = so3Log(R)
% SO(3) 对数映射，返回旋转向量。
cosTheta = clamp((trace(R) - 1) / 2, -1, 1);
theta = acos(cosTheta);
if theta < 1e-7
    v = 0.5 * veeMap(R - R');
elseif abs(pi - theta) < 1e-5
    % 近似 180 度时使用特征向量求旋转轴。
    [V, D] = eig((R + eye(3)) / 2);
    [~, index] = max(real(diag(D)));
    axis = real(V(:, index));
    axis = axis / max(norm(axis), eps);
    v = theta * axis;
else
    v = theta / (2 * sin(theta)) * veeMap(R - R');
end
end

function v = veeMap(S)
% 反对称矩阵到三维向量的 vee 映射。
v = [S(3, 2); S(1, 3); S(2, 1)];
end

function OmegaC = analyticOmegaC(Rc, A, A_dot, desired, cfg)
% 根据 Rc 的列向量解析求导，计算 Omega_c = vee(Rc' * Rc_dot)。
% 该路径体现了 A_dot -> b3c_dot -> Rc_dot -> Omega_c 的连续时间关系。
b3c = Rc(:, 3);
forceNorm = norm(A);
if forceNorm < cfg.omegaC.forceNormEpsilon
    OmegaC = zeros(3, 1);
    return;
end

% b3c = -A / ||A|| 的导数。
b3cDot = -(eye(3) - b3c * b3c') * A_dot / forceNorm;

% b1c 是 b1d 在垂直于 b3c 平面上的归一化投影。
b1d = desired.rotation(:, 1);
b1dDot = desiredFirstAxisDerivative(desired);
projection = (eye(3) - b3c * b3c') * b1d;
b1c = Rc(:, 1);
if norm(projection) < cfg.omegaC.headingProjectionEpsilon
    % 航向投影退化时，控制器使用了备用方向；此处令其导数为零，
    % 避免在奇异点附近产生异常大的角速度前馈。
    b1cDot = zeros(3, 1);
else
    projectionDot = -(b3cDot * b3c' + b3c * b3cDot') * b1d ...
        + (eye(3) - b3c * b3c') * b1dDot;
    b1cDot = (eye(3) - b1c * b1c') * projectionDot / norm(projection);
end

b2c = Rc(:, 2);
b2cDot = cross(b3cDot, b1c) + cross(b3c, b1cDot);
RcDot = [b1cDot, b2cDot, b3cDot];
OmegaHat = Rc' * RcDot;
OmegaHat = 0.5 * (OmegaHat - OmegaHat');
OmegaC = veeMap(OmegaHat);
end

function b1dDot = desiredFirstAxisDerivative(desired)
% 获取目标航向轴 b1d 的导数。优先使用 rotationDot；也支持 bodyRate。
if isfield(desired, 'rotationDot') && ~isempty(desired.rotationDot)
    b1dDot = desired.rotationDot(:, 1);
elseif isfield(desired, 'bodyRate') && ~isempty(desired.bodyRate)
    % 若 bodyRate 是目标机体系角速度，R_dot = R * hat(Omega_d)。
    b1dDot = desired.rotation * cross(desired.bodyRate(:), [1; 0; 0]);
else
    b1dDot = zeros(3, 1);
end
end

function rate = saturatedIntegralDerivative(integralState, rawRate, limit)
% 逐元素计算 sat(integralState) 的导数，在饱和边界处阻止继续向外积分。
rate = rawRate;
tolerance = 1e-12;
upperActive = integralState >= limit - tolerance & rawRate > 0;
lowerActive = integralState <= -limit + tolerance & rawRate < 0;
rate(upperActive | lowerActive) = 0;
end

function value = clamp(value, lowerBound, upperBound)
% 标量限幅。
value = min(max(value, lowerBound), upperBound);
end

function value = clampVector(value, lowerBound, upperBound)
% 向量逐元素限幅。
value = min(max(value, lowerBound), upperBound);
end
