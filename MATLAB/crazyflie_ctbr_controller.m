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

memory.positionIntegral = memory.positionIntegral + ...
    dt * (ev + cfg.positionController.c1 * ex);
memory.positionIntegral = clampVector(memory.positionIntegral, ...
    -cfg.positionController.integralLimit, ...
    cfg.positionController.integralLimit);

% 论文中的期望惯性系合力 A。
satIntegral = clampVector(memory.positionIntegral, ...
    -cfg.positionController.integralLimit, ...
    cfg.positionController.integralLimit);
A = -cfg.positionController.kx * ex ...
    - cfg.positionController.kv * ev ...
    - cfg.positionController.ki * satIntegral ...
    - cfg.vehicle.mass * cfg.vehicle.gravity * e3 ...
    + cfg.vehicle.mass * desired.acceleration(:);

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

% 计算姿态的角速度：Omega_c = vee(log(Rc_prev' * Rc)) / dt。
if isempty(memory.previousComputedRotation)
    computedBodyRate = zeros(3, 1);
else
    relativeRotation = memory.previousComputedRotation' * Rc;
    computedBodyRate = so3Log(relativeRotation) / dt;
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

function value = clamp(value, lowerBound, upperBound)
% 标量限幅。
value = min(max(value, lowerBound), upperBound);
end

function value = clampVector(value, lowerBound, upperBound)
% 向量逐元素限幅。
value = min(max(value, lowerBound), upperBound);
end
