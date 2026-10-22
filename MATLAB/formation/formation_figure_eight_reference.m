function [pc, vc, ac, Rd, jerkc, RdDot] = formation_figure_eight_reference(t)
%FORMATION_FIGURE_EIGHT_REFERENCE 水平 8 字参考轨迹及其导数。
%
% 输出全部使用 V2 的惯性系约定（z 轴向下）：
%   pc, vc, ac, jerkc : 位置、速度、加速度、jerk，均为 3x1；
%   Rd, RdDot         : 目标航向旋转矩阵及其导数。
% 六输出顺序严格采用 simulator/CTBR/codex/v2 的接口：
%   [position, velocity, acceleration, rotation, jerk, rotationDot]
%
% 轨迹为 x=A*sin(w*t), y=B*sin(2*w*t)，z 恒定。目标 b1d 指向
% 水平速度方向；b1d 的导数用解析式计算，供 analytic Omega_c 使用。

center = [0.0; 0.0; -0.60];
amplitudeX = 0.95;
amplitudeY = 0.48;
omega = 0.55;

phase = omega * t;
pc = center + [amplitudeX * sin(phase); ...
               amplitudeY * sin(2 * phase); ...
               0];
vc = [amplitudeX * omega * cos(phase); ...
      2 * amplitudeY * omega * cos(2 * phase); ...
      0];
ac = [-amplitudeX * omega^2 * sin(phase); ...
      -4 * amplitudeY * omega^2 * sin(2 * phase); ...
      0];
jerkc = [-amplitudeX * omega^3 * cos(phase); ...
         -8 * amplitudeY * omega^3 * cos(2 * phase); ...
          0];

horizontalVelocity = vc;
horizontalVelocity(3) = 0;
horizontalAcceleration = ac;
horizontalAcceleration(3) = 0;
speed = norm(horizontalVelocity);
if speed < 1e-8
    b1d = [1; 0; 0];
    b1dDot = zeros(3, 1);
else
    b1d = horizontalVelocity / speed;
    b1dDot = (eye(3) - b1d * b1d') * horizontalAcceleration / speed;
end

e3 = [0; 0; 1];
b2d = cross(e3, b1d);
b2d = b2d / max(norm(b2d), eps);
Rd = [b1d, b2d, e3];
RdDot = [b1dDot, cross(e3, b1dDot), zeros(3, 1)];
end
