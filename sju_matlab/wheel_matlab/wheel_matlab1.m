%% ========================================================================
% 三角波扭矩激励下的转动惯量辨识（直接驱动总惯量）
% 数据特点：扭矩在 ±4 N·m 之间连续扫描，转速在 ±50 rad/s 之间连续响应
% 模型：T = J * dw/dt + B * w + Tc * sign(w)
% ========================================================================
clear; clc; close all;

%% --------------------- 1. 读取数据 --------------------------------------
filename = 'wheel7.csv';   % 请修改为实际文件名
if ~exist(filename, 'file')
    error('文件 %s 未找到。', filename);
end

% 保留原始列名
data = readtable(filename, 'VariableNamingRule', 'preserve');
disp('文件列名：');
disp(data.Properties.VariableNames);

% 根据实际列名提取（请核对，若不同请修改）
timeCol   = 'Time';
torqueCol = 'l_side.T_wheel';
speedCol  = '((driven[1])->measure).speed_rads';

t = data.(timeCol);
T = data.(torqueCol);
w = data.(speedCol);

% 清理无效值
valid = isfinite(t) & isfinite(T) & isfinite(w);
t = t(valid); T = T(valid); w = w(valid);
[t, idx] = sort(t); T = T(idx); w = w(idx);
t = t(:); T = T(:); w = w(:);

fprintf('数据点数：%d，时间范围：%.2f ~ %.2f s\n', length(t), t(1), t(end));

%% --------------------- 2. 计算角加速度（带平滑） -------------------------
alpha = gradient(w, t);
alpha = smoothdata(alpha, 'gaussian', 20);   % 适度平滑，可根据噪声调整

%% --------------------- 3. 选取有效动态数据段 -----------------------------
% 排除转速接近限幅的平台区（|w| > 48 rad/s）以及扭矩变化死区
w_limit = 48;                % 转速限幅阈值
dTdt_th = 0.05;              % 扭矩变化率阈值，排除静止死区
dTdt = abs(gradient(T, t));

valid_idx = (abs(w) < w_limit) & (dTdt > dTdt_th);

if sum(valid_idx) < 100
    warning('有效动态点较少，放宽条件。');
    valid_idx = (abs(w) < w_limit);
end

t_valid = t(valid_idx);
T_valid = T(valid_idx);
w_valid = w(valid_idx);
alpha_valid = alpha(valid_idx);

fprintf('有效动态数据点数：%d (占总数据 %.1f%%)\n', ...
        sum(valid_idx), 100*sum(valid_idx)/length(t));

%% --------------------- 4. 三参数最小二乘辨识 -----------------------------
% 模型：T = J * alpha + B * w + Tc * sign(w)
X = [alpha_valid, w_valid, sign(w_valid)];
params = X \ T_valid;

J_est = params(1);
B_est = params(2);
Tc_est = abs(params(3));   % 库仑摩擦取正值

% 拟合优度
T_pred = X * params;
R2 = 1 - sum((T_valid - T_pred).^2) / sum((T_valid - mean(T_valid)).^2);

% 若 J 为负，尝试反转扭矩符号（传感器方向定义问题）
if J_est < 0
    warning('辨识得到负惯量，尝试自动反转扭矩符号...');
    params_flip = X \ (-T_valid);
    if params_flip(1) > 0
        J_est = params_flip(1);
        B_est = params_flip(2);
        Tc_est = abs(params_flip(3));
        T = -T;   % 全局翻转，以便后续仿真
        T_valid = -T_valid;
        T_pred = X * [J_est; B_est; Tc_est];
        R2 = 1 - sum((T_valid - T_pred).^2) / sum((T_valid - mean(T_valid)).^2);
        disp('已自动反转扭矩符号，请核实传感器方向。');
    end
end

%% --------------------- 5. 结果输出 --------------------------------------
fprintf('\n========== 辨识结果 ==========\n');
fprintf('模型：T = J·α + B·ω + Tc·sign(ω)\n');
fprintf('--------------------------------\n');
fprintf('转动惯量 J        = %.6f kg·m²\n', J_est);
fprintf('粘滞摩擦系数 B    = %.6f N·m·s/rad\n', B_est);
fprintf('库仑摩擦幅值 Tc   = %.6f N·m\n', Tc_est);
fprintf('拟合 R²           = %.4f\n', R2);
fprintf('================================\n\n');

%% --------------------- 6. 全过程仿真验证 ---------------------------------
w_sim = zeros(size(t));
w_sim(1) = w(1);
for i = 2:length(t)
    dt = t(i) - t(i-1);
    alpha_i = (T(i) - B_est * w_sim(i-1) - Tc_est * sign(w_sim(i-1))) / J_est;
    w_sim(i) = w_sim(i-1) + alpha_i * dt;
end

RMSE_w = sqrt(mean((w - w_sim).^2));
NRMSE = RMSE_w / (max(w) - min(w));

%% --------------------- 7. 可视化 ----------------------------------------
figure('Name', '三角波激励辨识结果', 'Position', [100 100 1400 900]);

% 子图1：原始数据
subplot(2,2,1);
yyaxis left; plot(t, w, 'b-', 'LineWidth', 1.2); ylabel('\omega (rad/s)');
yyaxis right; plot(t, T, 'r-', 'LineWidth', 1.2); ylabel('T (N·m)');
xlabel('t (s)'); title('原始测量数据'); grid on;
legend('\omega', 'T', 'Location', 'best');

% 子图2：动态段 T vs α 散点及拟合线
subplot(2,2,2);
scatter(alpha_valid, T_valid, 10, 'b', 'filled', 'MarkerEdgeAlpha', 0.3); hold on;
% 绘制拟合直线（取平均转速下的贡献）
w_mean = mean(w_valid);
alpha_line = linspace(min(alpha_valid), max(alpha_valid), 100);
T_line = J_est * alpha_line + B_est * w_mean + Tc_est * sign(w_mean);
plot(alpha_line, T_line, 'r-', 'LineWidth', 2);
xlabel('\alpha (rad/s^2)'); ylabel('T (N·m)');
title(sprintf('动态段 T vs \\alpha (R² = %.3f)', R2));
grid on; legend('数据点', '拟合线 (固定 ω=mean)', 'Location', 'best');

% 子图3：转速仿真对比
subplot(2,2,3);
plot(t, w, 'b-', 'LineWidth', 1.5); hold on;
plot(t, w_sim, 'r--', 'LineWidth', 1.5);
xlabel('t (s)'); ylabel('\omega (rad/s)');
title(sprintf('转速仿真对比 (NRMSE = %.3f)', NRMSE));
legend('实测', '仿真', 'Location', 'best'); grid on;

% 子图4：拟合残差分析
subplot(2,2,4);
residuals = T_valid - T_pred;
plot(alpha_valid, residuals, 'k.', 'MarkerSize', 6);
xlabel('\alpha (rad/s^2)'); ylabel('残差 (N·m)');
title('拟合残差'); grid on; hold on;
yline(0, 'r--');
yline(2*std(residuals), 'g--', '+2\sigma');
yline(-2*std(residuals), 'g--', '-2\sigma');

sgtitle('三角波扭矩激励下转动惯量辨识', 'FontSize', 14, 'FontWeight', 'bold');

%% --------------------- 8. 保存结果 --------------------------------------
fid = fopen('identification_results.txt', 'w');
fprintf(fid, '三角波激励辨识结果\n');
fprintf(fid, '==================\n');
fprintf(fid, '模型: T = J*α + B*ω + Tc*sign(ω)\n');
fprintf(fid, 'J  = %.6f kg·m²\n', J_est);
fprintf(fid, 'B  = %.6f N·m·s/rad\n', B_est);
fprintf(fid, 'Tc = %.6f N·m\n', Tc_est);
fprintf(fid, 'R² = %.4f\n', R2);
fprintf(fid, '转速仿真 NRMSE = %.4f\n', NRMSE);
fclose(fid);

saveas(gcf, 'identification_plots.png');
disp('结果已保存至 identification_results.txt 和 identification_plots.png');