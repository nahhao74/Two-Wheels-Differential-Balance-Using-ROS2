%% CHUONG TRINH TINH TOAN GA-LQR CHO XE CAN BANG 2 BANH
% Trang thai: X = [x; theta; x_dot; theta_dot]
% Dieu kien ban dau: X0 = [0; 15 do; 0; 0]
% Tin hieu dieu khien u: moment dong co, don vi N.m
clear; close all; clc;
rng default;

%% 1. THONG SO VAT LY
g = 9.81;
% ===== BODY: khung + tải + pin + mạch, KHÔNG gồm 2 bánh =====
m_b = 19.48015;          % kg
% ===== BÁNH XE =====
m_w = 2.95;              % kg, khối lượng 1 bánh/motor
r = 0.085;               % m, bán kính bánh xe
% ===== TRỌNG TÂM BODY =====
r_b = 0.14343;           % m, COM body so với Coordinate System1
% ===== MOMENT QUÁN TÍNH =====
I_b = 0.369926930;       % kg.m^2, quanh trục y của body tại COM
I_w = 0.010655;          % kg.m^2, moment quán tính 1 bánh

m_x = m_b + 2*m_w + 2*I_w/(r^2);
I_theta = I_b + m_b*r_b^2;

%% 2. GIOI HAN YEU CAU
u_max = 12;             % N.m
v_max = 0.6;            % m/s
a_max = 2.0;            % m/s^2, dung de tune 15 do

theta_init_deg = 15;    % do
theta_init = deg2rad(theta_init_deg);

%% 2.5 TÍNH TOÁN GÓC NGÃ TỐI ĐA (CHẠM GẦM) DỰA TRÊN BẢN VẼ
robot_depth = 0.260; % m, chiều sâu khung xe từ bản vẽ
L = robot_depth / 2; % m, khoảng cách ngang từ trục đến mép khung ngoài
H = 0.040;           % m, khoảng cách dọc từ trục xuống mép dưới gầm
R = 0.085;           % m, bán kính bánh xe

% Giải phương trình hình học: L*sin(theta) + H*cos(theta) = R
% <=> D * sin(theta + alpha) = R
D = sqrt(L^2 + H^2);
alpha = atan2(H, L);
theta_fall_rad = asin(R / D) - alpha;
theta_fall_deg = rad2deg(theta_fall_rad);

fprintf('=================================================\n');
fprintf('Khoang sang gam xe (Upright) : %.2f mm\n', (R - H)*1000);
fprintf('Goc nga toi da (cham gam)    : %.2f do\n', theta_fall_deg);
fprintf('=================================================\n');

%% 3. MA TRAN HE THONG A, B, C, D
Delta = m_x*I_theta - m_b^2*r_b^2;

A = zeros(4,4);
A(1,3) = 1;
A(2,4) = 1;
A(3,2) = (-m_b^2*r_b^2*g) / Delta;
A(4,2) = (m_x*m_b*r_b*g) / Delta;

B = zeros(4,1);
B(3,1) = (((-2*I_theta)/r) - 2*m_b*r_b) / Delta;
B(4,1) = (((2*m_b*r_b)/r) + 2*m_x) / Delta;

C = eye(4);             % Ma tran don vi 4x4
D = zeros(4,1);

disp('--- MA TRAN A ---');
disp(A);
disp('--- MA TRAN B ---');
disp(B);

%% 4. KIEM TRA NHANH GIOI HAN GIA TOC CO DU CHO 15 DO KHONG
u_need_zero_theta_ddot = -(A(4,2)*theta_init) / B(4,1);
a_need_zero_theta_ddot = A(3,2)*theta_init + B(3,1)*u_need_zero_theta_ddot;

fprintf('\n--- KIEM TRA SO BO O %.1f DO ---\n', theta_init_deg);
fprintf('Moment can de theta_ddot = 0: %.4f N.m\n', u_need_zero_theta_ddot);
fprintf('Gia toc tuong ung: %.4f m/s^2\n', a_need_zero_theta_ddot);
if abs(a_need_zero_theta_ddot) > a_max
    warning('a_max hien tai khong du de phuc hoi goc %.1f do.', theta_init_deg);
end

if theta_init_deg >= theta_fall_deg
    warning('Goc ban dau (%.1f) da lon hon goc cham gam (%.1f). Xe se nga ngay lap tuc!', theta_init_deg, theta_fall_deg);
end

%% 5. THIET LAP GA TUNE LQR
% vars = [q_x, q_theta, q_x_dot, q_theta_dot, R]
lb = [0.001,   1000,    0.001,    10,     1e-4];
ub = [200,     2e5,     100,      8000,   20];

options = optimoptions('ga', ...
    'PopulationSize', 80, ...
    'MaxGenerations', 150, ...
    'EliteCount', 6, ...
    'CrossoverFraction', 0.8, ...
    'FunctionTolerance', 1e-6, ...
    'Display', 'iter');

x0 = [0; theta_init; 0; 0];
dt = 0.005;
t_sim = 0:dt:8;

% Truyen them bien theta_fall_deg vao Cost Function
ObjectiveFunction = @(vars) lqr_cost_function(vars, A, B, x0, t_sim, ...
                                              u_max, v_max, a_max, theta_fall_deg);

fprintf('\nBAT DAU CHAY GA-LQR CHO GOC BAN DAU %.1f DO...\n', theta_init_deg);

%% 6. CHAY GA
[best_vars, best_cost] = ga(ObjectiveFunction, 5, [], [], [], [], ...
                            lb, ub, [], options);

%% 7. TINH BO DIEU KHIEN LQR TOI UU
Q_opt = diag([best_vars(1), best_vars(2), best_vars(3), best_vars(4)]);
R_opt = best_vars(5);
[K_opt, ~, eig_cl] = lqr(A, B, Q_opt, R_opt);

fprintf('\n=================================================\n');
fprintf('   KET QUA GA-LQR TOI UU CHO %.1f DO\n', theta_init_deg);
fprintf('=================================================\n');
disp('Q_opt = '); disp(Q_opt);
disp('R_opt = '); disp(R_opt);
disp('K_opt = '); disp(K_opt);

%% 8. MO PHONG KIEM TRA
x0_test = [0; theta_init; 0; 0];
t_test = 0:dt:10;
[t_total, x_total, u_total, acc_total] = simulate_lqr_limited(A, B, K_opt, ...
                                                              x0_test, t_test, ...
                                                              u_max, v_max, a_max);

%% 9. KIEM TRA GIOI HAN VA TRANG THAI NGA
max_u = max(abs(u_total));
max_v = max(abs(x_total(:,3)));
max_a = max(abs(acc_total));
max_theta = max(abs(x_total(:,2))) * 180/pi;
max_x = max(abs(x_total(:,1)));
theta_final = abs(x_total(end,2))*180/pi;
theta_dot_final = abs(x_total(end,4))*180/pi;

fprintf('\n=================================================\n');
fprintf('   KIEM TRA GIOI HAN SAU MO PHONG\n');
fprintf('=================================================\n');
fprintf('Vi tri lon nhat |x|          = %.4f m\n', max_x);
fprintf('Moment lon nhat |u|          = %.4f N.m\n', max_u);
fprintf('Van toc lon nhat |x_dot|     = %.4f m/s\n', max_v);
fprintf('Gia toc lon nhat |x_dotdot|  = %.4f m/s^2\n', max_a);
fprintf('Goc nghieng lon nhat         = %.4f do (Gioi han: %.2f do)\n', max_theta, theta_fall_deg);
fprintf('Goc nghieng cuoi             = %.6f do\n', theta_final);

% Check ngã
is_falling = false;
if max_theta >= theta_fall_deg
    is_falling = true;
    fprintf('\n!!! CANH BAO: XE DA BI NGA (CHAM GAM XUONG DAT) !!!\n');
end

tol = 1e-6;
if ~is_falling && max_u <= u_max + tol && max_v <= v_max + tol && max_a <= a_max + tol ...
        && theta_final < 0.5 && theta_dot_final < 1.0
    fprintf('Trang thai: DAT YEU CAU CAN BANG.\n');
else
    fprintf('Trang thai: CHUA DAT YEU CAU CAN BANG.\n');
end

%% 10. VE DO THI DAP UNG
figure('Name', 'GA-LQR Initial Angle 15 Deg With Limits', ...
       'Position', [100, 100, 850, 850]);

subplot(3,1,1);
plot(t_total, x_total(:,1), 'b', 'LineWidth', 2);
grid on; ylabel('Vi tri x (m)');
title('Dap ung vi tri tien lui');
yline(0, 'g--');

subplot(3,1,2);
plot(t_total, x_total(:,2)*180/pi, 'r', 'LineWidth', 2);
grid on; ylabel('Goc nghieng theta (deg)');
title('Dap ung goc nghieng voi dieu kien dau 15 do');
yline(0.5, 'g--'); yline(-0.5, 'g--');
% Ve ranh gioi cham gam
yline(theta_fall_deg, 'r-.', 'Cham gam (+)', 'LabelHorizontalAlignment', 'left');
yline(-theta_fall_deg, 'r-.', 'Cham gam (-)', 'LabelHorizontalAlignment', 'left');
if is_falling
    text(t_total(end)/2, 0, 'XE BI NGA!', 'Color', 'r', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end

subplot(3,1,3);
plot(t_total, u_total, 'k', 'LineWidth', 1.5);
grid on; ylabel('Moment u (N.m)'); xlabel('Thoi gian (s)');
title('Tin hieu dieu khien moment dong co');
yline(u_max, 'r--'); yline(-u_max, 'r--');
ylim([-u_max-2, u_max+2]);

%% ========================================================================
%  HAM MUC TIEU CHO GA (Da cap nhat them theta_fall_deg)
%  ========================================================================
function cost = lqr_cost_function(vars, A, B, x0, t_sim, u_max, v_max, a_max, theta_fall_deg)
    Q_mat = diag([vars(1), vars(2), vars(3), vars(4)]);
    R_val = vars(5);
    try
        [K, ~, ~] = lqr(A, B, Q_mat, R_val);
        [t, x, u, acc] = simulate_lqr_limited(A, B, K, x0, t_sim, ...
                                              u_max, v_max, a_max);
        
        theta = x(:,2);
        x_pos = x(:,1);
        v = x(:,3);
        theta_dot = x(:,4);
        dt = t(2) - t(1);
        
        iae_theta = sum(abs(theta)) * dt;
        iae_theta_dot = sum(abs(theta_dot)) * dt;
        iae_x = sum(abs(x_pos)) * dt;
        iae_v = sum(abs(v)) * dt;
        
        max_theta = max(abs(theta)) * 180/pi;
        max_x = max(abs(x_pos));
        max_u = max(abs(u));
        max_v = max(abs(v));
        max_a = max(abs(acc));
        
        theta_final = abs(theta(end));
        theta_dot_final = abs(theta_dot(end));
        v_final = abs(v(end));
        
        idx_settle = find(abs(theta) > 0.5*pi/180, 1, 'last');
        if isempty(idx_settle)
            t_settle = 0;
        else
            t_settle = t(idx_settle);
        end
        
        % Penalty gioi han
        penalty_u = max(0, max_u - u_max)^2 * 1e8;
        penalty_v = max(0, max_v - v_max)^2 * 1e8;
        penalty_a = max(0, max_a - a_max)^2 * 1e8;
        
        % Penalty nga xe (Su dung truc tiep goc cham gam tinh toan duoc)
        if max_theta >= theta_fall_deg
            penalty_fall = 1e10 + 1e8*(max_theta - theta_fall_deg)^2;
        else
            penalty_fall = 0;
        end
        
        % Cho phep xe di chuyen xa hon de cuu goc 15 do
        if max_x > 3.0
            penalty_position = 1e6*(max_x - 3.0)^2;
        else
            penalty_position = 0;
        end
        
        % Penalty neu cuoi mo phong chua ve can bang
        penalty_final = ...
            1e7*theta_final^2 + ...
            1e5*theta_dot_final^2 + ...
            1e4*v_final^2;
            
        cost = ...
            8000*t_settle + ...
            50000*iae_theta + ...
            5000*iae_theta_dot + ...
            100*iae_x + ...
            100*iae_v + ...
            10*max_u + ...
            100*max_v + ...
            200*max_a + ...
            penalty_final + ...
            penalty_u + ...
            penalty_v + ...
            penalty_a + ...
            penalty_fall + ...
            penalty_position;
    catch
        cost = 1e12;
    end
end

%% ========================================================================
%  HAM MO PHONG LQR CO GIOI HAN U, VAN TOC, GIA TOC
%  ========================================================================
function [t, x, u, acc] = simulate_lqr_limited(A, B, K, x0, t_sim, ...
                                               u_max, v_max, a_max)
    dt = t_sim(2) - t_sim(1);
    N = length(t_sim);
    x = zeros(N, 4);
    u = zeros(N, 1);
    acc = zeros(N, 1);
    
    x(1,:) = x0(:)';
    for k = 1:N-1
        xk = x(k,:)';
        u_raw = -K*xk;
        u_sat = limit_control(A, B, xk, u_raw, u_max, v_max, a_max);
        
        dx = A*xk + B*u_sat;
        acc(k) = dx(3);
        
        x_next = xk + dt*dx;
        % Gioi han van toc tien lui
        x_next(3) = min(max(x_next(3), -v_max), v_max);
        
        x(k+1,:) = x_next';
        u(k) = u_sat;
    end
    xk = x(end,:)';
    u_raw = -K*xk;
    u(end) = limit_control(A, B, xk, u_raw, u_max, v_max, a_max);
    dx = A*xk + B*u(end);
    acc(end) = dx(3);
    t = t_sim(:);
end

%% ========================================================================
%  HAM GIOI HAN TIN HIEU DIEU KHIEN
%  ========================================================================
function u_limited = limit_control(A, B, x, u_raw, u_max, v_max, a_max)
    % Gioi han moment
    u_limited = min(max(u_raw, -u_max), u_max);
    
    a_free = A(3,:)*x;
    b_acc = B(3,1);
    
    a_low = -a_max;
    a_high = a_max;
    
    v = x(3);
    if v >= v_max
        a_high = min(a_high, 0);
    elseif v <= -v_max
        a_low = max(a_low, 0);
    end
    
    if abs(b_acc) > 1e-9
        u1 = (a_low - a_free) / b_acc;
        u2 = (a_high - a_free) / b_acc;
        u_low = min(u1, u2);
        u_high = max(u1, u2);
        u_limited = min(max(u_limited, u_low), u_high);
    end
    
    u_limited = min(max(u_limited, -u_max), u_max);
end