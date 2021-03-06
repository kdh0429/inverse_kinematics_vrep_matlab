function  q_ik = compute_ik_jacobian(handle,q_cur,tar_p)
tic

% Iteration 초기 설정
dt=0.001;
maxiter = 30000;
threshold = 1e-4;
kp = 20.0;
kv = 1.0;
% 뻗을 수 있는 최대거리 설정.(ur-5)
R = 0.9281;
% 각 random 한 좌표마다 시간과 iter 횟수를 기록할 배열
iter_count = 0;


if R <norm(tar_p)
    tar_p = tar_p/norm(tar_p)*R;
end

x_d = tar_p;

q_current = q_cur';

%iteration 구문(PI)
while iter_count < maxiter
    T_current = FK(q_current);
    x_current = T_current(1:3, 4);
    
    % step input일 때의 x_dot_des
    x_dot_des = (x_d-x_current)/dt;
    
    x_error(1:3, 1) = x_d - x_current;
    J_ = Jaco(q_current);
    J_inv = J_' / (J_ * J_')
    
    %q_current = q_current + J_inv * (kv*x_dot_des + kp*x_error)*dt;
    %q_current = q_current + J_inv * (x_error);
    q_current = q_current + J_inv * (kv*x_dot_des)*dt;
    
    tol = norm(x_error);
    if tol < threshold
        break;
    end
    iter_count = iter_count + 1;
    
end
% 시간기록
time= toc;


q_ik = q_current;
iter_count;

end

