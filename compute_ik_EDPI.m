function q_ik= compute_ik_EDPI(handle,q_cur,tar_p)
tic

syms q1 q2 q3 q4 q5 q6

d1 = 0.0746;
a2 = 0.4251;
a3 = 0.3921;
d4 = 0.1100;
d5 = 0.0948;
d6 = 0.0623;

%         T = Rot(alpha) * Trans(a) * Rot(q) * Trans(d)
%           = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1] *
%             [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1] *
%             [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1] *
%             [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1]

T01 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q1) -sin(q1) 0 0; sin(q1) cos(q1) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d1; 0 0 0 1];
T12 = [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q2) -sin(q2) 0 0; sin(q2) cos(q2) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T23 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 a2; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q3) -sin(q3) 0 0; sin(q3) cos(q3) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T34 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 a3; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q4) -sin(q4) 0 0; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d4; 0 0 0 1];
T45 = [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q5) -sin(q5) 0 0; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d5; 0 0 0 1];
T56 = [1 0 0 0; 0 0 1 0; 0 -1 0 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q6) -sin(q6) 0 0; sin(q6) cos(q6) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d6; 0 0 0 1];

T06 = T01 * T12 * T23 * T34 * T45 * T56;
T06 = simplify(T06);

T(1:3, 1) = T06(1:3, 4);
J0 = jacobian(T, [q1, q2, q3, q4, q5, q6]);
J = matlabFunction(J0, 'Vars', {[q1; q2; q3; q4; q5; q6]});
T06 = matlabFunction(T06, 'Vars', {[q1; q2; q3; q4; q5; q6]});


% Iteration 초기 설정
dt=0.001;
maxiter = 30000;
threshold = 1e-4;
kp = 20.0;
kv = 0.3;
% 뻗을 수 있는 최대거리 설정.(ur-5)
R = sqrt((a2 + a3 + d5)^2 + (d4 + d6)^2);
% 각 random 한 좌표마다 시간과 iter 횟수를 기록할 배열
iter_count = 0;


if R <norm(tar_p)
    tar_p = tar_p/norm(tar_p)*R;
end

x_d = tar_p';

% 매 cycle 마다 q_current 초기화하기
q_current = q_cur';

%iteration 구문(PI)
while iter_count < maxiter
    T_current = T06(q_current);
    x_current = T_current(1:3, 4);
    
    % step input일 때의 x_dot_des
    x_dot_des = (x_d-x_current)/dt;
    
    x_error(1:3, 1) = x_d - x_current;
    J_ = J(q_current);
    
    J_EDPI = J_' / (J_ * J_'+0.5*(x_error'*x_error)*eye(3));
    
    q_current = q_current + J_EDPI * (kv*x_dot_des)*dt;
    
    tol = norm(x_error);
    if tol < threshold
        break;
    end
    iter_count = iter_count + 1;
    
end
% 시간기록
time= toc


q_ik = q_current;
iter_count;

end

