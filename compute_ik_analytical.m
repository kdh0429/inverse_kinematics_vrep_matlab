function q_ik= compute_ik_analytical(handle,q_cur,tar_p)

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

T01 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q_cur(1)) -sin(q_cur(1)) 0 0; sin(q_cur(1)) cos(q_cur(1)) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d1; 0 0 0 1];
T12 = [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q_cur(2)) -sin(q_cur(2)) 0 0; sin(q_cur(2)) cos(q_cur(2)) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T23 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 a2; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q_cur(3)) -sin(q_cur(3)) 0 0; sin(q_cur(3)) cos(q_cur(3)) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T34 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 a3; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q_cur(4)) -sin(q_cur(4)) 0 0; sin(q_cur(4)) cos(q_cur(4)) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d4; 0 0 0 1];
T45 = [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q_cur(5)) -sin(q_cur(5)) 0 0; sin(q_cur(5)) cos(q_cur(5)) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d5; 0 0 0 1];
T56 = [1 0 0 0; 0 0 1 0; 0 -1 0 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q_cur(6)) -sin(q_cur(6)) 0 0; sin(q_cur(6)) cos(q_cur(6)) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d6; 0 0 0 1];

T06 = T01 * T12 * T23 * T34 * T45 * T56;

T_fk=T06;
%T_fk=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,0];



% 목표 위치 설정
p = tar_p;
T_fk(:,4)=[p,1];

% 역으로 q1_ik 구하기
p05 = [p(1)-d6*T_fk(1,3), p(2)-d6*T_fk(2,3), p(3)-d6*T_fk(3,3)];
a_1 = atan2(p05(2), p05(1));
a_2 = acos(d4/sqrt(p05(1)^2+p05(2)^2));

count = 1;
q_ik = zeros(8, 6);
for i = 1:2
    for j = 1:2
        for k = 1:2
            % q1 구하기
            q1_ik = a_1 + (-1)^i*a_2 + pi/2;
            %역으로 q5_ik 구하기
            p06 = p;
            q5_ik = (-1)^j* acos((sin(q1_ik)*p(1)-cos(q1_ik)*p(2)-d4)/d6); % 부호 바뀌어도 됨
            
            q6_ik=0;
            if imag(-T_fk(1,2)*sin(q1_ik)+T_fk(2,2)*cos(q1_ik)/sin(q5_ik)) == 0 && imag((T_fk(1,1)*sin(q1_ik)-T_fk(2,1)*cos(q1_ik))/sin(q5_ik)) == 0
                q6_ik = atan2((-T_fk(1,2)*sin(q1_ik)+T_fk(2,2)*cos(q1_ik))/sin(q5_ik),(T_fk(1,1)*sin(q1_ik)-T_fk(2,1)*cos(q1_ik))/sin(q5_ik));
            end
            
            %
            T01_fk = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q1_ik) -sin(q1_ik) 0 0; sin(q1_ik) cos(q1_ik) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d1; 0 0 0 1];
            T46_fk = [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q5_ik) -sin(q5_ik) 0 0; sin(q5_ik) cos(q5_ik) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d5; 0 0 0 1] * [1 0 0 0; 0 0 1 0; 0 -1 0 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cos(q6_ik) -sin(q6_ik) 0 0; sin(q6_ik) cos(q6_ik) 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 d6; 0 0 0 1];
            T14_fk = (T01_fk\T_fk)/T46_fk;
            p14 = T14_fk(1:3,4);
            q3_ik = (-1)^k*acos((p14(1)^2+p14(3)^2-a2^2-a3^2)/(2*a2*a3));
            
            
            temp1Forq2 = atan2(real(p14(3)),real(p14(1))); %???
            temp2Forq2 = asin(a3*sin(q3_ik)/sqrt(p14(1)^2+p14(3)^2));
            q2_ik = temp1Forq2-temp2Forq2;
            X = sqrt(p05(1)^2+p05(2)^2-d4^2);
            
            
            R01f = T01_fk(1:3, 1:3);
            R46f = T46_fk(1:3, 1:3);
            R14 = R01f' * T_fk(1:3,1:3) * R46f';
            R14f = [1 0 0; 0 0 -1; 0 1 0 ]'* R14;
            q234_ik = atan2(real(R14f(2,1)),real(R14f(1,1)));      %논문 참조한 값임. %???
            q4_ik = q234_ik-q2_ik-q3_ik;
            
%             q234_ik=acos((X-p06(1)*cos(q_ik(1))-p06(2)*sin(q_ik(1)))/(d6*sin(q_ik(5))));
%             q4_ik = q234_ik-q2_ik-q3_ik;
            
            %Inverse Kinematics로 구한 값의 배열
            q_ik(count,:) = [q1_ik, q2_ik, q3_ik, q4_ik, q5_ik, q6_ik];
            %T_ik = double(subs(T06, [q1 q2 q3 q4 q5 q6], q_ik(count,:)))
            %error=norm(T_fk-T_ik)
            count = count + 1;
        end
    end
end

end
