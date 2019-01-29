vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    fprintf('Connected to %d remote API server',clientID);
    
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait);
    n=vrep.simxSynchronous(clientID, true);
    
    % Handle
    handle=init(vrep,clientID);
    pause(0.5)
    
    cnt = 0;
    hz = 1000;
    x_cur = [0; 0; 0];
    sphere_p_pre = [0, 0, 0];
    
    while(clientID>-1)
        
        % get joint state and target position
        [q_cur, sphere_p]=read(vrep,handle,clientID);
        
        moved_distance = sqrt((sphere_p(1)-sphere_p_pre(1))^2+(sphere_p(2)-sphere_p_pre(2))^2+(sphere_p(3)-sphere_p_pre(3))^2);
       
        T06 = FK(q_cur);
        x_cur = T06(1:3,4);
        
        if (moved_distance > 0.001)
            % set target (start time, end time)
            start_time = (cnt+1)/hz;
            end_time = start_time + 0.5;
            disp('check1')
        end
        
        sphere_p_pre = sphere_p;
        
        % set path with cubic
        tar_p = Cubic(cnt/hz, start_time , end_time, x_cur, [0;0;0], sphere_p', [0;0;0]);
        
        % solve IK
        q_ik = compute_ik_jacobian(handle,q_cur,tar_p);
        
        % set target joint state
        write(vrep, handle, clientID, q_ik);
        
        cnt = cnt + 1 ;
        
        vrep.simxSynchronousTrigger(clientID);
        
    end
    
    vrep.simxFinish(-1);
end

vrep.delete();