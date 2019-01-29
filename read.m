function [q_cur,tar_p]=read(vrep, handle,clientID)

q_cur=[0,0,0,0,0,0];
[returnCode,tar_p]=vrep.simxGetObjectPosition(clientID,handle.sphere,-1,vrep.simx_opmode_buffer);

for i=1:6
    [returnCode,q_cur(i)]=vrep.simxGetJointPosition(clientID,handle.joint(i),vrep.simx_opmode_buffer);
end

end