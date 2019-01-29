function handle=init(vrep,clientID)
handle=struct;
% Handle
joint=[0,0,0,0,0,0];

[res,sphere]=vrep.simxGetObjectHandle(clientID,'Sphere',vrep.simx_opmode_blocking);
[returnCode,]=vrep.simxGetObjectPosition(clientID,sphere,-1,vrep.simx_opmode_streaming);

for i = 1:6
    [res, joint(i)] = vrep.simxGetObjectHandle(clientID, sprintf('UR5_joint%d',i), vrep.simx_opmode_blocking);
    [returnCode, ]=vrep.simxGetJointPosition(clientID,joint(i),vrep.simx_opmode_streaming);
end
    
handle.sphere= sphere;
handle.joint=joint;
end