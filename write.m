% for analytical
% function y=write(vrep,handle, clientID,q_ik)
%         vrep.simxSetJointTargetPosition(clientID,handle.joint(1),real(q_ik(1,1)),vrep.simx_opmode_streaming);
%         vrep.simxSetJointTargetPosition(clientID,handle.joint(2),real(q_ik(1,2)),vrep.simx_opmode_streaming);
%         vrep.simxSetJointTargetPosition(clientID,handle.joint(3),real(q_ik(1,3)),vrep.simx_opmode_streaming);
%         vrep.simxSetJointTargetPosition(clientID,handle.joint(4),real(q_ik(1,4)),vrep.simx_opmode_streaming);
%         vrep.simxSetJointTargetPosition(clientID,handle.joint(5),real(q_ik(1,5)),vrep.simx_opmode_streaming);
%         vrep.simxSetJointTargetPosition(clientID,handle.joint(6),real(q_ik(1,6)),vrep.simx_opmode_streaming);
% end

% for jacobian
function y=write(vrep,handle, clientID,q_ik)
        vrep.simxSetJointTargetPosition(clientID,handle.joint(1),real(q_ik(1)),vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetPosition(clientID,handle.joint(2),real(q_ik(2)),vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetPosition(clientID,handle.joint(3),real(q_ik(3)),vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetPosition(clientID,handle.joint(4),real(q_ik(4)),vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetPosition(clientID,handle.joint(5),real(q_ik(5)),vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetPosition(clientID,handle.joint(6),real(q_ik(6)),vrep.simx_opmode_streaming);
end
