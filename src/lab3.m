robot = Robot();
traj = Traj_Planner();
live_plot = lab2_live_plot(robot);
robot.writeMotorState(true);
travelTime = 2;
robot.writeTime(travelTime);
%live_plot.setDisplay(true,false);

%Lab 3 Part 2
% robot.servo_jp(robot.ik3001([(2 + 1) * 25 0 3 * 25 90]));
% pause(travelTime);
% robot.servo_jp(robot.ik3001([(2 + 4) * 25 4 * 25 3 * 25 90]));
% live_plot.plot_during_move(travelTime);
% robot.servo_jp(robot.ik3001([(2 + 4) * 25 -4 *25 3 * 25 90]));
% live_plot.plot_during_move(travelTime);
% robot.servo_jp(robot.ik3001([(2 + 1) * 25 0 3 * 25 90]));
% live_plot.plot_during_move(travelTime);
% robot.servo_jp(robot.ik3001([(2 + 1) * 25 0 3 * 25 90]));
% live_plot.plot_during_move(travelTime);


p1 = [3 * 25 0 3 * 25 90];
p2 = [6 * 25 4 * 25 3 * 25 90];
p3 = [6 * 25 -4 * 25 3* 25 90];

degs1 = robot.ik3001(p1);
degs2 = robot.ik3001(p2);
degs3 = robot.ik3001(p3);
% 
% robot.servo_jp(degs1);
% pause(travelTime);
% robot.servo_jp(degs2);
% live_plot.plot_xyz(travelTime);
% robot.servo_jp(degs3);
% live_plot.plot_xyz(travelTime);
% robot.servo_jp(degs1);
% live_plot.plot_xyz(travelTime);

%Part 5: Trajectory in Joint Space
robot.servo_jp(degs1);
pause(travelTime);
travel = [degs1; degs2; degs3; degs1];
travelPos = [p1; p2; p3; p1];
timeInterval = 2;
startIndex = 1;
startTime = 0;
run = true;
cubic = true;

%Cubic & Quintic Polynomials Used in Joint and Task Space 
if run
result = zeros(5,0);
if cubic
jointSpace  = false;
if jointSpace
for to = 2:4
    from = to - 1;
    deg0 = travel(from,:);
    degf = travel(to,:);
    coef1 = traj.cubic_traj(0,timeInterval,0,0,deg0(1),degf(1));
    coef2 = traj.cubic_traj(0,timeInterval,0,0,deg0(2),degf(2));
    coef3 = traj.cubic_traj(0,timeInterval,0,0,deg0(3),degf(3));
    coef4 = traj.cubic_traj(0,timeInterval,0,0,deg0(4),degf(4));
    coefs = [coef1 coef2 coef3 coef4];
    resultCollected = robot.run_trajectory(coefs,timeInterval,jointSpace,cubic);
    resultCollectedSize = size(resultCollected);
    resultCollected(5,:) = resultCollected(5,:) + startTime;
    endIndex = startIndex + resultCollectedSize(2) - 1;
    result(:,startIndex:endIndex) = resultCollected;
    startIndex = endIndex;
    startTime = resultCollected(5,end);
end
else    
for to = 2:4
    from = to - 1;
    pos0 = travelPos(from,:);
    posf = travelPos(to,:);
    coef1 = traj.cubic_traj(0,timeInterval,0,0,pos0(1),posf(1));
    coef2 = traj.cubic_traj(0,timeInterval,0,0,pos0(2),posf(2));
    coef3 = traj.cubic_traj(0,timeInterval,0,0,pos0(3),posf(3));
    coef4 = traj.cubic_traj(0,timeInterval,0,0,pos0(4),posf(4));
    coefs = [coef1 coef2 coef3 coef4];
    resultCollected = robot.run_trajectory(coefs,timeInterval,jointSpace,cubic);
    resultCollectedSize = size(resultCollected);
    resultCollected(5,:) = resultCollected(5,:) + startTime;
    endIndex = startIndex + resultCollectedSize(2) - 1;
    result(:,startIndex:endIndex) = resultCollected;
    startIndex = endIndex;
    startTime = resultCollected(5,end);
end
end
else
jointSpace  = true;                                     
if jointSpace
for to = 2:4
    from = to - 1;
    deg0 = travel(from,:);
    degf = travel(to,:);
    coef1 = traj.quintic_traj(0,timeInterval,0,10,deg0(1),degf(1),0,0);
    coef2 = traj.quintic_traj(0,timeInterval,10,10,deg0(2),degf(2),0,0);
    coef3 = traj.quintic_traj(0,timeInterval,10,10,deg0(3),degf(3),0,0);
    coef4 = traj.quintic_traj(0,timeInterval,10,0,deg0(4),degf(4),0,0);
    coefs = [coef1 coef2 coef3 coef4];
    resultCollected = robot.run_trajectory(coefs,timeInterval,jointSpace,cubic);
    resultCollectedSize = size(resultCollected);
    resultCollected(5,:) = resultCollected(5,:) + startTime;
    endIndex = startIndex + resultCollectedSize(2) - 1;
    result(:,startIndex:endIndex) = resultCollected;
    startIndex = endIndex;
    startTime = resultCollected(5,end);
end
else  
for to = 2:4
% pause(travelTime);
% robot.servo_jp(degs2);
% live_plot.plot_xyz(travelTime);
% robot.servo_jp(degs3);
% live_plot.plot_xyz(travelTime);
% robot.servo_jp(degs1);
% live_plot.plot_xyz(travelTime);
    from = to - 1;
    pos0 = travelPos(from,:);
    posf = travelPos(to,:);
    coef1 = traj.quintic_traj(0,timeInterval,0,0,pos0(1),posf(1),100,100);
    coef2 = traj.quintic_traj(0,timeInterval,0,0,pos0(2),posf(2),100,100);
    coef3 = traj.quintic_traj(0,timeInterval,0,0,pos0(3),posf(3),100,100);
    coef4 = traj.quintic_traj(0,timeInterval,0,0,pos0(4),posf(4),100,100);
    coefs = [coef1 coef2 coef3 coef4];
    resultCollected = robot.run_trajectory(coefs,timeInterval,jointSpace,cubic);
    resultCollectedSize = size(resultCollected);
    resultCollected(5,:) = resultCollected(5,:) + startTime;
    endIndex = startIndex + resultCollectedSize(2) - 1;
    result(:,startIndex:endIndex) = resultCollected;
    startIndex = endIndex;
    startTime = resultCollected(5,end);
end
end
end
live_plot.plot_Pos_V_Acc(result,true);
% live_plot.plotPath(result);
end


% cubic = true;
% jointSpace = true;
% jointResult = zeros(5,0);
% for to = 2:4
%     from = to - 1;
%     deg0 = travel(from,:);
%     degf = travel(to,:);
%     coef1 = traj.cubic_traj(0,timeInterval,0,0,deg0(1),degf(1));
%     coef2 = traj.cubic_traj(0,timeInterval,0,0,deg0(2),degf(2));
%     coef3 = traj.cubic_traj(0,timeInterval,0,0,deg0(3),degf(3));
%     coef4 = traj.cubic_traj(0,timeInterval,0,0,deg0(4),degf(4));
%     coefs = [coef1 coef2 coef3 coef4];
%     resultCollected = robot.run_trajectory(coefs,timeInterval,jointSpace,cubic);
%     resultCollectedSize = size(resultCollected);
%     resultCollected(5,:) = resultCollected(5,:) + startTime;
%     endIndex = startIndex + resultCollectedSize(2) - 1;
%     jointResult(:,startIndex:endIndex) = resultCollected;
%     startIndex = endIndex;
%     startTime = resultCollected(5,end);
% end
% startIndex = 1;
% startTime = 0;
% taskResult = zeros(5,0);
% jointSpace = false;
% for to = 2:4
%     from = to - 1;
%     pos0 = travelPos(from,:);
%     posf = travelPos(to,:);
%     coef1 = traj.cubic_traj(0,timeInterval,0,0,pos0(1),posf(1));
%     coef2 = traj.cubic_traj(0,timeInterval,0,0,pos0(2),posf(2));
%     coef3 = traj.cubic_traj(0,timeInterval,0,0,pos0(3),posf(3));
%     coef4 = traj.cubic_traj(0,timeInterval,0,0,pos0(4),posf(4));
%     coefs = [coef1 coef2 coef3 coef4];
%     resultCollected = robot.run_trajectory(coefs,timeInterval,jointSpace,cubic);
%     resultCollectedSize = size(resultCollected);
%     resultCollected(5,:) = resultCollected(5,:) + startTime;
%     endIndex = startIndex + resultCollectedSize(2) - 1;
%     taskResult(:,startIndex:endIndex) = resultCollected;
%     startIndex = endIndex;
%     startTime = resultCollected(5,end);
% end
% live_plot.setDisplay(false,true);
% robot.servo_jp(degs1);
% pause(travelTime);
% robot.servo_jp(degs2);
% live_plot.plot_during_move(travelTime);
% robot.servo_jp(degs3);
% live_plot.plot_during_move(travelTime);
% robot.servo_jp(degs1);
% live_plot.plot_during_move(travelTime);

