travelTime = 0.05;
robot = Robot();
robot.writeMotorState(true);
live_plot = lab2_live_plot(robot);
syms t circleX circleY circleZ;
unit = 25;
r = 2.5 * unit;
k = 5 * unit;
h = 0;
n = 10;
z = 5 * unit;
a = 90;
circleX(t) = r * cos(t) + k;
circleY(t) = r * sin(t) + h;
circleZ(t) = n * t + z;
positions = zeros(0,4);
trajectoryNumber = 100;
dth = 0;
for i = 1:trajectoryNumber
    dth =dth + pi / trajectoryNumber * 2;
    positions(i,1) = circleX(dth);
    positions(i,2) = circleY(dth);
    positions(i,3) = circleZ(sin(dth));
    positions(i,4) = a;
end
clf;
pause(travelTime);
result = zeros(5,0);
for to = 2:trajectoryNumber
    from = to - 1;
    pos0 = positions(from,:);
    posf = positions(to,:);
    coefs = zeros(6,4);
    for i = 1:4
        coefs(:,i) = traj.quintic_traj(0,travelTime,0,0,pos0(i),posf(i),0,0);
    end
    resultCollected = robot.run_trajectory(coefs,travelTime,false,false);
    resultCollectedSize = size(resultCollected);
    resultCollected(5,:) = resultCollected(5,:) + startTime;
    endIndex = startIndex + resultCollectedSize(2) - 1;
    result(:,startIndex:endIndex) = resultCollected;
    startIndex = endIndex;
    startTime = resultCollected(5,end);
end
live_plot.plotPath(result);