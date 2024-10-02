%initialize
disp('initializing')
travelTime = 1;
sempleRate = 63; % observed from previous trial
robot = Robot();
robot.writeTime(travelTime)
robot.writeMotorState(true);
robot.servo_jp(0);
pause(travelTime);
disp('initialized')
%preset goals, initialize the matrix for saving data
goals = [
    -45 0 -80 -45;
    30 -45 20 50;
    -90 -20 45 -60;
    50 -85 -60 45
    ];
positions = zeros(sempleRate*travelTime,5);
sampleIndex = 1;
startTime = 0; %time already passed when one motion start
%start recording
disp('motion start')
for goalT = goals'
    goal = goalT'; %no direct way to loop by column, here's one alternative solution (not best) 
    robot.servo_jp(goal);
    tic
    while toc < travelTime
        positions(sampleIndex,1:4) = robot.setpoint_js;
        positions(sampleIndex,5) = toc + startTime;
        sampleIndex = sampleIndex + 1;
    end
    startTime = startTime + toc;
end
disp('motion completed')
disp('start plotting')
hold on
    plot(positions(:,5), positions(:,1));
    plot(positions(:,5), positions(:,2));
    plot(positions(:,5), positions(:,3));
    plot(positions(:,5), positions(:,4));
    legend('servo1', 'servo2', 'servo3', 'servo4');
    title('4 joint movement tracking');
    xlabel('Time (ms)');
    ylabel('Joint Movement (degree)')
hold off
disp('plotting end')