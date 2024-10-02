%% Setup robot
travelTime = 1; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 
robot.servo_jp(0);
pause(travelTime);
goal = [45,0,0,0];
%robot.interpolate_jp(goal,5000);
baseprofile2 = zeros(500,1);
timing = zeros(500,1);
currentPosition = robot.setpoint_js();
sampleIndex = 1;
timingIndex = 1;
counter = 0;
baseprofile = zeros(63,3);
timing = zeros(63,1);
disp('start')
while counter < 3
    disp('start recording')
    robot.servo_jp(goal);
    tic
    while toc < travelTime
        baseprofile(sampleIndex, counter+1) = currentPosition(1);
        timing(timingIndex, :) = toc * 1000;
        sampleIndex = sampleIndex + 1;
        timingIndex = timingIndex + 1;
        currentPosition = robot.setpoint_js;
    end
    counter = counter + 1;
    sampleIndex = 1;
    disp('end recording')
    robot.servo_jp(0);
    pause(1);
end
disp('completed')
disp('plotting')
hold on
     plot(baseprofile(2:63,1));
     plot(baseprofile(2:63,2));
     plot(baseprofile(2:63,3));
     legend('Profile 1', 'Profile 2', 'Profile 3');
     title('Base Joint Consistency (Non-Interpolation)')
     xlabel('Time (ms)')
     ylabel('Joint Movement in Degrees')
hold off
exportgraphics(gcf,"./export/3baseprofile.png");
clf

previousTime=0;
timeIntervals = zeros(63,1);
j = 1;
for i = transpose(timing)
     timeIntervals(j) = i - previousTime;
     j = j + 1;
     previousTime = i;
 end
 histogram(timeIntervals(2:63));
 title("timing histogram");
 xlabel("Time (ms)");
 exportgraphics(gcf,"./export/3baseTiming.png");
%writematrix(baseprofile2,'~/RBE3001_A23_Team22/baseprofile1.csv');
