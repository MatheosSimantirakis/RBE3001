travelTime = 1;
robot = Robot();
robot.writeTime(travelTime);
robot.writeMotorState(true);
robot.servo_jp([0 -90 90 0]);
pause(travelTime);

% robot.servo_jp([0 0 0 0]);
% pause(travelTime);
% disp("0 0 0 0 result:");
% disp("actual degree: " + robot.measured_js(true,false));
% result=robot.measured_cp;
% disp(result);
% robot.servo_jp([10 10 10 10]);
% pause(travelTime);
% disp("10 10 10 10 result:");
% disp("actual degree: " + robot.measured_js(true,false));
% result=robot.measured_cp;
% disp(result);
% robot.servo_jp([-5 -10 -5 -10])
% pause(travelTime);
% disp("-5 -10 -5 -10 result:");
% disp("actual degree: " + robot.measured_js(true,false));
% result=robot.measured_cp;
% disp(result);

% goals = [
%     -45 0 -80 -45;
%     30 -45 20 50;
%     -90 -20 45 -60;
%     50 -85 -60 45;
%     10 10 10 10;
%     ];


% tip_positions = zeros(4,4,1,5);
% recordIndex = 1;
% disp('motion start')
% for goalT = goals'
%     goal = goalT'; %no direct way to loop by column, here's one alternative solution (not best) 
%     robot.servo_jp(0);
%     pause(travelTime);
%     joints = robot.measured_cp;
%     tip_positions(:,:,1,recordIndex) = joints(:,:,1,5);
%     recordIndex = recordIndex + 1;
%     robot.servo_jp(goal);
%     pause(travelTime);
% end


% disp('motion completed')
% disp('start plotting')
% hold on 
%     for i = 1:5
%         current_position = tip_positions(:,:,1,i);
%         transformation = current_position(1:3,4);
%         transformationT = transformation';
%         plot3(transformation(1),transformation(2),transformation(3),'o');
%     end
%     legend();
%     title("Tip Position 1 to 5")
% hold off


%live_plot = lab2_live_plot(robot);
% live_plot.plot_arm();
% robot.servo_jp([0 0 0 0]);
% tic
% while toc < travelTime
%     live_plot.plot_arm();
% end


% increment = 5;
% points = zeros(3,(180/increment)^4);
% pointIndex = 1;
% for d1 = -90:increment:90
%     for d2 = -90:increment:90
%         for d3 = -90:increment:90
%             for d4 = -90:increment:90
%                 currentJoints = robot.fk3001([d1 d2 d3 d4]);
%                 tipPoint = currentJoints(1:3,4,1,4);
%                 points(:,pointIndex) = tipPoint;
%                 pointIndex = pointIndex + 1;
%             end
%         end
%     end
% end
% Cloud = pointCloud(points');
% pcshow(Cloud);


%Lab2 Part 6; Poses
% move_and_live_plot([40,20,20,20],live_plot,robot,travelTime);
% pause(5)
% move_and_live_plot([-45,-25,-45,45],live_plot,robot,travelTime);
% pause(5)
% move_and_live_plot([-60,-35,-30,90],live_plot,robot,travelTime);
% pause(5)
% move_and_live_plot([-60,25,-45,-90],live_plot,robot,travelTime);
% pause(5)
% move_and_live_plot([-25,30,-45,100],live_plot,robot,travelTime);
% angles = zeros(4,travelTime*63*4);
% angles(:,1:63*travelTime) = move_and_live_plot([0,0,0,0],live_plot,robot,travelTime);
% angles(:,63*travelTime+1:63*2*travelTime)=move_and_live_plot([0,0,-45,0],live_plot,robot,travelTime);
% angles(:,63*2*travelTime+1:63*3*travelTime)=move_and_live_plot([0,-40,40,0],live_plot,robot,travelTime);
% angles(:,63*3*travelTime+1:63*4*travelTime)=move_and_live_plot([0,10,-10,0],live_plot,robot,travelTime);



%  anglesSize = size(angles);
%  positions = zeros(3,anglesSize(2));
%  for i = 1:anglesSize(2)
%      position = robot.fk3001(angles(:,i)');
%      positions(:,i) = position(1:3,4,1,4);
%  end
% hold on;
%  times = 0:1/63:travelTime*4;
% plot(times(2:253),positions(1,:),'DisplayName','X');
% plot(times(2:253),positions(3,:),'displayName','Z');
% legend;
% hold off;
%  function angle = move_and_live_plot(degrees,lp,rb,tt)
%      lp.plot_arm();
%      rb.servo_jp([degrees(1) degrees(2) degrees(3) degrees(4)]);
%       angle = zeros(4,tt*63);
%      tic
%      i=1;
%      while toc < tt
%          lp.plot_arm();
%          rb_degs = rb.setpoint_js;
%          angle(:,i) = rb_degs';
%          i = i + 1;
%      end
%      disp(rb.setpoint_js);
%  end


