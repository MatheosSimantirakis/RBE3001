classdef lab2_live_plot
    properties
        robot;% the target robot object to be plotted 
        window;% the plot3 object for plotting 
        frameWindow; % the quiver3 object for plot reference frames 
        pathWindow;
        fps;% frame pair second ristriction for making matlab able to animate the plot 
        plotArrays;
    end
    methods
        %constructor
        function self = lab2_live_plot(robotObject)
            self.robot = robotObject;
            title("3D Plot")
            self.window = plot3(0,0,0);
            hold on;
            grid on;
            self.frameWindow = quiver3(0,0,0,0,0,0);
            self.pathWindow = plot3(0,0,0);
            hold off;
            xlabel("X");
            ylabel("Y");
            zlabel("Z");
            axis([-128 386 -386 386 0 483]);
            self.fps = 60000;
            self.plotArrays = livePlotHandle;
        end
        
        function setDisplay(self,body,path)
            self.plotArrays.haveBody = body;
            self.plotArrays.havePath = path;
        end
        %    O
        %  --|--
        %   / \
        % function consume a 1*4 matrix which repreasent the degrees of
        % each joint and generate on frame of the stick model live plot

        function stick_man(self,degrees)
            jointNum = size(self.robot.mDim);
            body = zeros(3,jointNum(2) + 1);
            frames = zeros(6,(jointNum(2) + 1) * 3);
            bodyJointIndex = 2;
            jointIndex = 1;
            frameIndex = 4;
            currentPoints = self.robot.fk3001(degrees);
            frames(1:3,1:3) = eye(3);
            for joint = 1:jointNum(2)
                    currentPoint = currentPoints(1:3,4,1,jointIndex);
                    vectors = currentPoints(1:3, 1:3, 1, jointIndex);
                    body(:,bodyJointIndex) = currentPoint;
                    for vectorIndex = 1:3
                       frames(1:3,frameIndex + vectorIndex - 1) = vectors(:,vectorIndex);
                       frames(4:6,frameIndex + vectorIndex - 1) = currentPoint;
                    end
                    bodyJointIndex = bodyJointIndex + 1;
                    frameIndex = frameIndex + 3;
                    jointIndex = jointIndex + 1;
            end
            self.plotArrays.pathArray(:,self.plotArrays.pathIndex) = currentPoints(1:3,4,1,4);
            self.plotArrays.pathIndex = self.plotArrays.pathIndex + 1;
            path = self.plotArrays.pathArray;
            bodyJointIndex = bodyJointIndex - 1;
            if self.plotArrays.haveBody
                set(self.window,'XData',body(1,1:bodyJointIndex),'YData',body(2,1:bodyJointIndex),'ZData',body(3,1:bodyJointIndex));
                set(self.frameWindow,'XData',frames(4,:),'YData',frames(5,:),'ZData',frames(6,:),'UData',frames(1,:),'VData',frames(2,:),'WData',frames(3,:));
            end
            if self.plotArrays.havePath
                set(self.pathWindow,'XData',path(1,:),'YData',path(2,:),'ZData',path(3,:));
            end
        end
        
        function clearPath(self)
            self.plotArrays.pathArray = zeros(3,0);
            self.plotArrays.pathIndex = 1;
        end

        function plotPath(self,degs)
            resultP = zeros(4,0);
            resultSize = size(degs);
            for i = 1:resultSize(2)
                positions = self.robot.fk3001(degs(1:4,i)');
                resultP(1:3,i) = positions(1:3,4,1,4)';
                resultP(4,i) = degs(5,i);
            end
            plot3(resultP(1,:),resultP(2,:),resultP(3,:));
            title('Path plot');
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            grid on;
        end
        function plot_degrees(self,degs)
            hold on;
            %Joint Degrees Change Graphs
            title("Joint Motion")
            xlabel("Time(s)");
            ylabel("Degree(deg)");
            plot(degs(5,:),degs(1,:),"DisplayName",'Joint 1');
            plot(degs(5,:),degs(2,:),"DisplayName",'Joint 2');
            plot(degs(5,:),degs(3,:),"DisplayName",'Joint 3');
            plot(degs(5,:),degs(4,:),"DisplayName",'Joint 4');
            legend;
            hold on;
        end
        %revert the boolean variable control if add pause between frame for
        %plotting 
        function revertHavePause(self)
            if self.plotArrays.havePause
                self.plotArrays.havePause = false;
            else
                self.plotArrays.havePause = true;
            end
        end

        % call the stick_man() function and do live plot based on the read
        % value of degrees from robot 
        function plot_arm(self)
            self.stick_man(self.robot.setpoint_js);
            if self.plotArrays.havePause
                pause(1/self.fps);
            end
        end
        
        function plot_during_move(self,travelTime)
            tic
            while toc < travelTime
                self.plot_arm;
            end
        end

        function plot_xyz(self,travelTime)
            tipIndex = self.plotArrays.posIndex;
            startIndex = tipIndex;
            tic
            while toc < travelTime
                degs = self.robot.setpoint_js();
                joints = self.robot.fk3001(degs);
                position = joints(1:3,4,1,4);
                self.plotArrays.posPlotArray(1:3,tipIndex) = position;
                self.plotArrays.posPlotArray(4,tipIndex) = toc;
                tipIndex = tipIndex + 1;
            end
            endTime = toc;
            tipIndex = tipIndex - 1;
            self.plotArrays.posPlotArray(4,startIndex:tipIndex) = self.plotArrays.posPlotArray(4,startIndex:tipIndex) + self.plotArrays.startTime;
            self.plotArrays.startTime = self.plotArrays.startTime + endTime;
            self.plotArrays.posIndex = tipIndex;
            clf;
            hold on;
                title("x y z motion");
                xlabel("time (s)");
                ylabel("position (mm)");
                plot(self.plotArrays.posPlotArray(4,:),self.plotArrays.posPlotArray(1,:),"DisplayName",'X');
                plot(self.plotArrays.posPlotArray(4,:),self.plotArrays.posPlotArray(2,:),"DisplayName",'Y');
                plot(self.plotArrays.posPlotArray(4,:),self.plotArrays.posPlotArray(3,:),"DisplayName",'Z');
                %disp(self.plotArrays.posPlotArray);
                legend;
            hold off;
        end

        function self = plot_joints(self,travelTime)
            jointIndex = self.plotArrays.degIndex;
            startIndex = jointIndex;
            disp(startIndex);
            tic
            while toc < travelTime
                degs = self.robot.setpoint_js();
                self.plotArrays.degPlotArray(1:4,jointIndex) = degs;
                self.plotArrays.degPlotArray(5,jointIndex) = toc;
                jointIndex = jointIndex + 1;
            end
            endTime = toc;
            jointIndex = jointIndex - 1;
            disp(self.plotArrays.degPlotArray);
            self.plotArrays.degIndex = jointIndex;
            self.plotArrays.degPlotArray(5,startIndex:jointIndex) = self.plotArrays.degPlotArray(5,startIndex:jointIndex) + self.plotArrays.startTime;
            self.plotArrays.startTime = self.plotArrays.startTime + endTime;
            clf;
            hold on;
                title("joint angles");
                xlabel("time (s)");
                ylabel("joint angle (degs)");
                plot(self.plotArrays.degPlotArray(5,:),self.plotArrays.degPlotArray(1,:),"DisplayName",'joint 1');
                plot(self.plotArrays.degPlotArray(5,:),self.plotArrays.degPlotArray(2,:),"DisplayName",'joint 2');
                plot(self.plotArrays.degPlotArray(5,:),self.plotArrays.degPlotArray(3,:),"DisplayName",'joint 3');
                plot(self.plotArrays.degPlotArray(5,:),self.plotArrays.degPlotArray(4,:),"DisplayName",'joint 4');
                legend;
            hold off;
        end

        function resetPlotArrays(self)
            self.plotArrays.degIndex = 1;
            self.plotArrays.posIndex = 1;
            self.plotArrays.startTime = 0;
        end
    
        %Lab3 Parts 5,6,& 7: Position_Velocity_Acceleration_Graphs
        function plot_Pos_V_Acc(self,degs,onlyPos)
            resultP = zeros(4,0);
            resultSize = size(degs);
            for i = 1:resultSize(2)
                positions = self.robot.fk3001(degs(1:4,i)');
                resultP(1:3,i) = positions(1:3,4,1,4)';
                resultP(4,i) = degs(5,i);
            end
            resultV = diff(resultP,1,2) ./ diff(resultP(4,:));
            resultV(4,:) = resultP(4,1:end - 1);
            resultA = diff(resultV,1,2) ./ diff(resultV(4,:));
            resultA(4,:) = resultV(4,1:end - 1);
            clf;
            if ~onlyPos 
                subplot(3,1,1); 
            end
            hold on;
            title('Position');
                xlabel('time (s)');
                ylabel('position (mm)');
                plot(resultP(4,:),resultP(1,:),'displayName','Px');
                plot(resultP(4,:),resultP(2,:),'displayName','Py');
                plot(resultP(4,:),resultP(3,:),'displayName','Pz');
                legend;
            hold off;
            if ~onlyPos
                subplot(3,1,2);
                hold on;
                    title('Velocity');
                    xlabel('time (s)');
                    ylabel('velocity (mm/s)');
                    plot(resultV(4,:),resultV(1,:),'displayName','Vx');
                    plot(resultV(4,:),resultV(2,:),'displayName','Vy');
                    plot(resultV(4,:),resultV(3,:),'displayName','Vz');
                    legend;
                hold off;
                subplot(3,1,3);
                hold on;
                    title('Acceleration');
                    xlabel('time (s)');
                    ylabel('position (mm/s^2)');
                    plot(resultA(4,:),resultA(1,:),'displayName','Ax');
                    plot(resultA(4,:),resultA(2,:),'displayName','Ay');
                    plot(resultA(4,:),resultA(3,:),'displayName','Az');
                    legend;
                hold off;
            end
        end
    end
end