% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        links; % dimentions been selected for calculation 
        rotationAroundZ % Stores if the joints rotation around Z axis true as yes, false as no
        currentSetPoints; % Stores latest set point
        isMoveable;
        dh;
        trajP; 
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);

            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            self.links = [96.326, 128, 24, 124, 133.4];

            %Lab 3: Calling Class Traj_Planner.m
            traj = Traj_Planner();
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            disp("time")
            disp(time_ms)
            disp("acc time")
            disp(acc_time_ms)
            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end

        %lab 1 edits

        % Sends a 1x4 array of four joint values in degrees to the robot bypassing interpolation
        % values [1x4 double] - joint angles for each of the joints to travel to
        function servo_jp(self, values)
            self.currentSetPoints = values;
            self.writeJoints(values);
        end
        

        % Sends a 1x4 array of four joint values in degrees to the robot with an 
        % interpolation time in milliseconds
        % values [1x4 double] - joint angles for each of the joints to travel to
        % time [double] - interpolation time in milliseconds
        function interpolate_jp(self, values, time)
            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, time/3);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time);
            self.servo_jp(values);
        end

        % Creates a 2x4 array of the current joint positions in the first row ofand/or the current joint values as requested
        % GETPOS [boolean] - true to collect position values, false to return zeros for position
        % GETVEL [boolean] - true to collect velocity values, false toreturn zeros for velocity
        
        % Creates a 2x4 array of the current joint positions in the first:size(dhInpu
        % row of and/or the current joint values as requested
        % GETPOS [boolean] - true to collect position values, false to return
        % zeros for position
        % GETVEL [boolean] - true to collect velocity:size(dhInpu
        % values, false toreturn zeros for velocity

        function result = measured_js(self, GETPOS,GETVEL)
            result = zeros(2,4);

            if GETPOS
                result(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end

            if GETVEL
                result(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            end
        end

        % Creates a 1x4 array of of current joint setpoints in degrees
        % read [1x4 interger] - current joint setpoint positions
        function read = setpoint_js(self)
             read(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
        end
        

        %function goals = goal_js(self,goals)

        % Creates a 1x4 array of target joint setpoints in degrees
        function goals = goal_js(self)
            goals = self.currentSetPoints;
        end
        
        %lab2
        % section2 code start 
        %helper functions 
        function red = toRed(self,deg)
            red = deg/180 * pi;
        end

        function deg = toDeg(self,red)
            deg = red/pi * 180;
        end

        % Eular Angle for Z rotation
        function T = rotationZ(self,theta)
            redTheta = self.toRed(theta);
            T = [
                cos(redTheta) -sin(redTheta) 0 0;
                sin(redTheta) cos(redTheta) 0 0;
                0 0 1 0;
                0 0 0 1
            ];
        end
        
        % Eular Angle for X rotation
        function T = rotationX(self,theta)
            redTheta = self.toRed(theta);
            T = [
                1 0 0 0;
                0 cos(redTheta) -sin(redTheta) 0;
                0 sin(redTheta) cos(redTheta) 0;
                0 0 0 1;
            ];
        end

        %create 4x4 homogeneous transformation matrix for given link
        function dhmat = dh2mat(self,theta,d,a,alpha)
            dhlinear = [1 0 0 a; 
                        0 1 0 0; 
                        0 0 1 d;
                        0 0 0 1];
            dhmat = self.rotationZ(theta) * dhlinear * self.rotationX(alpha);
        end

        %takes in an nx4 matrix and return the final transformation 
        function DHtransformations = dh2fk(self, dhInput)
            dhmat = [1 0 0 0;
                     0 1 0 0;
                     0 0 1 0;
                     0 0 0 1];
            inputSize = size(dhInput);
            DHtransformations = zeros(4,4,1,inputSize(2));
            for dh = 1:size(dhInput)
                dhmat = dhmat * self.dh2mat(dhInput(dh,1),dhInput(dh,2),dhInput(dh,3),dhInput(dh,4)); %pass value of dh table in order
                DHtransformations(:,:,1,dh) = dhmat;
            end
        end

        % function consume degree value and out put the 4*4*1*4 matrix
        % repreasent the dh transformation from the base frame to each
        % joint 
        function fkOutput = fk3001(self, degrees)
            offsetDegree = self.toDeg(atan2(self.mOtherDim(1),self.mOtherDim(2)));
            dhinput = [
                degrees(1) self.mDim(1) 0 -90;
                degrees(2)-offsetDegree 0 self.mDim(2) 0;
                degrees(3)+offsetDegree 0 self.mDim(3) 0;
                degrees(4) 0 self.mDim(4) 0;
            ];
            fkOutput = self.dh2fk(dhinput);
        end
        %section 2 code end
        %section 3 code start
        function measured_cp = measured_cp(self)
            result_measured_js = self.measured_js(true,false);
            measured_cp = self.fk3001(result_measured_js);
        end

        % Takes setpoint_js() returns 4x4 homogneous transormation on the
        % current joint set point in degrees
        function current_setPoint = setpoint_cp(self)
            result_SetPoint = self.setpoint_js();
            current_setPoint = self.fk3001(result_SetPoint);
        end

        %Takes goal_js() returns 4x4 homogneous transormtion 
        function cp_goal = goal_cp(self)
            result_Goal = self.goal_js();
            cp_goal = self.fk3001(result_Goal);
        end     
        %section 3 code end

        %=======

        %lab3
        function degrees_ik = ik3001(self,ee)
            %constants 
            offsetDegree = atan2(self.mOtherDim(2),self.mOtherDim(1));
            x = ee(1);
            y = ee(2);
            z = ee(3);
            a = self.toRed(ee(4));
            %Lengths
            l1 = self.mDim(1);
            l2 = self.mDim(2);
            l3 = self.mDim(3);
            l4 = self.mDim(4);
            th0 = atan2(y,x);

            %s r w
            s = z - l1;
            r = sqrt(x^2 + y^2);

            %minus offset from first joint 
            ds = sin(a) * l4;
            dr = cos(a) * l4;
            s = s + ds;
            r = r - dr;
            
            % start ik calculation
            w = sqrt(s^2 + r^2);
            b3 = atan2(s,r);
            cb1 = (l2^2 + l3^2 - (s^2 + r^2)) / (2 * l2 * l3);
            sb1 = sqrt(1-cb1^2);
            disp(sb1);
            b1 = atan2(sb1,cb1);
            th2 = pi/2 + offsetDegree - b1;
            sb2 = sb1 * l3 / w;
            cb2 = sqrt(1-sb2^2);
            b2 = atan2(sb2,cb2);
            th1 = pi/2 - (b2 + b3 + offsetDegree);
            th4 = a - th1 - th2;
            degrees_ik = [self.toDeg(th0) self.toDeg(th1) self.toDeg(th2) self.toDeg(th4)]; 
         end

%          function traj_run = run_trajectory(self,traj, cubic_traj)
%              Traj_Coefficient = [self.ik3001(1)]
%          end 

        %Traj Movment 
        function traj_record = run_trajectory(self,coefficents, tsum, Joint_Space, isCubic)
            dt = 0.0005;
            self.writeTime(dt,0);
            traj_record = zeros(5,0);
            trajIndex = 1;
            tic
            while toc < tsum
                t = toc;
                coefIndex = 1;
                degrees = zeros(1,4);
                orientations = zeros(1,4);
                if Joint_Space % jointSpace
                    % polynomials summed up
                    for coef = coefficents
                        coefT = coef';
                        if isCubic
                            degree = coefT(1) + coefT(2)*t + coefT(3)*t^2 + coefT(4)*t^3;
                        else
                            degree = coefT(1) + coefT(2)*t + coefT(3)*t^2 + coefT(4)*t^3 + coefT(5)*t^4 + coefT(6)*t^5;
                        end
                        degrees(coefIndex) = degree;
                        % Next data entery
                        coefIndex = coefIndex + 1;
                    end
                else % taskSpace
                    for coef = coefficents
                        coefT = coef';
                        if isCubic
                            orientation = coefT(1) + coefT(2)*t + coefT(3)*t^2 + coefT(4)*t^3;
                        else
                            orientation = coefT(1) + coefT(2)*t + coefT(3)*t^2 + coefT(4)*t^3 + coefT(5)*t^4 + coefT(6)*t^5;
                        end
                        orientations(coefIndex) = orientation;
                        coefIndex = coefIndex + 1;
                    end
                    degrees = self.ik3001(orientations);
                end
                self.servo_jp(degrees);
                pause(dt);
                traj_record(5,trajIndex) = t;
                traj_record(1:4,trajIndex) = degrees;
                trajIndex = trajIndex + 1;
            end
        end % end traj_record
    end % end methods
end % end class 
