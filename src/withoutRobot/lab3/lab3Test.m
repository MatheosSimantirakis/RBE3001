classdef lab3Test
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
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = lab3Test()
            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            self.links = [96.326, 128, 24, 124, 133.4];
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

        function T = rotationZ(self,theta)
            redTheta = self.toRed(theta);
            T = [
                cos(redTheta) -sin(redTheta) 0 0;
                sin(redTheta) cos(redTheta) 0 0;
                0 0 1 0;
                0 0 0 1
            ];
        end

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
        
        %=======

        %lab3
 
        % from sine get cosine or from cosine get sine
        function output = sc(self,input)
            output = sqrt(1 - input^2);
        end
        % calculate diagonal length
        function output = diagonal(self,a,b)
            output = sqrt(a^2 + b^2);
        end

        function degrees_ik = ik3001_2(self,ee)
            %constants 
            offsetDegree = atan2(self.mOtherDim(2),self.mOtherDim(1));
            x = ee(1);
            y = ee(2);
            z = ee(3);
            a = self.toRed(ee(4));
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


        
        
    end
end