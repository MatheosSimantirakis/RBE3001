
classdef lab2Test
    properties
        mDim;
        mOtherDim;
        links;
        isMoveable;
    end
    methods
        function self = lab2Test()
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            self.links = [96.326, 128, 24, 124, 133.4];
            self.isMoveable = [true false true true true];
        end

        %lab2
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

        function fkOutput = fk3001(self, degrees)
            dhinput = [
                degrees(1) self.links(1) 0 -90;
                -90+degrees(2) 0 self.links(2) 0;
                90 0 self.links(3) 0;
                degrees(3) 0 self.links(4) 0;
                degrees(4) 0 self.links(5) 0
            ];
            fkOutput = self.dh2fk(dhinput);
        end
        %    O
        %  --|--
        %   / \
        function stick_man(self,window,degrees)
            jointNum = size(self.links);
            body = zeros(3,jointNum(2) + 1);
            bodyJointIndex = 2;
            jointIndex = 1;
            currentPoints = self.fk3001(degrees);
            axis([-500 500 -500 500 0 500]);
            for joint = 1:size(self.links(:))
                if self.isMoveable(jointIndex)
                    currentPoint = currentPoints(1:3,4,1,jointIndex);
                    body(:,bodyJointIndex) = currentPoint;
                    bodyJointIndex = bodyJointIndex + 1;
                end
                jointIndex = jointIndex + 1;
            end
            bodyJointIndex = bodyJointIndex - 1;
            set(window,'XData',body(1,1:bodyJointIndex),'YData',body(2,1:bodyJointIndex),'ZData',body(3,1:bodyJointIndex));
        end
        
    end % end methods
end

