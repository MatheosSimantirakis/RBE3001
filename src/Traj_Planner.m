classdef Traj_Planner
    properties

    end
    
    methods
%         function self = Traj_Planner()
%            
%         end

        %
        % Cubic polynomials
        function coefficents = cubic_traj(self,t0,tf,v0,vf,q0,qf)
            % Cubic polynomial 4x4 matrix *First 2 rows are start time,
            % while Last 2 rows are end time
            Cubic_poly = [1 t0 t0^2 t0^3;
                           0  1  2*t0 3*t0^2;
                           1  tf  tf^2  tf^3;
                           0  1  2*tf   3*tf^2];

            vq_given = [q0 v0 qf vf]';
            % Cubic inverse multiplied by vq_given
            T = Cubic_poly \ vq_given; 
            coefficents = T;
        end
        
        %Lab3 Part7
        function coefficents = quintic_traj(self, t0, tf,v0, vf, q0, qf, a0, af)
            % Quintic polynomial 6x6 matrix *First 3 rows are start time
            % while Last 3 rows are end time
            Quintic_traj = [1 t0 t0^2 t0^3 t0^4 t0^5;
                            0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
                            0 0 2 6*t0 12*t0^2 20*t0^3;
                            1 tf tf^2 tf^3 tf^4 tf^5;
                            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
                            0 0 2 6*tf 12*tf^2 20*tf^3 ];

            vqa_given = [q0 v0 a0 qf vf af]';
            % Quintic inverse multiplied by vq_given
            Tqva = Quintic_traj \ vqa_given;
            coefficents = Tqva;
        end     

    end
end

