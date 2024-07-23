
classdef Robot < OM_X_arm
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
    end
% % % % 
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
        end
        
               % Function to read the current (torque) of the motors
        function torques = readCurrent(self)
                c = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT); 
                torques = c * 2.69;
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
                robot = Robot();

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

        % Function to read the current (torque) of the motors
        function torques = readTorques(self)
                c = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT); 
                cur = c * 2.69;
                torques = self.torque(cur);
        end

        function const = torque(self, current)
            c = current / 1000;
            const = ((7/4)*c)-0.175;
        end


        function F = get_EEForce(self, angles)
            jac = self.get_jacobian(angles);
            tor = self.readTorques();
            
            jt = jac';
            F = tor * jt;
            
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

        function set_joint_vars(self, varargin) %function to set joint to travel to desired position, with optional time variable
            sampleArr = rand(1, 2);
            degrees2 = varargin(1);
            degrees = degrees2{1};
            if size(varargin) == size(sampleArr) % if there is a second parameter
                math = varargin(2);
                travelTime = (math{1} / 1000); % sets it to the time in milliseconds while converting into seconds for input
            else
                travelTime = 0; % if second parameter is not passed in, sets the time to quickest possible time to get to position
            end
            self.writeTime(travelTime); % Write travel time
            self.writeMotorState(true); % Write position mode
            %self.writeJoints(0); % Write joints to zero position
            %pause(travelTime); % Wait for trajectory completion
            self.writeJoints(degrees); 
        end

        function output = read_joint_vars(self, GETPOS, GETVEL) %function to choose whether to read both, just position or just velocity
            %readings = self.getJointsReadings();
            if GETPOS && ~GETVEL
                %output(1, :) = readings(1, :);
                output(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG; 
                output(2, :) = zeros(1, 4); 
            elseif ~GETPOS && GETVEL
                output(1, :) = zeros(1, 4);
                %output(2, :) = readings(2, :);
                output(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            else 
                %output(1, :) = readings(1, :);
                output(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
                %output(2, :) = readings(2, :);
                output(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            end
            output = deg2rad(output);
            output(2,2:3) = output(2:2,3) - [1.0791 1.511] ;
        end

         function output = read_joint_curs(self) %function to choose whether to read both, just position or just velocity
             
                c = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT); 
                output = c * 2.69;
             
        end

        function transformMatrix = dh2mat(self, dhParam) %dhParam is Theta, d, a, Alpha and creates transformation matrix of a consecutive joints
            
            transformMatrix = [cos(dhParam(1)), -sin(dhParam(1))*cos(dhParam(4)), sin(dhParam(1))*sin(dhParam(4)), dhParam(3)*cos(dhParam(1));
                               sin(dhParam(1)), cos(dhParam(1))*cos(dhParam(4)), -cos(dhParam(1))*sin(dhParam(4)), dhParam(3)*sin(dhParam(1));
                               0, sin(dhParam(4)), cos(dhParam(4)), dhParam(2);
                               0, 0, 0, 1]; 
        end


        function T = POE_FK(self, params) % params must be [th1 th2 th3]
            theta_1 = params(1);
            theta_2= -params(2)+pi/2;
            theta_3 = -params(3)-pi/2;
            l_1 = 96.326;
            l_2 = 130.231;
            l_3 = 124;

            t_final = [cos(theta_2 + theta_3)*cos(theta_1), -sin(theta_1), -sin(theta_2 + theta_3)*cos(theta_1), cos(theta_1)*(l_3*cos(theta_2 + theta_3) + l_2*cos(theta_2));
                       cos(theta_2 + theta_3)*sin(theta_1),  cos(theta_1), -sin(theta_2 + theta_3)*sin(theta_1), sin(theta_1)*(l_3*cos(theta_2 + theta_3) + l_2*cos(theta_2));
                       sin(theta_2 + theta_3),             0,               cos(theta_2 + theta_3),          l_1 + l_3*sin(theta_2 + theta_3) + l_2*sin(theta_2);
                                 0,             0,                                    0,                                                            1];
 
            T = t_final;
        end


        function jointsMatrix = dh2fk(self, dhTable)  %given dh table, outputs transformation matrices of intermediate and end effector positions
            accumulatorMatrix = eye(4);
            jointsMatrix = zeros(4,4,size(dhTable,1));
            for i = 1:size(dhTable,1)
                accumulatorMatrix = accumulatorMatrix * self.dh2mat(dhTable(i, :)); 
                %disp(self.dh2mat(dhTable(i,:))); 
                jointsMatrix(:,:,i) = accumulatorMatrix;
            end
            %disp(jointsMatrix);
        end

        function pulledJointsMatrix = joints2fk(self, jointDegrees) %given joint angle values, gives you transformation matrices of intermediate and end effector position
            j2 = deg2rad(10.65);
            theta1 = jointDegrees(1);
            theta2 = jointDegrees(2)-j2;
            theta3 = jointDegrees(3)+j2;
            theta4 = jointDegrees(4);
     
            dhTable = [0, 36.076, 0, 0; theta1, 60.25, 0, -(pi/2); theta2 - 1.3854, 0, 130.231, 0; theta3 + 1.3854, 0, 124, 0; theta4, 0, 133.4, 0]; 
         
            pulledJointsMatrix = self.dh2fk(dhTable);

        end
            


        function jacobianMatrix = get_jacobian(self, jointAngleSym)  %gives you jacobian at a certiain position and location given 4 joint angles

            q1 = jointAngleSym(1);
            q2 = jointAngleSym(2);
            q3 = jointAngleSym(3);
            q4 = jointAngleSym(4); 

            Jacobian = [- 124*cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064) - 124*sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1)) - (572761995187585*cos(q2 - 6927/5000)*sin(q1))/4398046511104 - (2845342754596609492674480140935*sin(q2 - 6927/5000)*cos(q1))/356811923176489970264571492362373784095686656 - (667*cos(q4)*(cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064) + sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1))))/5 - (667*sin(q4)*(cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1)) - sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064)))/5, - 124*cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1)) - 124*sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064) - (2845342754596609492674480140935*cos(q2 - 6927/5000)*sin(q1))/356811923176489970264571492362373784095686656 - (572761995187585*sin(q2 - 6927/5000)*cos(q1))/4398046511104 - (667*cos(q4)*(cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1)) + sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064)))/5 - (667*sin(q4)*(cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064) - sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1))))/5, - 124*cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1)) - 124*sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064) - (667*cos(q4)*(cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1)) + sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064)))/5 - (667*sin(q4)*(cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064) - sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1))))/5, - (667*cos(q4)*(cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1)) + sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064)))/5 - (667*sin(q4)*(cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064) - sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1))))/5;
  124*cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064) - 124*sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1)) + (572761995187585*cos(q2 - 6927/5000)*cos(q1))/4398046511104 - (2845342754596609492674480140935*sin(q2 - 6927/5000)*sin(q1))/356811923176489970264571492362373784095686656 + (667*cos(q4)*(cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064) - sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1))))/5 - (667*sin(q4)*(cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064 + sin(q2 - 6927/5000)*cos(q1)) + sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*cos(q1) - (4967757600021511*sin(q2 - 6927/5000)*sin(q1))/81129638414606681695789005144064)))/5,   124*cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1)) - 124*sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064) + (2845342754596609492674480140935*cos(q2 - 6927/5000)*cos(q1))/356811923176489970264571492362373784095686656 - (572761995187585*sin(q2 - 6927/5000)*sin(q1))/4398046511104 + (667*cos(q4)*(cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1)) - sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064)))/5 - (667*sin(q4)*(cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064) + sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1))))/5,   124*cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1)) - 124*sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064) + (667*cos(q4)*(cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1)) - sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064)))/5 - (667*sin(q4)*(cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064) + sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1))))/5,   (667*cos(q4)*(cos(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1)) - sin(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064)))/5 - (667*sin(q4)*(cos(q3 + 6927/5000)*(cos(q2 - 6927/5000)*sin(q1) + (4967757600021511*sin(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064) + sin(q3 + 6927/5000)*((4967757600021511*cos(q2 - 6927/5000)*cos(q1))/81129638414606681695789005144064 - sin(q2 - 6927/5000)*sin(q1))))/5;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 (667*sin(q4)*(cos(q2 - 6927/5000)*sin(q3 + 6927/5000) + cos(q3 + 6927/5000)*sin(q2 - 6927/5000)))/5 - (667*cos(q4)*(cos(q2 - 6927/5000)*cos(q3 + 6927/5000) - sin(q2 - 6927/5000)*sin(q3 + 6927/5000)))/5 - (572761995187585*cos(q2 - 6927/5000))/4398046511104 - 124*cos(q2 - 6927/5000)*cos(q3 + 6927/5000) + 124*sin(q2 - 6927/5000)*sin(q3 + 6927/5000),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           (667*sin(q4)*(cos(q2 - 6927/5000)*sin(q3 + 6927/5000) + cos(q3 + 6927/5000)*sin(q2 - 6927/5000)))/5 - (667*cos(q4)*(cos(q2 - 6927/5000)*cos(q3 + 6927/5000) - sin(q2 - 6927/5000)*sin(q3 + 6927/5000)))/5 - 124*cos(q2 - 6927/5000)*cos(q3 + 6927/5000) + 124*sin(q2 - 6927/5000)*sin(q3 + 6927/5000),                                                                                                                                                                                                                                                                                                                                                                                   (667*sin(q4)*(cos(q2 - 6927/5000)*sin(q3 + 6927/5000) + cos(q3 + 6927/5000)*sin(q2 - 6927/5000)))/5 - (667*cos(q4)*(cos(q2 - 6927/5000)*cos(q3 + 6927/5000) - sin(q2 - 6927/5000)*sin(q3 + 6927/5000)))/5;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    -sin(q1);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     cos(q1);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           4967757600021511/81129638414606681695789005144064,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               4967757600021511/81129638414606681695789005144064,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           4967757600021511/81129638414606681695789005144064];

            jacobianMatrix = Jacobian; 
        end


        function pDot = vel2fdk(self, jointAngles, jointVelocities) %given joint angles and joint velocities, calculates the 6x1 end effector velocities
            J = self.get_jacobian(jointAngles);
            qDot = jointVelocities.'; 
            pDot = J * qDot; 

        end
            

        function endEffectorPos = moveToSamePos(self, iterations, jointsAngles, travelTime) %moves to a certain end effector position for specified iterations, jointangle values and travel time
            for j = 1:iterations
                self.set_joint_vars(jointsAngles, travelTime); %sets travelTime to 0 when no 2nd param passed through
                row1 = self.read_joint_vars(true, false);
                jointPos(j,:) = row1(1, :); 
                allJoints = self.joints2fk(jointPos(j,:));
                endEffector(:,:,j) = allJoints(:,:,5); 
            end
            endEffectorPos = endEffector(:,4,:); 
        end
                
        function T = geoIK(self,endPosition)  %does the inverse kinematics to get joint angles given x,y,z and alpha 
            x = endPosition(1);
            y = endPosition(2);
            z = endPosition(3);
            alpha = endPosition(4);
            L1 = 96.326;
            L2 = 130.231;
            L3 = 124;
            L4 = 133.4;

            theta1 = atan2d(y,x);
            w = sqrt(x^2 + y^2) - L4*cos(alpha);
            q = z -L1 - L3*sin(alpha);
            inte = (-L2^2 -L3^2 +(w^2+q^2))/(2*L2*L3);
            theta3 = acos(inte)-90+10.61;
            alpha_new_frame =  atan2d(q,w);
            Beta = acos((L2^2+w^2+q^2-L3^2)/(2*(L2*sqrt(w^2+q^2))));
            theta2 = 90-alpha_new_frame-Beta-10.61;
            theta4 = -alpha - theta3 -theta2;
            T = [theta1,theta2,theta3,theta4];
        end

        
        function data = runTrajectory(self,coefficent_matrix, duration,task_space) %plots trajectory for both taskspace or jointspace depending on third boolean parameter
            if nargin() < 3 | task_space == false
            tic;
            data =[];
            while toc < duration
                joint_values = [polyval(coefficent_matrix(:,1),toc), ...
                polyval(coefficent_matrix(:,2),toc), ...
                polyval(coefficent_matrix(:,3),toc), ...
                polyval(coefficent_matrix(:,4),toc)];
                disp(joint_values)
                self.set_joint_vars(joint_values);
                position = self.read_joint_vars(true,false);
                data = [data; position(1) position(2) position(3);];
                %pause(0.01);
            end     
        else
            tic;
            data =[];

            while toc < duration
                angles = self.read_joint_vars(true, false);
                task_values = [polyval(coefficent_matrix(:,1),toc), ...
                polyval(coefficent_matrix(:,2),toc), ...
                polyval(coefficent_matrix(:,3),toc),];
                disp(task_values)
                alpha = sum(self.read_joint_vars(true,false));
                joint_values = self.task2ik([task_values, alpha]);
                self.set_joint_vars(joint_values);
                position = self.read_joint_vars(true,false);
                data = [data; position(1) position(2) position(3);];
                %pause(0.01);
            end
            end 
       
        end 
        
        function invert = NRIK(self,desired_target, current_robot_joint_angles) %implements the Newton Raphson IK algorithm
                % desired target is [x y z]
                lamb = .1;
                data = [];
                pasta = self.POE_FK(current_robot_joint_angles);
                pasta = pasta(1:3,4,5);              
                while norm( desired_target - pasta.') > lamb
                    jacob = self.get_jacobian(current_robot_joint_angles);
                    jp = jacob(1:3,:);
                    wak = pinv(jp);
                    m = self.POE_FK(current_robot_joint_angles);
                    a = desired_target-m(1:3,4,5).';
                    walk = wak*a.';
                    current_robot_joint_angles = lamb*walk.' + current_robot_joint_angles;
                    data = [data pasta];
                    a = self.POE_FK(current_robot_joint_angles);
                    pasta = a(1:3,4,5);
                    
                    x = a(1,4,:);
                    y = a(1,4,:);
                    z = a(1,4,:);
                    x = permute(x,[1 3 2]);
                    x = x(:,:);
                    y = permute(x,[1 3 2]);
                    y = y(:,:);
                    z = permute(x,[1 3 2]);
                    z = z(:,:);

%                     plot = plot3(0,0,0);
%                     axis([-300, 300, -300, 300, 0, 500]);
%                     for i = 1:4
%                         line(x(i:i+1),y(i:i+1),z(i:i+1),"Color","red","LineStyle","-","LineWidth",2)
%                     end
                    pause(.1)
                end 
                invert = current_robot_joint_angles;
        end


        function valid = validJacobian(self,jp) %checks if the robot is close to a singularity
            threshold = 1500;
            valid  = det(jp) >threshold;

        end
    end % end methods
end % end class 
