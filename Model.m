classdef Model < Robot

    methods

        function plot_arm(self, jointAngles) %plot_arm plots 3D representation of robot's pose, jointAngles is a 1x4 array representing robot's joint angles
            allJointPos = self.joints2fk(jointAngles); 
            cla('reset')
            xJoints = allJointPos(1, 4, :);
            yJoints = allJointPos(2, 4, :);
            zJoints = allJointPos(3, 4, :); 
            disp(size(xJoints(3)));

            x_inProgress = permute(xJoints,[1 3 2]);
            y_inProgress = permute(yJoints,[1 3 2]);          
            z_inProgress = permute(zJoints,[1 3 2]);
            x = x_inProgress(:,:);
            y = y_inProgress(:,:); 
            z = z_inProgress(:,:);
            u = x(1)/norm(x);
            v = y(1)/norm(y);
            w = z(1)/norm(z);

            Plot = plot3(x(1),y(1),z(1));%, u, v, w);
            text(x,y,z, '.'); 
            for k = 2:5
                pause(0.5);
                xdata = [get(Plot, 'xData') x(k)];
                ydata = [get(Plot, 'yData') y(k)];
                zdata = [get(Plot, 'zData') z(k)];
                set(Plot, 'xData', xdata, 'yData', ydata, 'zData', zdata);
            end
            %quiver3(ax,x,y,z,u,v,w);    
            
        end
        
        

        function set_arm(self, Plot, x, y, z)   %constantly updates the stick plot model
             

            for k = 1:5 
                pause(0.01);

                set(Plot, 'xData', x, 'yData', y, 'zData', z);
                
            end

        end

  function livePlot(self, jointsAngles, Plot, videoWriter) 
            % Method this is a live plot of the arm position in the 
            % jointAngles 
            self.moveToSamePos(1, jointsAngles, 10000); %run position in miliseconds

            tic; % Start timer runs in seconds mov
            k = 1; % initializes frame counter
            while toc < 10 
                jointAndVelReadings = self.read_joint_vars(true, false);
                jointReadings = jointAndVelReadings(1,:);
                allJointPos = self.joints2fk(jointReadings);
                xJoints = allJointPos(1, 4, :);
                yJoints = allJointPos(2, 4, :);
                zJoints = allJointPos(3, 4, :); 

                x_inProgress = permute(xJoints,[1 3 2]);
                y_inProgress = permute(yJoints,[1 3 2]);          
                z_inProgress = permute(zJoints,[1 3 2]);

                x = x_inProgress(:,:);
                y = y_inProgress(:,:); 
                z = z_inProgress(:,:);

                self.set_arm(Plot, x, y, z); %updates the plot in real time
                line()
                
                k = k+1;

                frame = getframe(gcf);
                writeVideo(videoWriter, frame);

            end


 
  end



  function livePlotNonBlocking(self,  Plot) 
            % Method this is alive plot of the arm position in the 
            % jointAngles 
            jointAndVelReadings = self.read_joint_vars(true, true);
            end_vel = self.vel2fdk(jointAndVelReadings(1,:), jointAndVelReadings(2,:));
            jointReadings = jointAndVelReadings(1,:);
            velocit_vec_mm = end_vel(1:3);
            unit_velocity_vec = velocit_vec_mm/norm(velocit_vec_mm);
            unit_velocity_vec = unit_velocity_vec/20
            allJointPos = self.POE_FK(jointReadings);
            xJoints = allJointPos(1, 4, :);
            yJoints = allJointPos(2, 4, :);
            zJoints = allJointPos(3, 4, :); 
            x_inProgress = permute(xJoints,[1 3 2]);
            y_inProgress = permute(yJoints,[1 3 2]);          
            z_inProgress = permute(zJoints,[1 3 2]);
            x = x_inProgress(:,:);
            y = y_inProgress(:,:); 
            z = z_inProgress(:,:);            
            x_vel= [x(:,5),x(:,5)+unit_velocity_vec(1)*1];
            y_vel = [y(:,5),y(:,5)+unit_velocity_vec(2)*1];
            z_vel = [z(:,5),z(:,5)+unit_velocity_vec(3)*1]; 
            quiver3(x_vel(1),y_vel(1),z_vel(1), x_vel(2),y_vel(2),z_vel(2),.05);
            for i = 1:4
                line(x(i:i+1),y(i:i+1),z(i:i+1),"Color","red","LineStyle","-","LineWidth",2)
            end
         end
    end
end