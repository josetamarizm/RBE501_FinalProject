robot = Robot(); 
model = Model(); 

% Step 1: Initialize the video writer
videoFileName = 'final_project.avi';
videoWriter = VideoWriter(videoFileName);
videoWriter.FrameRate = 10; % Set the frame rate
open(videoWriter);

% Live plot of the robot
FiveArbPos = [-15,-35,25,35; 0,0,0,0; 30,15,10,45; -35,-10,-25,-30; 0,0,0,0];
Plot = plot3(0,0,0); %, u, v, w);
axis([-300, 300, -300, 300, 0, 500]); 
xlabel('x Position');
ylabel('y Position');
zlabel('z Position');
title('Stick Plot of Arm');
grid on; 
%text(x,y,z, '.');
for i = 1:5
    model.livePlot(FiveArbPos(i,:), Plot, videoWriter); 
    % Capture the current frame and write it to the video
    frame = getframe(gcf);
    writeVideo(videoWriter, frame);
end

robot.moveToSamePos(1, [0,-40,55,75], 1000);
jointsAngles(1,:) = [0,-15,25,0];
jointsAngles(2,:) = [0,5,45,0];
jointsAngles(3,:) = [0,-40,55,75]; 
endEffector = zeros; 
time = zeros; 
i = 1; 
for j = 1:3
    robot.moveToSamePos(1, jointsAngles(j,:), 1000); 
    tic;
    %i = 1; 
    while toc < 1
        velocityAndJointAngleReadings = robot.read_joint_vars(true, false); 
        jointAngleReadings(i,:) = velocityAndJointAngleReadings(1,:); 
        dispy = robot.POE_FK(jointAngleReadings(i,:)); 
        dispyy = dispy(:,:,5);
        dispyyx = dispyy(1, 4);
        dispyyy = dispyy(2, 4);
        dispyyz = dispyy(3, 4); 
        endEffector(1,i) = dispyyx;
        endEffector(2,i) = dispyyy;
        endEffector(3,i) = dispyyz;
        time(j, i) = toc;
        i = i + 1;
    end
end

% Step 2: Close the video writer
close(videoWriter);
disp('Video recording complete.');

