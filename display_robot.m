robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 3);

q1 = simulation.q1.signals.values;
q2 = simulation.q2.signals.values;
q3 = simulation.q3.signals.values;

%% Drawing the robot

L1 = 1;
L2 = 1;
L3 = 1;
    
% Adding the link 1
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
joint.HomePosition = q0_1;
setFixedTransform(joint, trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

% Adding the link 2
body = rigidBody('link2');
joint = rigidBodyJoint('joint2', 'revolute');
joint.HomePosition = q0_2;
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

% Adding the link3
body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
joint.HomePosition = q0_3;
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

%Adding the end-effector

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L3, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link3');


% This block of code is needed to create cylinders 
for i = 1:robot.NumBodies -1
   ang = pi/2;
   collisionObj = collisionCylinder(0.05,1);
   mat = axang2tform([0 1 0 ang]);
   mat(1,4) = L1/2;
   collisionObj.Pose =mat;
   addCollision(robot.Bodies{i},collisionObj);
   
end

qs = [q1 q2 q3];
count = length(qs(:,1));
disp(count);

%% Defining the linear trajectory for the display
% t = (0:0.2:10)'; % Time
% start = [cos(pi/6) + cos(2*pi/6) + cos(3*pi/6); sin(pi/6) + sin(2*pi/6) + sin(3*pi/6)];
% finish = [2.5; 0];
% disp(start);
% disp(finish);
% points = [start finish];

%% Defining the circular trajectory for the display
t = (0:0.2:10)'; % Time
center = [1.5 1 0];
radius = 0.5;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];


figure
show(robot,qs(1,:)','Collisions','on','Visuals','off');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
% plot(points(1,:),points(2,:),'k') %this is only for the linear trajectory
plot(points(:,1),points(:,2),'k')
axis([-2 3 -2 3])

pause;

framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:40:count
    show(robot,qs(i,:)','Collisions','on','Visuals','off','PreservePlot',false);
    drawnow
    waitfor(r);
end




