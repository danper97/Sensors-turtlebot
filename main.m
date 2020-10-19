close;
clear;
clc;

%% Can Starting Positions

r = [-0.4 0.2 1.14];
g = [-0.5 0 1.14];
b = [-0.6 -0.2 1.14];

% Create starting can transforms
redStart = transl(r);
greenStart = transl(g);
blueStart = transl(b);

% Transforms for Robot Use
redPos = transl(r)*troty(pi/2)*trotx(pi/2)*trotz(pi);
greenPos = transl(g)*troty(pi/2)*trotx(pi/2)*trotz(pi);
bluePos = transl(b)*troty(pi/2)*trotx(pi/2)*trotz(pi);

%% Create Environment, Robots, and Objects

set(figure(1), 'WindowStyle', 'Docked');

% Create Environment
env = Environment;
env.generateObjects(env);

% Plot Collision boxes
% col = Collision;

% Create Robot
robot = KinovaGen3;

% Teach Functionality
% robot.model.teach;

% Can Objects in Environment
redCanLocation = redCan(redStart);
greenCanLocation = greenCan(greenStart);
blueCanLocation = blueCan(blueStart);

hold on;
axis equal;
view(3);

%% Get user to select colour configuration

str = input('Which colour should go on top? (Red, Green, Blue)? ','s');

[redGoalCo, greenGoalCo, blueGoalCo] = selectColour(str);

redGoalPose = transl(redGoalCo)*troty(pi/2)*trotz(pi);
greenGoalPose = transl(greenGoalCo)*troty(pi/2)*trotz(pi);
blueGoalPose = transl(blueGoalCo)*troty(pi/2)*trotz(pi);
%% Boxes
        tableC = [0 0 0.55];
        tableS = [2 1.5 1.1];
        
        % Shelf1
        shelf1C = [0 -.7 1.4];
        shelf1S = [0.8 0.2 0.02];
        % Shelf2
        shelf2C = [0 -.7 1.4255];
        shelf2S = [0.8 0.2 0.02];
        
        % Shelf3
        shelf3C = [0 -.7 1.6925];
        shelf3S = [0.8 0.2 0.02];
        
        % Shelf4
        shelf4C = [0 -.7 1.9595];
        shelf4S = [0.8 0.2 0.02];
        
        % Shelf5
        shelf5C = [0 -.7 2.2265];
        shelf5S = [0.8 0.2 0.02];
        
plotOptions.plotFaces = true;
[vertex1,faces1,Normal1] = RectangularPrism(tableC + tableS/2, tableC - tableS/2,plotOptions);
[vertex2,faces2,Normal2] = RectangularPrism(shelf1C + shelf1S, shelf1C - shelf1S,plotOptions);
[vertex3,faces3,Normal3] = RectangularPrism(shelf2C + shelf2S, shelf2C - shelf2S,plotOptions);
[vertex4,faces4,Normal4] = RectangularPrism(shelf3C + shelf3S, shelf3C - shelf3S,plotOptions);
[vertex5,faces5,Normal5] = RectangularPrism(shelf4C + shelf4S, shelf4C - shelf4S,plotOptions);
[vertex6,faces6,Normal6] = RectangularPrism(shelf5C + shelf4S, shelf5C - shelf5S,plotOptions);
axis equal
%% Initial Values

q0 = [0 0 0 0 0 0];
steps = 64;

%% Animation
r = [-1 0 1.14];
g = [-1 0.3 1.14];
b = [1 -0.4 1.14];
q0 = [0 0 0 0 0 0];
rad = [ -0.5027 -0.3456 2.1669 0.1160 -0.2939 0.1257;
    -3.2776 0 2.5796 0.0704 1.26 3.3141;
    0.0000 -0.3456 2.1669 0 0 0;
    0.0219 -.1580 -1.5796 -0.0079 .1 -0.0262;
    .1397 -1 .9 .17 -1.36 .04;
    0.3246 -0.6283 1.7025 0.1160 -0.2939 0.1257];

current = robot.model.fkine(q0);
robot.model.animate(q0)
% Red Can Sequence
positions = [current(1,4),current(2,4),current(3,4);
             r(1),r(2),r(3);       %Red Can
             redGoalCo(1),redGoalCo(2),redGoalCo(3);      %Final Red Can
             g(1),g(2),g(3);       %Blue Can
              greenGoalCo(1),greenGoalCo(2),greenGoalCo(3);        %Final Blue Can
              b(1),b(2),b(3);      %Green Can
              blueGoalCo(1),blueGoalCo(2),blueGoalCo(3)];      %Final Green Can
   
for k = 1:6
q0 = robot.model.getpos;
q1 = robot.model.ikcon(transl(positions(k+1,:)),q0);
% q1 = rad(k,:);
x = positions(k,1);
y = positions(k,2);
z = positions(k,3);
fx = (positions(k+1,1)-positions(k,1))/10;
fy = (positions(k+1,2)-positions(k,2))/10;
fz = (positions(k+1,3)-positions(k,3))/10;
qWaypoints = [q0 ; robot.model.ikcon(transl(x,y,z),q0)];
[~,all] = robot.model.fkine(qWaypoints(end,:));
for j = 1:6
    for i = 1 : size(all,3)-1
        for faceIndex1 = 1:size(faces1,1)
            futureWaypoints = [qWaypoints ; robot.model.ikcon(transl(x+fx,y+fy,z+fz),qWaypoints(end,:))];
            [~,newall] = robot.model.fkine(futureWaypoints(end,:));
            vertOnPlane1 = vertex1(faces1(faceIndex1,1)',:);
            vertOnPlane2 = vertex2(faces2(faceIndex1,1)',:);
            vertOnPlane3 = vertex3(faces3(faceIndex1,1)',:);
            vertOnPlane4 = vertex4(faces4(faceIndex1,1)',:);
            vertOnPlane5 = vertex5(faces5(faceIndex1,1)',:);
            vertOnPlane6 = vertex6(faces6(faceIndex1,1)',:);
            [intersects1,check1]=LinePlaneIntersection(Normal1(faceIndex1,:),vertOnPlane1,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects2,check2]=LinePlaneIntersection(Normal2(faceIndex1,:),vertOnPlane2,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects3,check3]=LinePlaneIntersection(Normal3(faceIndex1,:),vertOnPlane3,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects4,check4]=LinePlaneIntersection(Normal4(faceIndex1,:),vertOnPlane4,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects5,check5]=LinePlaneIntersection(Normal5(faceIndex1,:),vertOnPlane5,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects6,check6]=LinePlaneIntersection(Normal6(faceIndex1,:),vertOnPlane6,newall(1:3,4,i)',newall(1:3,4,i+1)');
            
            if (check1 == 1 && IsIntersectionPointInsideTriangle(intersects1,vertex1(faces1(faceIndex1,:)',:))) || (check2 == 1 && IsIntersectionPointInsideTriangle(intersects2,vertex2(faces2(faceIndex1,:)',:))) || (check3 == 1 && IsIntersectionPointInsideTriangle(intersects3,vertex3(faces3(faceIndex1,:)',:)))...
                    || (check4 == 1 && IsIntersectionPointInsideTriangle(intersects4,vertex4(faces4(faceIndex1,:)',:)))|| (check5 == 1 && IsIntersectionPointInsideTriangle(intersects5,vertex5(faces5(faceIndex1,:)',:)))|| (check6 == 1 && IsIntersectionPointInsideTriangle(intersects6,vertex6(faces6(faceIndex1,:)',:)))
                disp('Avoiding Collision')
                x = x-.2;
                y = y+.2;
                z = z+.2;
            end
        end
    end
    qWaypoints = [qWaypoints; robot.model.ikcon(transl(x,y,z),qWaypoints(end,:))];
    x=x+fx;
    y=y+fy;
    z =z+fz;

end
qWaypoints = [qWaypoints; robot.model.ikcon(transl(x,y,z),q1)];
% RMRC
% steps = size(qWaypoints,1);
steps = 20;
deltaT = 0.05;
x0 = zeros(3,steps);
s = lspb(0,1,steps);
delta = 2*pi/steps;
epsilon = 0.1; 
W = diag([1 1 1 0.1 0.1 0.1]); 
m = zeros(steps,1);                  
qdot = zeros(steps,6);          
theta = zeros(3,steps);                   
positionError = zeros(3,steps);
angleError = zeros(3,steps); 
x1 = robot.model.fkine(q0);
x2 = robot.model.fkine(qWaypoints(end,:));
for i = 1:steps
    x0(:,i) = x1(1:3,4)*(1-s(i)) + s(i)*x2(1:3,4);                 
    theta(1,i) = 0;               
    theta(2,i) = 5*pi/9;            
    theta(3,i) = 0;
end

for i = 1:steps-1
    T = robot.model.fkine(real(qWaypoints(i,:))); 
    deltaX = x0(:,i+1) - T(1:3,4);  
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     
    Ra = T(1:3,1:3);                                                        
    Rdot = (1/deltaT)*(Rd - Ra);                                               
    S = Rdot*Ra';                                                           
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              
    xdot = W*[linear_velocity;angular_velocity];                          
    J = robot.model.jacob0(real(qWaypoints(i,:)));                 
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   
    qdot(i,:) = (invJ*xdot)';                                                
    for j = 1:6                                                             
        if qWaypoints(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                    
            qdot(i,j) = 0; % Stop the motor
        elseif qWaypoints(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qWaypoints(i+1,:) = real(qWaypoints(i,:)) + deltaT*real(qdot(i,:));                         	
    positionError(:,i) = x0(:,i+1) - T(1:3,4);                                               
end

% robot.model.animate(qWaypoints);
robot.model.animate(qWaypoints);

[~,all] = robot.model.fkine(q1);
for i = 1 : size(all,3)-1
    for faceIndex1 = 1:size(faces1,1)
        vertOnPlane1 = vertex1(faces1(faceIndex1,1)',:);
        vertOnPlane2 = vertex2(faces2(faceIndex1,1)',:);
        vertOnPlane3 = vertex3(faces3(faceIndex1,1)',:);
        vertOnPlane4 = vertex4(faces4(faceIndex1,1)',:);
        vertOnPlane5 = vertex5(faces5(faceIndex1,1)',:);
        vertOnPlane6 = vertex6(faces6(faceIndex1,1)',:);
        [intersects1,check1]=LinePlaneIntersection(Normal1(faceIndex1,:),vertOnPlane1,newall(1:3,4,i)',newall(1:3,4,i+1)');
        [intersects2,check2]=LinePlaneIntersection(Normal2(faceIndex1,:),vertOnPlane2,newall(1:3,4,i)',newall(1:3,4,i+1)');
        [intersects3,check3]=LinePlaneIntersection(Normal3(faceIndex1,:),vertOnPlane3,newall(1:3,4,i)',newall(1:3,4,i+1)');
        [intersects4,check4]=LinePlaneIntersection(Normal4(faceIndex1,:),vertOnPlane4,newall(1:3,4,i)',newall(1:3,4,i+1)');
        [intersects5,check5]=LinePlaneIntersection(Normal5(faceIndex1,:),vertOnPlane5,newall(1:3,4,i)',newall(1:3,4,i+1)');
        [intersects6,check6]=LinePlaneIntersection(Normal6(faceIndex1,:),vertOnPlane6,newall(1:3,4,i)',newall(1:3,4,i+1)');
        
        if (check1 == 1 && IsIntersectionPointInsideTriangle(intersects1,vertex1(faces1(faceIndex1,:)',:))) || (check2 == 1 && IsIntersectionPointInsideTriangle(intersects2,vertex2(faces2(faceIndex1,:)',:))) || (check3 == 1 && IsIntersectionPointInsideTriangle(intersects3,vertex3(faces3(faceIndex1,:)',:)))...
                || (check4 == 1 && IsIntersectionPointInsideTriangle(intersects4,vertex4(faces4(faceIndex1,:)',:)))|| (check5 == 1 && IsIntersectionPointInsideTriangle(intersects5,vertex5(faces5(faceIndex1,:)',:)))|| (check6 == 1 && IsIntersectionPointInsideTriangle(intersects6,vertex6(faces6(faceIndex1,:)',:)))
            disp('Cannot get to final destination, please clear the area')
            %uiwait
        end
    end
end
qWaypoints = [qWaypoints(end,:); q1];


%  RMRC To final Position

q0 = robot.model.getpos;
x1 = robot.model.fkine(q0);
x2 = robot.model.fkine(q1);
for i = 1:steps
    x0(:,i) = x1(1:3,4)*(1-s(i)) + s(i)*x2(1:3,4);                  
    theta(1,i) = 0;               
    theta(2,i) = 5*pi/9;            
    theta(3,i) = 0;
end

for i = 1:steps-1
    T = robot.model.fkine(real(qWaypoints(i,:))); 
    deltaX = x0(:,i+1) - T(1:3,4);  
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                    
    Ra = T(1:3,1:3);                                                        
    Rdot = (1/deltaT)*(Rd - Ra);                                               
    S = Rdot*Ra';                                                           
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                             
    xdot = W*[linear_velocity;angular_velocity];                          
    J = robot.model.jacob0(real(qWaypoints(i,:)));                 
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon  
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   
    qdot(i,:) = (invJ*xdot)';                                                
    for j = 1:6                                                             
        if qWaypoints(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                  
            qdot(i,j) = 0; % Stop the motor
        elseif qWaypoints(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qWaypoints(i+1,:) = real(qWaypoints(i,:)) + deltaT*real(qdot(i,:));                         
    positionError(:,i) = x0(:,i+1) - T(1:3,4);                                            
 end
% robot.model.animate(qWaypoints);
robot.model.animate(qWaypoints);

end
   
%% Trajectory with RRT
% q0 = robot.model.getpos;
% q1 = rad(1,:);
% qWaypoints = [q0;q1];
% checkedTillWaypoint = 1;
% isCollision = true;
% qMatrix = [];
% while (isCollision)
%     startWaypoint = checkedTillWaypoint;
%     for i = startWaypoint:size(qWaypoints,1)-1
%          qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10),robot.model);
%         if ~IsCollision(robot.model,qMatrixJoin,faces1,vertex1,Normal1)
%             qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
%              robot.model.plot(qMatrixJoin);
%             size(qMatrix);
%             isCollision = false;
%             checkedTillWaypoint = i+1;
%             % Now try and join to the final goal (q2)
%              qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q1],deg2rad(10),robot.model);
%             if ~IsCollision(robot.model,qMatrixJoin,faces1,vertex1,Normal1)
%                 qMatrix = [qMatrix;qMatrixJoin];
%                 % Reached goal without collision, so break out
%                 break;
%             end
%         else
%             % Randomly pick a pose that is not in collision
%             qRand = (2 * rand(1,3) - 1) * pi;
%             while IsCollision(robot.model,qRand,faces1,vertex1,Normal1)
%                 qRand = (2 * rand(1,3) - 1) * pi;
%             end
%             qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
%             isCollision = true;
%             break;
%         end
%     end
% 
% end 
%  
%           

%% Trajectory with jtraj
% q1 = robot.model.ikcon(redPos);
% qMatrix = jtraj(q0, q1, steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
% 
% q2 = robot.model.ikcon(redGoalPose);
% qMatrix = jtraj(q1, q2, steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     redStart = robot.model.fkine(qMatrix(i,:));
%     delete(redCanLocation);
%     redCanLocation = redCan(redStart*troty(pi/2));
%     drawnow();
% end
% 
% qMatrix = jtraj(q2,q0,steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
% 
% % Green Can Sequence
% 
% q3 = robot.model.ikcon(greenPos);
% qMatrix = jtraj(q0, q3, steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
% 
% q4 = robot.model.ikcon(greenGoalPose);
% qMatrix = jtraj(q3, q4, steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     greenStart = robot.model.fkine(qMatrix(i,:));
%     delete(greenCanLocation);
%     greenCanLocation = greenCan(greenStart*troty(pi/2));
%     drawnow();
% end
% 
% qMatrix = jtraj(q4,q0,steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
% 
% % Blue Can Sequence
% 
% q5 = robot.model.ikcon(bluePos);
% qMatrix = jtraj(q0,q5,steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
% 
% q6 = robot.model.ikcon(blueGoalPose);
% qMatrix = jtraj(q5, q6, steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     blueStart = robot.model.fkine(qMatrix(i,:));
%     delete(blueCanLocation);
%     blueCanLocation = blueCan(blueStart*troty(pi/2));
%     drawnow();
% end
% 
% qMatrix = jtraj(q6,q0,steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix(i,:));
%     drawnow();
% end
%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end
%% GetLinkPoses
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links)+1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:,i);
    current_transform = current_transform * trotz(q(1,i) + L.offset) * transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end