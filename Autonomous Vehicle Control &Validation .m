%% SINGLE-LANE AUTONOMOUS CAR - MATLAB SIMULATION
% Servo-free simulation based on the latest Arduino Mega 2560 program.
% Features:
% 1. Two IR sensors for lane-following decisions
% 2. Fixed forward-facing ultrasonic sensor
% 3. Slow-to-fast warning buzzer between 30 cm and 20 cm
% 4. Emergency stop at or below 20 cm
% 5. Restart only after 6 consecutive readings above 35 cm
% 6. 130 PWM kick-start for 150 ms, with obstacle re-checking
% 7. Simulation safety clamp prevents a time-step from moving the car
%    through the obstacle before the next virtual ultrasonic reading.
%
% IMPORTANT: This is a software simulation. Sensor readings are virtual;
% it does not communicate with a physical Arduino or motor driver.

clear; clc; close all;

%% Arduino Mega 2560 pin mapping (shown for reference; not used by MATLAB)
leftSensorPin  = "A3";
rightSensorPin = "A2";
trigPin        = 3;
echoPin        = 2;
buzzerPin      = 4;
ENA            = 11;
IN1            = 10;
IN2            = 9;
IN3            = 8;
IN4            = 7;
ENB            = 6;

%% Values copied from the Arduino program
speedA       = 60;       % normal motor PWM
speedB       = 60;
turnSpeed    = 90;       % turning motor PWM
kickSpeed    = 130;      % restart PWM
kickTime     = 0.150;    % seconds

obstacleDist = 20;       % cm: stop at or below this distance
warnDist     = 30;       % cm: warning beeps start
resumeDist   = 35;       % cm: path must be strictly above this distance
clearNeeded  = 6;        % consecutive clear readings

beepSlow     = 0.600;    % seconds between beeps at 30 cm
beepFast     = 0.100;    % seconds between beeps near 20 cm
beepOnTime   = 0.060;    % seconds per beep

%% Simulation and oval track settings
dt = 0.04;
simulationTime = 55;
time = 0:dt:simulationTime;
numSteps = numel(time);

ovalA = 30;
ovalB = 18;
roadWidth = 6.5;
trackLength = pi*(3*(ovalA+ovalB) - ...
    sqrt((3*ovalA+ovalB)*(ovalA+3*ovalB)));

carS = 0;                       % distance travelled along track (m)
carSpeed = 0;                   % virtual speed (m/s)
maxSpeed = 6;                   % visual speed cap (m/s)
normalAcceleration = 2.2;       % m/s^2

% Put one stationary obstacle ahead on the oval.
obstacleS = 0.28*trackLength;
obstacleHalfLength = 0.75;      % approximate obstacle size (m)

% Demo-only event: remove the virtual obstacle after 32 seconds so that
% the clear-path confirmation and restart sequence can be demonstrated.
% Set this to Inf to keep the virtual obstacle in place for the whole run.
obstacleRemovalTime = 32;
obstaclePresent = true;

%% State variables corresponding to the Arduino sketch
stopped = false;
clearCount = 0;
lastBeepTime = -inf;
buzzerState = false;

kickActive = false;
kickElapsed = 0;

%% Create the oval track display
fig = figure('Name','Servo-Free Single-Lane Autonomous Car', ...
    'Color',[0.94 0.96 0.98]);
ax = axes(fig);
hold(ax,'on');
axis(ax,'equal');
grid(ax,'on');
view(ax,0,90);
xlabel(ax,'X position (m)');
ylabel(ax,'Y position (m)');
title(ax,'IR Lane Following with Ultrasonic Warning and Emergency Stop');
set(ax,'Color',[0.68 0.77 0.66]);
axis(ax,[-44 44 -32 32]);

theta = linspace(0,2*pi,600);
fill(ax,(ovalA+roadWidth/2)*cos(theta), ...
    (ovalB+roadWidth/2)*sin(theta),[0.22 0.24 0.27], ...
    'EdgeColor','none');
fill(ax,(ovalA-roadWidth/2)*cos(theta), ...
    (ovalB-roadWidth/2)*sin(theta),[0.68 0.77 0.66], ...
    'EdgeColor','none');

plot(ax,(ovalA+roadWidth/2)*cos(theta), ...
    (ovalB+roadWidth/2)*sin(theta),'w','LineWidth',2);
plot(ax,(ovalA-roadWidth/2)*cos(theta), ...
    (ovalB-roadWidth/2)*sin(theta),'w','LineWidth',2);

% Dashed yellow lane centreline.
for n = 1:2:100
    q = linspace((n-1)*2*pi/100,n*2*pi/100,12);
    plot(ax,ovalA*cos(q),ovalB*sin(q),'--', ...
        'Color',[1 0.88 0.2],'LineWidth',1.5);
end
text(ax,-4,-ovalB-8,'START / FINISH','FontWeight','bold');

%% Draw the obstacle and vehicle
[obsX,obsY,obsHeading] = ovalPose(obstacleS,ovalA,ovalB,trackLength);
obstacleGraphics = drawObstacle(ax,[obsX obsY],obsHeading);

[carX,carY,carHeading] = ovalPose(carS,ovalA,ovalB,trackLength);
carGraphics = createCar(ax,[carX carY],carHeading);

% Fixed forward ultrasonic sensor field-of-view (no servo scanning).
sensorPatch = patch(ax,'XData',[carX carX+8 carX+8 carX], ...
    'YData',[carY-0.7 carY-2 carY+2 carY+0.7], ...
    'FaceColor',[0.1 0.9 0.3],'FaceAlpha',0.13, ...
    'EdgeColor',[0.1 0.8 0.3],'LineStyle','--');

statusText = text(ax,-41,27,'','FontName','Consolas', ...
    'FontSize',9.5,'FontWeight','bold', ...
    'BackgroundColor','w','Margin',5, ...
    'VerticalAlignment','top');

%% Data logging
speedLog = zeros(1,numSteps);
distanceLog = zeros(1,numSteps);
buzzerLog = false(1,numSteps);
stoppedLog = false(1,numSteps);
clearCountLog = zeros(1,numSteps);
leftIRLog = zeros(1,numSteps);
rightIRLog = zeros(1,numSteps);
modeLog = strings(1,numSteps);

%% Main simulation loop
for k = 1:numSteps
    if ~isgraphics(fig)
        break;
    end

    currentTime = time(k);

    % Remove the obstacle at the demonstration time.
    if obstaclePresent && currentTime >= obstacleRemovalTime
        obstaclePresent = false;
        if all(isgraphics(obstacleGraphics))
            delete(obstacleGraphics);
        end
    end

    % Virtual ultrasonic measurement in centimetres.
    if obstaclePresent
        % Distance from the FRONT of the car to the FRONT of the obstacle.
        % Car half-length is about 2.05 m; obstacle half-length is 0.75 m.
        gap = mod(obstacleS-carS,trackLength) ...
            - obstacleHalfLength - 2.05;
        if gap > trackLength/2
            gap = inf;          % obstacle is behind the car
        end

        if isfinite(gap)
            frontDistance = max(0,round(gap*100));
        else
            frontDistance = 200; % ultrasonic no-echo/out-of-range value
        end
    else
        frontDistance = 200;
    end

    %% BUZZER LOGIC - mirrors updateBuzzer()
    if frontDistance <= obstacleDist
        buzzerState = true;     % continuous sound while stopped threshold met

    elseif frontDistance <= warnDist
        beepInterval = beepFast + ...
            (frontDistance-obstacleDist)/(warnDist-obstacleDist) * ...
            (beepSlow-beepFast);

        if currentTime-lastBeepTime >= beepInterval
            lastBeepTime = currentTime;
            buzzerState = true;
        elseif currentTime-lastBeepTime >= beepOnTime
            buzzerState = false;
        end

    else
        buzzerState = false;
    end

    %% STOPPED STATE AND AUTOMATIC RESTART
    if stopped
        carSpeed = 0;
        currentMode = "STOPPED - WAITING FOR CLEAR PATH";

        if frontDistance > resumeDist
            clearCount = clearCount+1;
        else
            clearCount = 0;
        end

        % Start the kick exactly once after the required consecutive
        % clear readings. Do not reset kickElapsed on every frame.
        if clearCount >= clearNeeded && ~kickActive
            kickActive = true;
            kickElapsed = 0;
        end

        if kickActive
            % Re-check the ultrasonic distance during the kick-start.
            if frontDistance <= obstacleDist
                kickActive = false;
                carSpeed = 0;
                clearCount = 0;
                currentMode = "KICK-START CANCELLED";
            else
                kickElapsed = kickElapsed+dt;
                carSpeed = maxSpeed*(kickSpeed/255);
                carS = mod(carS+carSpeed*dt,trackLength);
                currentMode = "KICK-STARTING";

                if kickElapsed >= kickTime
                    kickActive = false;
                    stopped = false;
                    clearCount = 0;
                    % The next simulation step returns to the normal IR
                    % lane-following branch and continues around the oval.
                    currentMode = "RESUMING LANE FOLLOWING";
                end
            end
        end

    else
        %% OBSTACLE DETECTION AND EMERGENCY STOP
        if frontDistance <= obstacleDist
            carSpeed = 0;
            stopped = true;
            clearCount = 0;
            kickActive = false;
            currentMode = "EMERGENCY STOP";

        else
            %% IR SENSOR LANE-FOLLOWING LOGIC
            % Virtual IR readings are generated to demonstrate the same
            % truth table as the Arduino program.
            % LOW/LOW = forward; HIGH/LOW = right; LOW/HIGH = left.
            % HIGH/HIGH = stop.
            lateralError = 0.22*sin(0.45*currentTime);

            if lateralError > 0.13
                leftIR = 1;
                rightIR = 0;
            elseif lateralError < -0.13
                leftIR = 0;
                rightIR = 1;
            else
                leftIR = 0;
                rightIR = 0;
            end

            if leftIR == 0 && rightIR == 0
                currentMode = "LANE FOLLOW - FORWARD";
                carSpeed = min(maxSpeed, ...
                    carSpeed+normalAcceleration*dt);

            elseif leftIR == 1 && rightIR == 0
                currentMode = "LANE FOLLOW - TURN RIGHT";
                carSpeed = min(maxSpeed, ...
                    carSpeed+normalAcceleration*dt);

            elseif leftIR == 0 && rightIR == 1
                currentMode = "LANE FOLLOW - TURN LEFT";
                carSpeed = min(maxSpeed, ...
                    carSpeed+normalAcceleration*dt);

            else
                currentMode = "BOTH IR ON BLACK - STOP";
                carSpeed = 0;
            end

            % Vehicle follows the oval lane centreline; no lane change.
            % Safety clamp: prevent one simulation time-step from carrying
            % the car through the obstacle before the next sensor reading.
            proposedAdvance = carSpeed*dt;
            if obstaclePresent && isfinite(gap) && gap > 0
                clearanceBeforeStop = max(0,gap-obstacleDist/100);
                if proposedAdvance >= clearanceBeforeStop
                    carS = mod(carS+clearanceBeforeStop,trackLength);
                    carSpeed = 0;
                    stopped = true;
                    clearCount = 0;
                    currentMode = "EMERGENCY STOP - SAFETY CLAMP";
                else
                    carS = mod(carS+proposedAdvance,trackLength);
                end
            else
                carS = mod(carS+proposedAdvance,trackLength);
            end
        end
    end

    %% Update car and forward sensor display
    [carX,carY,carHeading] = ovalPose(carS,ovalA,ovalB,trackLength);
    updateCarGraphics(carGraphics,[carX carY],carHeading);

    rotation = [cos(carHeading) -sin(carHeading); ...
                sin(carHeading)  cos(carHeading)];
    sensorCoordinates = rotation * ...
        [0 8 8 0; -0.7 -2 2 0.7] + [carX;carY];

    set(sensorPatch,'XData',sensorCoordinates(1,:), ...
        'YData',sensorCoordinates(2,:));

    if frontDistance <= warnDist
        set(sensorPatch,'FaceColor',[1 0.1 0.05], ...
            'EdgeColor',[1 0.05 0.05]);
    else
        set(sensorPatch,'FaceColor',[0.1 0.9 0.3], ...
            'EdgeColor',[0.1 0.8 0.3]);
    end

    %% Update on-screen status
    if buzzerState, buzzerLabel = "ON"; else, buzzerLabel = "OFF"; end
    if stopped, stopLabel = "YES"; else, stopLabel = "NO"; end

    if frontDistance >= 200
        distanceLabel = "OUT OF RANGE";
    else
        distanceLabel = sprintf('%d cm',frontDistance);
    end

    if exist('leftIR','var')
        leftValue = leftIR;
        rightValue = rightIR;
    else
        leftValue = 0;
        rightValue = 0;
    end

    set(statusText,'String',sprintf([ ...
        'TIME: %.1f s\nSPEED: %.1f km/h\n' ...
        'LEFT IR (%s): %d\nRIGHT IR (%s): %d\n' ...
        'ULTRASONIC DISTANCE: %s\n' ...
        'OBSTACLE STOP: %s\nCLEAR READINGS: %d / %d\n' ...
        'BUZZER: %s\nMODE: %s'], ...
        currentTime,carSpeed*3.6,leftSensorPin,leftValue, ...
        rightSensorPin,rightValue,distanceLabel,stopLabel, ...
        clearCount,clearNeeded,buzzerLabel,char(currentMode)));

    if stopped
        set(statusText,'Color',[0.8 0.04 0.04]);
    elseif frontDistance <= warnDist
        set(statusText,'Color',[0.85 0.38 0.02]);
    else
        set(statusText,'Color',[0.08 0.1 0.14]);
    end

    %% Save data
    speedLog(k) = carSpeed*3.6;
    distanceLog(k) = frontDistance;
    buzzerLog(k) = buzzerState;
    stoppedLog(k) = stopped;
    clearCountLog(k) = clearCount;
    leftIRLog(k) = leftValue;
    rightIRLog(k) = rightValue;
    modeLog(k) = currentMode;

    drawnow limitrate;
    pause(dt);
end

%% Plot simulation results
lastStep = k;
figure('Name','Autonomous Car Results','Color','w');

subplot(4,1,1);
plot(time(1:lastStep),speedLog(1:lastStep),'LineWidth',1.5);
grid on;
ylabel('Speed (km/h)');
title('Single-Lane Following and Obstacle Response');

subplot(4,1,2);
plot(time(1:lastStep),distanceLog(1:lastStep),'LineWidth',1.3);
hold on;
yline(obstacleDist,'r--','Stop threshold: 20 cm');
yline(warnDist,'Color',[0.9 0.55 0.05], ...
    'LineStyle','--','Label','Warning: 30 cm');
yline(resumeDist,'g:','Resume: 35 cm');
grid on;
ylabel('Distance (cm)');

subplot(4,1,3);
stairs(time(1:lastStep),double(buzzerLog(1:lastStep)), ...
    'LineWidth',1.3);
hold on;
stairs(time(1:lastStep),double(stoppedLog(1:lastStep)), ...
    'LineWidth',1.3);
grid on;
ylim([-0.1 1.1]);
yticks([0 1]);
yticklabels({'OFF','ON'});
ylabel('State');
legend('Buzzer','Obstacle stop','Location','best');

subplot(4,1,4);
stairs(time(1:lastStep),clearCountLog(1:lastStep),'LineWidth',1.3);
grid on;
ylim([0 clearNeeded+1]);
yticks(0:clearNeeded);
ylabel('Clear count');
xlabel('Time (s)');

fprintf('\nSimulation completed.\n');
fprintf('Servo has been removed; ultrasonic sensor is fixed forward.\n');
fprintf('IR and ultrasonic readings are virtual MATLAB values.\n');

%% ================= LOCAL FUNCTIONS =================

function [x,y,heading] = ovalPose(s,a,b,totalLength)
    angle = mod(s/totalLength*2*pi,2*pi);
    x = a*cos(angle);
    y = b*sin(angle);
    heading = atan2(b*cos(angle),-a*sin(angle));
end

function car = createCar(ax,position,heading)
    car.parts = gobjects(0);
    car.pose = [position heading];

    specifications = { ...
        [0 0 0.55],[4.1 1.9 0.72],[0.05 0.32 0.88];
        [-0.15 0 1.02],[2.15 1.52 0.68],[0.12 0.17 0.22];
        [0.62 0 1.07],[0.82 1.42 0.50],[0.28 0.68 0.86];
        [-0.88 0 1.07],[0.72 1.42 0.50],[0.18 0.42 0.62];
        [1.95 0 0.58],[0.18 1.72 0.22],[0.72 0.75 0.8];
        [-2.02 0 0.58],[0.16 1.72 0.22],[0.22 0.22 0.24];
        [-1.2 -1.0 0.3],[0.85 0.28 0.58],[0.025 0.025 0.03];
        [-1.2 1.0 0.3],[0.85 0.28 0.58],[0.025 0.025 0.03];
        [1.2 -1.0 0.3],[0.85 0.28 0.58],[0.025 0.025 0.03];
        [1.2 1.0 0.3],[0.85 0.28 0.58],[0.025 0.025 0.03]};

    for n = 1:size(specifications,1)
        part = drawBox(ax,specifications{n,1}, ...
            specifications{n,2},specifications{n,3}, ...
            position,heading);
        car.parts(end+1) = part;
    end
end

function handle = drawBox(ax,centre,dimensions,faceColour,position,heading)
    vertices = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
                -1 -1 1;1 -1 1;1 1 1;-1 1 1];
    vertices = vertices.*dimensions/2 + centre;

    rotation = [cos(heading) -sin(heading); ...
                sin(heading)  cos(heading)];
    vertices(:,1:2) = (rotation*vertices(:,1:2)')' + position;

    faces = [1 2 3 4;5 8 7 6;1 5 6 2; ...
             2 6 7 3;3 7 8 4;4 8 5 1];

    handle = patch(ax,'Vertices',vertices,'Faces',faces, ...
        'FaceColor',faceColour,'EdgeColor',[0.08 0.08 0.09], ...
        'LineWidth',0.4);
end

function updateCarGraphics(car,position,newHeading)
    if isappdata(car.parts(1),'latestPose')
        previousPose = getappdata(car.parts(1),'latestPose');
    else
        previousPose = car.pose;
        setappdata(car.parts(1),'latestPose',previousPose);
    end
    oldPosition = previousPose(1:2);
    oldHeading = previousPose(3);

    oldRotation = [cos(oldHeading) -sin(oldHeading); ...
                   sin(oldHeading)  cos(oldHeading)];
    newRotation = [cos(newHeading) -sin(newHeading); ...
                   sin(newHeading)  cos(newHeading)];

    for n = 1:numel(car.parts)
        vertices = get(car.parts(n),'Vertices');
        localXY = (oldRotation' * ...
            (vertices(:,1:2)-oldPosition)')';
        vertices(:,1:2) = (newRotation*localXY')' + position;
        set(car.parts(n),'Vertices',vertices);
    end

    % Update pose in the returned structure for the next animation frame.
    car.pose = [position newHeading];
    setappdata(car.parts(1),'latestPose',car.pose);
end

function graphics = drawObstacle(ax,position,heading)
    graphics = gobjects(2,1);
    graphics(1) = drawBox(ax,[0 0 0.7],[1.7 1.7 1.4], ...
        [0.85 0.06 0.04],position,heading);
    graphics(2) = drawBox(ax,[0 0 1.55],[0.35 0.35 0.35], ...
        [1 0.85 0.05],position,heading);
end
