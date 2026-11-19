%% ARDUINO-MATCHED DIGITAL TWIN - SINGLE LANE OVAL TRACK
% Mirrors the decision conditions in the supplied Arduino Mega sketch.
% MATLAB base graphics only. Inputs are simulated; this is not live Arduino data.
clear; clc; close all;

fprintf('\nARDUINO-MATCHED OVAL TRACK DIGITAL TWIN\n');
fprintf('1 No obstacle / lane following\n2 Obstacle removed after 14 seconds\n');
fprintf('3 Obstacle remains in front\n4 IR sensor truth-table demonstration\n');
scenario=input('Choose scenario (1-4): ');
if isempty(scenario)||~ismember(scenario,1:4), scenario=2; end

% Values copied from the Arduino sketch
speedA=60; speedB=60; turnSpeed=90; kickSpeed=130; kickTime=.150;
obstacleDist=20; warnDist=30; resumeDist=35; clearNeeded=6;
beepSlow=.600; beepFast=.100; beepOnTime=.060; servoAngle=90;

% Oval track and virtual vehicle
a=28; b=17; roadWidth=5; C=pi*(3*(a+b)-sqrt((3*a+b)*(a+3*b)));
dt=.02; t=0:dt:35; N=numel(t);
s=0; lateral=0; heading=pi/2; v=0; maxYaw=1.25;
speedLog=zeros(1,N); distLog=zeros(1,N); leftLog=zeros(1,N);
rightLog=zeros(1,N); buzzerLog=false(1,N); stopLog=false(1,N);
stopped=false; clearCount=0; lastBeep=-1e6; beepEnd=-1e6; buzzer=false;
if scenario==1 || scenario==4, obstacleS=inf; else, obstacleS=24; end

% Main scene
fig=figure('Name','Arduino Matched Digital Twin','Color',[.05 .07 .10], ...
    'Position',[60 60 1450 820]);
ax=axes('Parent',fig,'Position',[.035 .12 .61 .80]);
hold(ax,'on'); axis(ax,'equal'); view(ax,0,90); grid(ax,'on');
set(ax,'Color',[.12 .17 .16]); axis(ax,[-40 40 -27 27]);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
title(ax,'SINGLE-LANE OVAL TEST TRACK','Color',[.9 .95 1],'FontSize',14);
th=linspace(0,2*pi,500);
fill(ax,(a+roadWidth/2)*cos(th),(b+roadWidth/2)*sin(th),[.22 .25 .29],'EdgeColor','none');
fill(ax,(a-roadWidth/2)*cos(th),(b-roadWidth/2)*sin(th),[.12 .17 .16],'EdgeColor','none');
plot(ax,(a+roadWidth/2)*cos(th),(b+roadWidth/2)*sin(th),'w','LineWidth',2);
plot(ax,(a-roadWidth/2)*cos(th),(b-roadWidth/2)*sin(th),'w','LineWidth',2);
for k=1:100
    if mod(k,2)==1
        q=linspace((k-1)*2*pi/100,k*2*pi/100,10);
        plot(ax,a*cos(q),b*sin(q),'--','Color',[1 .83 .16],'LineWidth',1.5);
    end
end
text(ax,-3,-b-6,'START / FINISH','Color','w','FontWeight','bold');
[x,y,h]=ovalPose(s,0,a,b,C);
car=makeCar(ax,[x y],h);
obs=gobjects(0);
if isfinite(obstacleS)
    [ox,oy,oh]=ovalPose(obstacleS,0,a,b,C);
    obs=makeBlock(ax,[ox oy],oh);
end
cone=patch(ax,'XData',[x x+7 x+7 x],'YData',[y-.6 y-1.8 y+1.8 y+.6], ...
    'FaceColor',[.1 .85 .4],'FaceAlpha',.12,'EdgeColor',[.1 .85 .4],'LineStyle','--');

% Dashboard
p=uipanel(fig,'Title',' LIVE DIGITAL TWIN - CONTROL MONITOR ','FontSize',12, ...
    'ForegroundColor',[.3 .85 1],'BackgroundColor',[.07 .10 .15], ...
    'Position',[.67 .05 .31 .90]);
mkLabel(p,'SYSTEM STATUS',.92);
status=mkValue(p,'SIMULATION ONLY - ARDUINO NOT CONNECTED',.875,[1 .78 .35]);
mkLabel(p,'LIVE VIRTUAL TELEMETRY',.81);
vSpeed=mkRead(p,'Vehicle speed','0 km/h',.755);
vDist=mkRead(p,'Ultrasonic distance','200 cm',.69);
vMode=mkRead(p,'Motor/control state','FORWARD',.625);
vPWM=mkRead(p,'Motor PWM','60 / 60',.56);
vIR=mkRead(p,'IR sensors','L LOW / R LOW',.495);
vBuzz=mkRead(p,'Buzzer','OFF',.43);
vServo=mkRead(p,'Servo position','90 deg FRONT',.365);
vClear=mkRead(p,'Clear readings','0 / 6',.30);
mkLabel(p,'ARDUINO THRESHOLDS',.235);
uicontrol(p,'Style','text','String',sprintf( ...
    'STOP <=20 cm | WARN <=30 cm\\nRESTART >35 cm for 6 readings\\nKICK PWM 130 for 150 ms\\nNORMAL PWM 60 | TURN PWM 90'), ...
    'Units','normalized','Position',[.06 .11 .88 .11], ...
    'BackgroundColor',[.11 .15 .22],'ForegroundColor',[.85 .9 .98], ...
    'FontName','Consolas','FontSize',9,'HorizontalAlignment','left');
uicontrol(p,'Style','text','String','Virtual sensor values are simulated, not read from the Arduino.', ...
    'Units','normalized','Position',[.06 .035 .88 .055], ...
    'BackgroundColor',[.07 .10 .15],'ForegroundColor',[.6 .85 .95], ...
    'FontSize',8,'HorizontalAlignment','left');

% Simulation loop
for i=1:N
    if ~ishandle(fig), break; end
    now=t(i);
    % In scenario 2, emulate physically removing the obstacle after 14 s.
    if scenario==2 && now>=14
        obstacleS=inf;
        if ~isempty(obs) && isgraphics(obs), delete(obs); end
        obs=gobjects(0);
    end

    % Virtual ultrasonic sensor
    if isfinite(obstacleS)
        gap=mod(obstacleS-s,C);
        if gap<.45*C, frontDist=max(0,round((gap-2.2)*100));
        else, frontDist=200; end
    else
        frontDist=200;
    end

    % Virtual IR sensor readings; scenario 4 cycles through all four combinations.
    if scenario==4
        phase=mod(floor(now/4),4);
        if phase==0, leftIR=0; rightIR=0;
        elseif phase==1, leftIR=1; rightIR=0;
        elseif phase==2, leftIR=0; rightIR=1;
        else, leftIR=1; rightIR=1; end
    else
        if lateral>.22, leftIR=1; rightIR=0;
        elseif lateral<-.22, leftIR=0; rightIR=1;
        else, leftIR=0; rightIR=0; end
    end

    % updateBuzzer() logic
    if frontDist<=obstacleDist
        buzzer=true;
    elseif frontDist<=warnDist
        interval=mapValue(frontDist,obstacleDist,warnDist,beepFast,beepSlow);
        if now-lastBeep>=interval
            lastBeep=now; buzzer=true; beepEnd=now+beepOnTime;
        elseif now>=beepEnd
            buzzer=false;
        end
    else
        buzzer=false;
    end

    % loop() stop latch and restart logic
    pwmL=speedA; pwmR=speedB; motorState="FORWARD";
    if stopped
        pwmL=0; pwmR=0; motorState="STOPPED";
        if frontDist>resumeDist, clearCount=clearCount+1;
        else, clearCount=0; end

        if clearCount>=clearNeeded
            % kickStart(): PWM 130 for 150 ms with distance checks
            kickOK=true; kickElapsed=0;
            while kickElapsed<kickTime
                if isfinite(obstacleS)
                    kickGap=mod(obstacleS-s,C);
                    kickDist=max(0,round((kickGap-2.2)*100));
                else
                    kickDist=200;
                end
                if kickDist<=obstacleDist
                    kickOK=false; break;
                end
                kickElapsed=kickElapsed+dt;
            end
            if kickOK
                stopped=false; clearCount=0; motorState="KICK START COMPLETE";
                pwmL=kickSpeed; pwmR=kickSpeed;
            else
                clearCount=0; motorState="KICK ABORTED";
            end
        end
    elseif frontDist<=obstacleDist
        pwmL=0; pwmR=0; stopped=true; clearCount=0;
        motorState="OBSTACLE STOP";
    else
        % Exact laneFollow() if/else truth table
        if leftIR==0 && rightIR==0
            motorState="FORWARD"; pwmL=speedA; pwmR=speedB;
        elseif leftIR==1 && rightIR==0
            motorState="TURN RIGHT"; pwmL=turnSpeed; pwmR=turnSpeed;
        elseif leftIR==0 && rightIR==1
            motorState="TURN LEFT"; pwmL=turnSpeed; pwmR=turnSpeed;
        else
            motorState="BOTH BLACK - STOP"; pwmL=0; pwmR=0;
        end
    end

    % Approximate differential-drive motion for visualization only.
    if motorState=="FORWARD" || motorState=="KICK START COMPLETE"
        v=.045*(pwmL+pwmR)/2; yaw=0;
    elseif motorState=="TURN RIGHT"
        v=.035*(pwmL+pwmR)/2; yaw=-maxYaw;
    elseif motorState=="TURN LEFT"
        v=.035*(pwmL+pwmR)/2; yaw=maxYaw;
    else
        v=0; yaw=0;
    end
    heading=heading+yaw*dt;
    s=mod(s+v*dt,C);
    if motorState=="TURN LEFT", lateral=min(.6,lateral+.1*dt);
    elseif motorState=="TURN RIGHT", lateral=max(-.6,lateral-.1*dt);
    elseif motorState=="FORWARD", lateral=lateral*max(0,1-.18*dt); end

    [x,y,h]=ovalPose(s,lateral,a,b,C);
    updateCar(car,[x y],h);
    R=[cos(h) -sin(h);sin(h) cos(h)];
    fan=R*[0 7 7 0;-.6 -1.8 1.8 .6]+[x;y];
    set(cone,'XData',fan(1,:),'YData',fan(2,:));

    speedLog(i)=v*3.6; distLog(i)=frontDist; leftLog(i)=leftIR;
    rightLog(i)=rightIR; buzzerLog(i)=buzzer; stopLog(i)=stopped;

    if leftIR==0, ltxt='LOW'; else, ltxt='HIGH'; end
    if rightIR==0, rtxt='LOW'; else, rtxt='HIGH'; end
    if buzzer, btxt='ON'; else, btxt='OFF'; end
    set(vSpeed,'String',sprintf('%.1f km/h',v*3.6));
    set(vDist,'String',sprintf('%d cm',frontDist));
    set(vMode,'String',char(motorState));
    set(vPWM,'String',sprintf('%d / %d',pwmL,pwmR));
    set(vIR,'String',sprintf('L %s / R %s',ltxt,rtxt));
    set(vBuzz,'String',btxt);
    set(vServo,'String',sprintf('%d deg FRONT',servoAngle));
    set(vClear,'String',sprintf('%d / %d',clearCount,clearNeeded));
    if stopped
        set(status,'String','OBSTACLE STOP LATCH ACTIVE','ForegroundColor',[1 .45 .3]);
    else
        set(status,'String','VIRTUAL SIMULATION RUNNING','ForegroundColor',[.65 1 .72]);
    end
    drawnow limitrate; pause(dt);
end

% Validation plots
figure('Name','Arduino Logic Validation','Color','w');
subplot(4,1,1); plot(t(1:numel(speedLog)),speedLog,'LineWidth',1.4);
grid on; ylabel('Speed km/h'); title('Digital Twin - Arduino Control Logic');
subplot(4,1,2); plot(t(1:numel(distLog)),distLog,'LineWidth',1.2); hold on;
yline(obstacleDist,'r--','Stop 20 cm'); yline(warnDist,'--','Color',[.9 .5 .05],'Label','Warn 30 cm');
yline(resumeDist,'g--','Resume >35 cm'); grid on; ylabel('Distance cm');
subplot(4,1,3); stairs(t(1:numel(leftLog)),leftLog); hold on;
stairs(t(1:numel(rightLog)),rightLog); grid on; ylabel('IR');
yticks([0 1]); yticklabels({'LOW','HIGH'}); legend('Left IR','Right IR');
subplot(4,1,4); stairs(t(1:numel(buzzerLog)),double(buzzerLog)); hold on;
stairs(t(1:numel(stopLog)),double(stopLog)); grid on; ylabel('State'); xlabel('Time (s)');
yticks([0 1]); legend('Buzzer','Stopped latch');

fprintf('\nFinished. This models your Arduino logic but does not communicate with the Arduino.\n');

%% Helper functions
function mkLabel(parent,str,y)
uicontrol(parent,'Style','text','String',str,'Units','normalized', ...
    'Position',[.06 y .88 .035],'BackgroundColor',[.07 .10 .15], ...
    'ForegroundColor',[.3 .85 1],'FontSize',9,'FontWeight','bold','HorizontalAlignment','left');
end
function h=mkValue(parent,str,y,col)
h=uicontrol(parent,'Style','text','String',str,'Units','normalized', ...
    'Position',[.06 y .88 .04],'BackgroundColor',[.11 .15 .22], ...
    'ForegroundColor',col,'FontName','Consolas','FontSize',9,'FontWeight','bold');
end
function h=mkRead(parent,labelStr,valueStr,y)
uicontrol(parent,'Style','text','String',labelStr,'Units','normalized', ...
    'Position',[.06 y .46 .04],'BackgroundColor',[.07 .10 .15], ...
    'ForegroundColor',[.75 .82 .9],'FontSize',8,'HorizontalAlignment','left');
h=uicontrol(parent,'Style','text','String',valueStr,'Units','normalized', ...
    'Position',[.53 y .41 .04],'BackgroundColor',[.11 .15 .22], ...
    'ForegroundColor',[.65 1 .72],'FontName','Consolas','FontSize',9,'FontWeight','bold');
end
function [x,y,h]=ovalPose(s,off,a,b,C)
q=mod(s/C*2*pi,2*pi); x0=a*cos(q); y0=b*sin(q);
h=atan2(b*cos(q),-a*sin(q)); x=x0-off*sin(h); y=y0+off*cos(h);
end
function car=makeCar(ax,pos,ang)
car.pos=pos; car.ang=ang; car.parts=gobjects(0);
spec={ [0 0 .52],[3.8 1.75 .68],[.05 .35 .92];
[-.1 0 .98],[2 1.42 .62],[.1 .16 .22];
[.55 0 1.02],[.8 1.34 .45],[.25 .65 .85];
[-.8 0 1.02],[.65 1.34 .45],[.18 .4 .6];
[-1.15 -.92 .28],[.78 .25 .52],[.025 .025 .03];
[-1.15 .92 .28],[.78 .25 .52],[.025 .025 .03];
[1.15 -.92 .28],[.78 .25 .52],[.025 .025 .03];
[1.15 .92 .28],[.78 .25 .52],[.025 .025 .03];
[1.82 -.55 .62],[.12 .22 .14],[1 .9 .35];
[1.82 .55 .62],[.12 .22 .14],[1 .9 .35]};
for k=1:size(spec,1), car.parts(end+1)=addBox(ax,spec{k,1},spec{k,2},spec{k,3},pos,ang); end
end
function h=addBox(ax,c,d,col,pos,ang)
V=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1].*d/2+c;
R=[cos(ang) -sin(ang);sin(ang) cos(ang)]; V(:,1:2)=(R*V(:,1:2)')'+pos;
F=[1 2 3 4;5 8 7 6;1 5 6 2;2 6 7 3;3 7 8 4;4 8 5 1];
h=patch(ax,'Vertices',V,'Faces',F,'FaceColor',col,'EdgeColor',[.07 .08 .1],'LineWidth',.4);
end
function updateCar(car,pos,ang)
old=getappdata(car.parts(1),'pose'); if isempty(old), old=[car.pos car.ang]; end
oldPos=old(1:2); oldAng=old(3);
Ro=[cos(oldAng) -sin(oldAng);sin(oldAng) cos(oldAng)];
Rn=[cos(ang) -sin(ang);sin(ang) cos(ang)];
for k=1:numel(car.parts)
V=get(car.parts(k),'Vertices'); local=(Ro'*(V(:,1:2)-oldPos)')';
V(:,1:2)=(Rn*local')'+pos; set(car.parts(k),'Vertices',V);
end
setappdata(car.parts(1),'pose',[pos ang]);
end
function h=makeBlock(ax,pos,ang)
h=addBox(ax,[0 0 .7],[1.4 1.4 1.4],[.85 .08 .05],pos,ang);
end
function y=mapValue(x,a,b,c,d)
y=(x-a)*(d-c)/(b-a)+c;
end
