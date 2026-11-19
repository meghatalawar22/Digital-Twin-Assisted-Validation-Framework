clear; clc; close all;
scenario = input('Scenario number to validate (1-4): ');
load(sprintf('hw_run_%d.mat', scenario), 'D');

% ---- same values as the Arduino sketch ----
p.obstacleDist=20; p.warnDist=30; p.resumeDist=35; p.clearNeeded=6;
p.speedA=60; p.turnSpeed=90; p.kickSpeed=130;

n = height(D);
twMode=zeros(n,1); twPwm=zeros(n,1); twSt=zeros(n,1); twClr=zeros(n,1); kick=false(n,1);
for i = 1:n
    [twMode(i),twPwm(i),twSt(i),twClr(i),kick(i)] = ...
        twinController(D.L(i),D.R(i),D.dist(i),D.stIn(i),D.clrIn(i),p);
end

% ---- compare, one loop pass at a time ----
hwKick = ismember(D.mode,[6 7]);
modeOK = (~kick & D.mode==twMode) | (kick & hwKick);

pwmExp = twPwm;
pwmExp(kick & D.mode==6) = p.kickSpeed;
pwmExp(kick & D.mode==7) = 0;
pwmOK = abs(D.pwmA)==pwmExp & abs(D.pwmB)==pwmExp;

stExp = twSt;  stExp(kick) = double(D.mode(kick)==7);   % kick result comes from fresh readings
stOK  = D.stOut == stExp;
clrOK = D.clrOut == twClr;

zone   = D.dist>p.obstacleDist & D.dist<=p.warnDist;     % beeping zone, timing-dependent
buzOK  = zone | (D.buz == double(D.dist<=p.obstacleDist));

all_ok = modeOK & pwmOK & stOK & clrOK & buzOK;

names = {'FWD','TURN_R','TURN_L','BOTH_BLK','OBST_STOP','LATCHED','KICK_OK','KICK_ABORT'};
twShow = twMode; twShow(kick & hwKick) = D.mode(kick & hwKick);
C = accumarray([D.mode+1, twShow+1], 1, [8 8]);

fprintf('\n=== TWIN vs HARDWARE: scenario %d ===\n', scenario);
fprintf('Records compared          : %d\n', n);
fprintf('Mode match                : %.2f %%\n', 100*mean(modeOK));
fprintf('PWM match                 : %.2f %%\n', 100*mean(pwmOK));
fprintf('Stop-latch match          : %.2f %%\n', 100*mean(stOK));
fprintf('Clear-counter match       : %.2f %%\n', 100*mean(clrOK));
fprintf('Buzzer match (outside warn zone): %.2f %%\n', 100*mean(buzOK));
fprintf('Records with all five OK  : %.2f %%\n', 100*mean(all_ok));
disp(array2table(C,'VariableNames',names,'RowNames',names));   % rows = hardware, cols = twin

bad = find(~all_ok);
if ~isempty(bad)
    disp('First mismatches:');
    disp(D(bad(1:min(10,numel(bad))), {'ms','L','R','dist','stIn','clrIn','mode','pwmA','buz'}));
end

% ---- timing ----
loopMs = diff(D.ms);  loopMs = loopMs(loopMs>0 & loopMs<100);
fprintf('\nHardware loop period      : %.1f ms median  (your twin uses dt = 20 ms)\n', median(loopMs));

stopIdx = find(D.mode==4);
if ~isempty(stopIdx)
    fprintf('Obstacle stops            : %d\n', numel(stopIdx));
    fprintf('Distance when stopping    : %.1f +/- %.1f cm  (rule: <= %d)\n', ...
        mean(D.dist(stopIdx)), std(D.dist(stopIdx)), p.obstacleDist);
end

delays = nan(numel(stopIdx),1);
for k = 1:numel(stopIdx)
    i0 = stopIdx(k);
    kk = find(D.mode==6 & (1:n)'>i0, 1);
    jj = find(D.dist>p.resumeDist & (1:n)'>i0, 1);
    if ~isempty(kk) && ~isempty(jj), delays(k) = (D.ms(kk)-D.ms(jj))/1000; end
end
if any(~isnan(delays))
    fprintf('Restart delay (hardware)  : %.2f s\n', mean(delays,'omitnan'));
    fprintf('Restart delay (twin, dt=20ms): %.2f s\n', p.clearNeeded*0.02);
end

% ---- plots ----
t = (D.ms - D.ms(1))/1000;
figure('Name','Twin vs Hardware','Color','w');
subplot(3,1,1);
stairs(t, D.mode, 'b', 'LineWidth',1.3); hold on;
stairs(t, twShow, 'r--', 'LineWidth',1.1);
plot(t(~all_ok), D.mode(~all_ok), 'kx');
yticks(0:7); yticklabels(names); ylabel('Mode'); grid on;
legend('Hardware','Twin','Mismatch'); title('Controller decisions');

subplot(3,1,2);
plot(t, D.dist, 'k'); hold on;
yline(p.obstacleDist,'r--','Stop'); yline(p.warnDist,'--','Warn');
yline(p.resumeDist,'g--','Resume'); ylim([0 120]); ylabel('Distance (cm)'); grid on;

subplot(3,1,3);
stairs(t, D.buz, 'm'); hold on; stairs(t, D.stOut, 'k');
ylim([-0.2 1.2]); ylabel('State'); xlabel('Time (s)'); grid on;
legend('Buzzer','Stop latch');

%% ============ the twin's decision logic as one function ============
function [mode,pwm,stOut,clrOut,kick] = twinController(L,R,dist,stIn,clrIn,p)
kick = false;  stOut = stIn;  clrOut = clrIn;
if stIn                                   % stopped latch
    mode = 5; pwm = 0;
    if dist > p.resumeDist, clrOut = clrIn+1; else, clrOut = 0; end
    if clrOut >= p.clearNeeded
        kick = true; mode = 6; pwm = p.kickSpeed; clrOut = 0; stOut = 0;
    end
    return
end
if dist <= p.obstacleDist                 % obstacle stop
    mode = 4; pwm = 0; stOut = 1; clrOut = 0; return
end
if L==0 && R==0,      mode = 0; pwm = p.speedA;      % forward
elseif L==1 && R==0,  mode = 1; pwm = p.turnSpeed;   % turn right
elseif L==0 && R==1,  mode = 2; pwm = p.turnSpeed;   % turn left
else,                 mode = 3; pwm = 0;             % both black
end
end
