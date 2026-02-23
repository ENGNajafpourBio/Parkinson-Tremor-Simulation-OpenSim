%% Parkinson's Tremor Simulation - Specialized Forearm and Wrist Analysisclear all; close all; clc;

%% File path
modelPath = 'C:\Users\rayan\Documents\action tremor 3\MOBL_ARMS_41.osim';
motionPath = 'C:\Users\rayan\Documents\action tremor 3\action_tremor_motion.mot';

%% Import the OpenSim library
import org.opensim.modeling.*


model = Model(modelPath);
state = model.initSystem();

disp(' Model loaded. ');
disp(['Number of muscles: ', num2str(model.getMuscles().getSize())]);
disp(['Number of joints: ', num2str(model.getCoordinateSet().getSize())]);

%% Identifying joints
coordSet = model.getCoordinateSet();
numCoords = coordSet.getSize();

coordNames = {};
for i = 0:numCoords-1
    coordName = char(coordSet.get(i).getName());
    coordNames{end+1} = coordName;
end

%% Simulation parameters
duration = 60;
dt = 0.01;
time = 0:dt:duration;
nFrames = length(time);
Fs = 1/dt;

restPhaseEnd = 10;
reachPhaseEnd = 20;

restTremorFreq = 4.5;
restTremorAmp = struct();
restTremorAmp.shoulder = 2.0;
restTremorAmp.elbow = 1.5;
restTremorAmp.wrist = 1.0;
restTremorAmp.forearm = 1.0;

actionTremorFreq = 5.5;
actionTremorAmp = struct();
actionTremorAmp.shoulder = 4.0;
actionTremorAmp.elbow = 3.0;
actionTremorAmp.wrist = 2.5;
actionTremorAmp.forearm = 2.0;

SNR_dB = 40;
SNR_linear = 10^(SNR_dB/20);

%% Definition of positions
restPose = struct();
restPose.shoulder_elev = deg2rad(5);
restPose.shoulder_add = deg2rad(-10);
restPose.shoulder_rot = deg2rad(20);
restPose.elbow_flex = deg2rad(20);
restPose.pro_sup = deg2rad(0);
restPose.wrist_flex = deg2rad(5);
restPose.wrist_dev = deg2rad(0);

targetPose = struct();
targetPose.shoulder_elev = deg2rad(90);
targetPose.shoulder_add = deg2rad(40);
targetPose.shoulder_rot = deg2rad(-15);
targetPose.elbow_flex = deg2rad(100);
targetPose.pro_sup = deg2rad(70);
targetPose.wrist_flex = deg2rad(-10);
targetPose.wrist_dev = deg2rad(0);

%% Create a motion data matrix
motionData = zeros(nFrames, numCoords + 1);
motionData(:, 1) = time';

rng(42);
generateNoise = @(amp_deg, n) randn(n, 1) * (deg2rad(amp_deg) / SNR_linear);

noise_rest = struct();
noise_rest.shoulder = generateNoise(restTremorAmp.shoulder, nFrames);
noise_rest.elbow = generateNoise(restTremorAmp.elbow, nFrames);
noise_rest.wrist = generateNoise(restTremorAmp.wrist, nFrames);
noise_rest.forearm = generateNoise(restTremorAmp.forearm, nFrames);

noise_action = struct();
noise_action.shoulder = generateNoise(actionTremorAmp.shoulder, nFrames);
noise_action.elbow = generateNoise(actionTremorAmp.elbow, nFrames);
noise_action.wrist = generateNoise(actionTremorAmp.wrist, nFrames);
noise_action.forearm = generateNoise(actionTremorAmp.forearm, nFrames);

phase_jitter_rest = cumsum(randn(nFrames, 1) * 0.01);
phase_jitter_action = cumsum(randn(nFrames, 1) * 0.01);

amp_mod_freq = 0.1;
amp_modulation = 1 + 0.15 * sin(2*pi*amp_mod_freq*time');

%% Main loop - motion generation
for i = 1:nFrames
    t = time(i);
    
    if t <= restPhaseEnd
        tremorWeight = min(t/2, 1.0);
        tremorFreq = restTremorFreq;
        currentAmp = restTremorAmp;
        currentPose = restPose;
        phase_jitter = phase_jitter_rest(i);
        currentNoise_shoulder = noise_rest.shoulder(i);
        currentNoise_elbow = noise_rest.elbow(i);
        currentNoise_wrist = noise_rest.wrist(i);
        currentNoise_forearm = noise_rest.forearm(i);
        
    elseif t <= reachPhaseEnd
        reachProgress = (t - restPhaseEnd) / (reachPhaseEnd - restPhaseEnd);
        smoothProgress = reachProgress^2 * (3 - 2*reachProgress);
        
        restWeight = (1 - smoothProgress);
        actionWeight = smoothProgress * 0.2;
        
        tremorFreq = restTremorFreq * restWeight + actionTremorFreq * smoothProgress;
        
        currentAmp = struct();
        currentAmp.shoulder = restTremorAmp.shoulder * restWeight + actionTremorAmp.shoulder * actionWeight;
        currentAmp.elbow = restTremorAmp.elbow * restWeight + actionTremorAmp.elbow * actionWeight;
        currentAmp.wrist = restTremorAmp.wrist * restWeight + actionTremorAmp.wrist * actionWeight;
        currentAmp.forearm = restTremorAmp.forearm * restWeight + actionTremorAmp.forearm * actionWeight;
        
        currentNoise_shoulder = noise_rest.shoulder(i) * restWeight + noise_action.shoulder(i) * smoothProgress;
        currentNoise_elbow = noise_rest.elbow(i) * restWeight + noise_action.elbow(i) * smoothProgress;
        currentNoise_wrist = noise_rest.wrist(i) * restWeight + noise_action.wrist(i) * smoothProgress;
        currentNoise_forearm = noise_rest.forearm(i) * restWeight + noise_action.forearm(i) * smoothProgress;
        
        phase_jitter = phase_jitter_rest(i) * restWeight + phase_jitter_action(i) * smoothProgress;
        
        currentPose = struct();
        currentPose.shoulder_elev = restPose.shoulder_elev + smoothProgress * (targetPose.shoulder_elev - restPose.shoulder_elev);
        currentPose.shoulder_add = restPose.shoulder_add + smoothProgress * (targetPose.shoulder_add - restPose.shoulder_add);
        currentPose.shoulder_rot = restPose.shoulder_rot + smoothProgress * (targetPose.shoulder_rot - restPose.shoulder_rot);
        currentPose.elbow_flex = restPose.elbow_flex + smoothProgress * (targetPose.elbow_flex - restPose.elbow_flex);
        
        if reachProgress > 0.5
            wristProgress = (reachProgress - 0.5) / 0.5;
            wristSmooth = wristProgress^2 * (3 - 2*wristProgress);
            currentPose.pro_sup = restPose.pro_sup + wristSmooth * (targetPose.pro_sup - restPose.pro_sup);
            currentPose.wrist_flex = restPose.wrist_flex + wristSmooth * (targetPose.wrist_flex - restPose.wrist_flex);
        else
            currentPose.pro_sup = restPose.pro_sup;
            currentPose.wrist_flex = restPose.wrist_flex;
        end
        currentPose.wrist_dev = restPose.wrist_dev;
        
        tremorWeight = restWeight + actionWeight;
        
    else
        timeInAction = t - reachPhaseEnd;
        tremorWeight = min(timeInAction / 1.0, 1.0);
        tremorFreq = actionTremorFreq;
        currentAmp = actionTremorAmp;
        currentPose = targetPose;
        phase_jitter = phase_jitter_action(i);
        currentNoise_shoulder = noise_action.shoulder(i);
        currentNoise_elbow = noise_action.elbow(i);
        currentNoise_wrist = noise_action.wrist(i);
        currentNoise_forearm = noise_action.forearm(i);
    end
    
    freq_variation = tremorFreq + 0.3*sin(0.2*t) + 0.1*cos(0.5*t);
    
    tremor_base = sin(2*pi*freq_variation*t + phase_jitter);
    tremor_harmonic2 = 0.25 * sin(2*pi*2*freq_variation*t + phase_jitter*1.3);
    tremor_harmonic3 = 0.1 * sin(2*pi*3*freq_variation*t + phase_jitter*0.7);
    
    tremor_pattern = (tremor_base + tremor_harmonic2 + tremor_harmonic3) * amp_modulation(i);
    
    for j = 0:numCoords-1
        coordName = char(coordSet.get(j).getName());
        coordLower = lower(coordName);
        
        baseValue = 0;
        tremorAmp = 0;
        noise_value = 0;
        
        %  Modified: Joint identification
        if contains(coordLower, 'shoulder')
            if contains(coordLower, 'elev') || contains(coordLower, 'flexion')
                baseValue = currentPose.shoulder_elev;
                tremorAmp = deg2rad(currentAmp.shoulder);
                noise_value = currentNoise_shoulder;
            elseif contains(coordLower, 'add') || contains(coordLower, 'abduction')
                baseValue = currentPose.shoulder_add;
                tremorAmp = deg2rad(currentAmp.shoulder * 0.7);
                noise_value = currentNoise_shoulder * 0.7;
            elseif contains(coordLower, 'rot')
                baseValue = currentPose.shoulder_rot;
                tremorAmp = deg2rad(currentAmp.shoulder * 0.5);
                noise_value = currentNoise_shoulder * 0.5;
            end
            
        elseif contains(coordLower, 'elbow') && contains(coordLower, 'flex')
            baseValue = currentPose.elbow_flex;
            tremorAmp = deg2rad(currentAmp.elbow);
            noise_value = currentNoise_elbow;
            
        elseif contains(coordLower, 'pro_sup') || contains(coordLower, 'forearm')
            baseValue = currentPose.pro_sup;
            tremorAmp = deg2rad(currentAmp.forearm);
            noise_value = currentNoise_forearm;
            
        % Main fix: Add strcmp for 'flexion' and 'deviation' 
        elseif contains(coordLower, 'wrist') || strcmp(coordLower, 'flexion') || strcmp(coordLower, 'deviation')
            if contains(coordLower, 'flex') || strcmp(coordLower, 'flexion')
                baseValue = currentPose.wrist_flex;
                tremorAmp = deg2rad(currentAmp.wrist);
                noise_value = currentNoise_wrist;
            elseif contains(coordLower, 'dev') || strcmp(coordLower, 'deviation') || contains(coordLower, 'ulnar') || contains(coordLower, 'radial')
                baseValue = currentPose.wrist_dev;
                tremorAmp = deg2rad(currentAmp.wrist * 0.7);
                noise_value = currentNoise_wrist * 0.7;
            end
        end
        
        value = baseValue + tremorWeight * tremorAmp * tremor_pattern + noise_value;
        motionData(i, j+2) = value;
    end
end

%% Save Motion file
disp(' ');
disp(' Save Motion file ');

fid = fopen(motionPath, 'w');
fprintf(fid, 'parkinsons_tremor_simulation\n');
fprintf(fid, 'version=1\n');
fprintf(fid, 'nRows=%d\n', nFrames);
fprintf(fid, 'nColumns=%d\n', numCoords + 1);
fprintf(fid, 'inDegrees=no\n\n');
fprintf(fid, 'Units are S.I. units (second, meters, Newtons, ...)\n');
fprintf(fid, 'Angles are in radians.\n\n');
fprintf(fid, 'endheader\n');

fprintf(fid, 'time\t');
for i = 0:numCoords-1
    coordName = char(coordSet.get(i).getName());
    if i < numCoords-1
        fprintf(fid, '%s\t', coordName);
    else
        fprintf(fid, '%s\n', coordName);
    end
end

for i = 1:nFrames
    for j = 1:(numCoords + 1)
        if j < numCoords + 1
            fprintf(fid, '%.6f\t', motionData(i, j));
        else
            fprintf(fid, '%.6f\n', motionData(i, j));
        end
    end
end

fclose(fid);
disp(['Motion file saved: ', motionPath]);

%% Identifying key joints
jointIndices = struct();

disp(' ');
disp(' Identifying joints ');
for i = 1:length(coordNames)
    nameLower = lower(coordNames{i});
    disp(['Joint ', num2str(i), ': ', coordNames{i}]);
    
    if contains(nameLower, 'pro_sup') || contains(nameLower, 'forearm') || contains(nameLower, 'pronation') || contains(nameLower, 'supination')
        jointIndices.forearm = i;
        disp('  --> Identified as FOREARM');
    end
    
    if strcmp(nameLower, 'flexion') || contains(nameLower, 'wrist_flex')
        jointIndices.wrist_flex = i;
        disp('  --> Identified as WRIST FLEXION');
    end
    
    if strcmp(nameLower, 'deviation') || contains(nameLower, 'wrist_dev')
        jointIndices.wrist_dev = i;
        disp('  Identified as WRIST DEVIATION');
    end
end

%% استخراج سیگنال‌های ساعد و مچ
rest_idx = (time >= 2) & (time <= restPhaseEnd);
reach_idx = (time > restPhaseEnd) & (time <= reachPhaseEnd);
action_idx = (time >= reachPhaseEnd + 2);

% Forearm signal
if isfield(jointIndices, 'forearm')
    forearm_signal = rad2deg(motionData(:, jointIndices.forearm+1));
else
    warning('Forearm joint not found! The first joint is used.');
    jointIndices.forearm = 1;
    forearm_signal = rad2deg(motionData(:, 2));
end

% Wrist signal
if isfield(jointIndices, 'wrist_flex')
    wrist_signal = rad2deg(motionData(:, jointIndices.wrist_flex+1));
else
    warning('Wrist joint not found! Second joint being used.');
    if numCoords >= 2
        jointIndices.wrist_flex = 2;
        wrist_signal = rad2deg(motionData(:, 3));
    else
        jointIndices.wrist_flex = 1;
        wrist_signal = rad2deg(motionData(:, 2));
    end
end

% Detrend signals for different phases
forearm_rest = detrend(forearm_signal(rest_idx));
forearm_action = detrend(forearm_signal(action_idx));
wrist_rest = detrend(wrist_signal(rest_idx));
wrist_action = detrend(wrist_signal(action_idx));
% detrended signals for the total
forearm_signal_detrend = detrend(forearm_signal);
wrist_signal_detrend = detrend(wrist_signal);

 
%% FIGURE 1: Effective muscle and movement alignment
fig1 = figure('Position', [50 50 1600 1000], 'Color', 'w', 'Name', 'Protocol & Affected Muscles');

% Panel 1: Protocol Timeline (larger)
subplot(4,1,1);
hold on;
rectangle('Position', [0, 0, restPhaseEnd, 1], 'FaceColor', [1 0.8 0.8], 'EdgeColor', [0.8 0 0], 'LineWidth', 4);
rectangle('Position', [restPhaseEnd, 0, reachPhaseEnd-restPhaseEnd, 1], 'FaceColor', [1 0.95 0.6], 'EdgeColor', [0.9 0.6 0], 'LineWidth', 4);
rectangle('Position', [reachPhaseEnd, 0, duration-reachPhaseEnd, 1], 'FaceColor', [0.8 0.9 1], 'EdgeColor', [0 0.3 0.8], 'LineWidth', 4);

text(restPhaseEnd/2, 0.5, {'REST PHASE', sprintf('(0-%.0fs)', restPhaseEnd), 'Rest Tremor'}, ...
    'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.6 0 0]);
text((restPhaseEnd+reachPhaseEnd)/2, 0.5, {'REACHING', sprintf('(%.0f-%.0fs)', restPhaseEnd, reachPhaseEnd), 'Transition'}, ...
    'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.7 0.4 0]);
text((reachPhaseEnd+duration)/2, 0.5, {'ACTION PHASE', sprintf('(%.0f-%.0fs)', reachPhaseEnd, duration), 'Action Tremor'}, ...
    'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0 0.2 0.7]);

xlim([0 duration]);
ylim([0 1]);
set(gca, 'YTick', [], 'FontSize', 13);
xlabel('Time (seconds)', 'FontSize', 14, 'FontWeight', 'bold');
title('EXPERIMENTAL PROTOCOL - Three-Phase Parkinsonian Tremor Simulation', 'FontSize', 16, 'FontWeight', 'bold');
grid on;
box on;

% Panel 2: Forearm Muscles
subplot(4,2,3);
axis off;
text(0.5, 0.98, 'FOREARM MUSCLES AFFECTED', 'FontSize', 13, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.2 0.4 0.8]);

muscles_forearm = {...
    '\bf1. Pronator Teres'; ...
    '   • Pronates forearm'; ...
    '   • Tremor Impact: HIGH'; ...
    ''; ...
    '\bf2. Pronator Quadratus'; ...
    '   • Primary pronator'; ...
    '   • Tremor Impact: HIGH'; ...
    ''; ...
    '\bf3. Supinator'; ...
    '   • Supinates forearm'; ...
    '   • Tremor Impact: HIGH'; ...
    ''; ...
    '\bf4. Brachioradialis'; ...
    '   • Assists flexion'; ...
    '   • Tremor Impact: MODERATE'};

text(0.05, 0.88, muscles_forearm, 'FontSize', 10.5, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

% Panel 3: Wrist Muscles
subplot(4,2,4);
axis off;
text(0.5, 0.98, 'WRIST MUSCLES AFFECTED', 'FontSize', 13, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.8 0.2 0.4]);

muscles_wrist = {...
    '\bf1. Flexor Carpi Radialis'; ...
    '   • Flexion + Radial deviation'; ...
    '   • Tremor Impact: HIGH'; ...
    ''; ...
    '\bf2. Flexor Carpi Ulnaris'; ...
    '   • Flexion + Ulnar deviation'; ...
    '   • Tremor Impact: HIGH'; ...
    ''; ...
    '\bf3. Extensor Carpi Radialis'; ...
    '   • Extension + Radial deviation'; ...
    '   • Tremor Impact: MODERATE'; ...
    ''; ...
    '\bf4. Extensor Carpi Ulnaris'; ...
    '   • Extension + Ulnar deviation'; ...
    '   • Tremor Impact: MODERATE'};

text(0.05, 0.88, muscles_wrist, 'FontSize', 10.5, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

% Panel 4: Tremor Parameters - Forearm
subplot(4,2,5);
axis off;
text(0.5, 0.98, 'FOREARM TREMOR PARAMETERS', 'FontSize', 12, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.2 0.4 0.8]);

params_forearm = {...
    sprintf('\\bfRest Tremor:'); ...
    sprintf('  Frequency: %.1f Hz', restTremorFreq); ...
    sprintf('  Amplitude: %.1f degrees', restTremorAmp.forearm); ...
    ''; ...
    sprintf('\\bfAction Tremor:'); ...
    sprintf('  Frequency: %.1f Hz', actionTremorFreq); ...
    sprintf('  Amplitude: %.1f degrees', actionTremorAmp.forearm); ...
    ''; ...
    sprintf('\\bfAmplitude Increase:'); ...
    sprintf('  %.0f%% increase during action', ...
        100*(actionTremorAmp.forearm-restTremorAmp.forearm)/restTremorAmp.forearm)};

text(0.05, 0.88, params_forearm, 'FontSize', 10.5, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

% Panel 5: Tremor Parameters - Wrist
subplot(4,2,6);
axis off;
text(0.5, 0.98, 'WRIST TREMOR PARAMETERS', 'FontSize', 12, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.8 0.2 0.4]);

params_wrist = {...
    sprintf('\\bfRest Tremor:'); ...
    sprintf('  Frequency: %.1f Hz', restTremorFreq); ...
    sprintf('  Amplitude: %.1f degrees', restTremorAmp.wrist); ...
    ''; ...
    sprintf('\\bfAction Tremor:'); ...
    sprintf('  Frequency: %.1f Hz', actionTremorFreq); ...
    sprintf('  Amplitude: %.1f degrees', actionTremorAmp.wrist); ...
    ''; ...
    sprintf('\\bfAmplitude Increase:'); ...
    sprintf('  %.0f%% increase during action', ...
        100*(actionTremorAmp.wrist-restTremorAmp.wrist)/restTremorAmp.wrist)};

text(0.05, 0.88, params_wrist, 'FontSize', 10.5, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

% Panel 6: Technical Parameters
subplot(4,1,4);
axis off;
text(0.5, 0.95, 'SIMULATION TECHNICAL PARAMETERS', 'FontSize', 13, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);

params_tech = {...
    sprintf('\\bfDuration & Sampling:  Total = %.0f sec  |  Sampling Rate = %.0f Hz  |  SNR = %.0f dB', duration, Fs, SNR_dB); ...
    ''; ...
    sprintf('\\bfPhase Breakdown:'); ...
    sprintf('  • Rest (0-%.0fs): Pure resting tremor, relaxed posture', restPhaseEnd); ...
    sprintf('  • Reaching (%.0f-%.0fs): Smooth transition from rest to target position', restPhaseEnd, reachPhaseEnd); ...
    sprintf('  • Action (%.0f-%.0fs): Sustained action tremor at target position', reachPhaseEnd, duration); ...
    ''; ...
    sprintf('\\bfBiomechanical Realism Features:'); ...
    sprintf('  • Harmonic content: 2nd (25%%) + 3rd (10%%) harmonics included'); ...
    sprintf('  • Amplitude modulation: 0.1 Hz sinusoidal (±15%%) for natural variability'); ...
    sprintf('  • Phase jitter: Cumulative random walk (σ=0.01) for irregularity'); ...
    sprintf('  • Frequency variation: ±0.3 Hz sinusoidal drift for physiological realism')};

text(0.05, 0.80, params_tech, 'FontSize', 10.5, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

%% FIGURE 2: Forearm time analysis
fig2 = figure('Position', [100 50 1700 950], 'Color', 'w', 'Name', 'Forearm Time-Domain Analysis');

% Subplot 1: Complete Signal / phase shading
subplot(4,1,1);
plot(time, forearm_signal, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.8);
hold on;

y_lim = [min(forearm_signal)-1 max(forearm_signal)+1];
patch([0 restPhaseEnd restPhaseEnd 0], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)], ...
    'r', 'FaceAlpha', 0.12, 'EdgeColor', 'none');
patch([restPhaseEnd reachPhaseEnd reachPhaseEnd restPhaseEnd], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)], ...
    'y', 'FaceAlpha', 0.12, 'EdgeColor', 'none');
patch([reachPhaseEnd duration duration reachPhaseEnd], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)], ...
    'b', 'FaceAlpha', 0.12, 'EdgeColor', 'none');

xline(restPhaseEnd, 'k--', 'LineWidth', 2.5);
xline(reachPhaseEnd, 'k--', 'LineWidth', 2.5);

xlim([0 60]);
ylim(y_lim);
ylabel('Angle (degrees)', 'FontSize', 13, 'FontWeight', 'bold');
title('FOREARM PRONATION/SUPINATION - Complete 60-Second Signal', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
legend('Forearm Angle', 'Location', 'best', 'FontSize', 11);
set(gca, 'FontSize', 12);

% Subplot 2: Rest Phase Detail
subplot(4,2,3);
plot(time(rest_idx), forearm_rest, 'Color', [0.8 0 0], 'LineWidth', 2);
xlim([2 restPhaseEnd]);
ylabel('Detrended Amplitude (deg)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('REST TREMOR (2-10s)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.8 0 0]);
grid on;
set(gca, 'FontSize', 11);

env_rest = abs(hilbert(forearm_rest));
stat_text = sprintf('Mean: %.3f°\nStd: %.3f°\nPeak: %.3f°\nRMS: %.3f°', ...
    mean(abs(forearm_rest)), std(forearm_rest), max(abs(forearm_rest)), rms(forearm_rest));
text(0.97, 0.96, stat_text, 'Units', 'normalized', 'FontSize', 10, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'BackgroundColor', [1 1 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Subplot 3: Action Phase Detail
subplot(4,2,4);
plot(time(action_idx), forearm_action, 'Color', [0 0.3 0.8], 'LineWidth', 2);
xlim([reachPhaseEnd+2 60]);
ylabel('Detrended Amplitude (deg)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('ACTION TREMOR (22-60s)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0.3 0.8]);
grid on;
set(gca, 'FontSize', 11);

env_action = abs(hilbert(forearm_action));
stat_text = sprintf('Mean: %.3f°\nStd: %.3f°\nPeak: %.3f°\nRMS: %.3f°', ...
    mean(abs(forearm_action)), std(forearm_action), max(abs(forearm_action)), rms(forearm_action));
text(0.97, 0.96, stat_text, 'Units', 'normalized', 'FontSize', 10, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'BackgroundColor', [0.9 0.95 1], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Subplot 4: Overlay Rest vs Action 
subplot(4,2,5);
rest_sample_idx = (time >= 5) & (time <= 10);
action_sample_idx = (time >= 25) & (time <= 30);

plot(time(rest_sample_idx) - 5, forearm_signal_detrend(rest_sample_idx), 'r-', 'LineWidth', 2);
hold on;
plot(time(action_sample_idx) - 25, forearm_signal_detrend(action_sample_idx), 'b-', 'LineWidth', 2);
xlim([0 5]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Amplitude (deg)', 'FontSize', 12, 'FontWeight', 'bold');
title('5-Second Comparison: Rest vs Action', 'FontSize', 13, 'FontWeight', 'bold');
legend('Rest (5-10s)', 'Action (25-30s)', 'Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 11);

% Subplot 5: Envelope Comparison
subplot(4,2,6);
plot(time(rest_idx), env_rest, 'Color', [0.8 0 0], 'LineWidth', 2.5);
hold on;
plot(time(action_idx), env_action, 'Color', [0 0.3 0.8], 'LineWidth', 2.5);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Tremor Envelope (deg)', 'FontSize', 12, 'FontWeight', 'bold');
title('Tremor Envelope - Rest vs Action', 'FontSize', 13, 'FontWeight', 'bold');
legend('Rest Envelope', 'Action Envelope', 'Location', 'best', 'FontSize', 11);
grid on;
set(gca, 'FontSize', 11);

% Subplot 6: Histogram Comparison
subplot(4,2,7);
histogram(forearm_rest, 50, 'FaceColor', [0.8 0 0], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
hold on;
histogram(forearm_action, 50, 'FaceColor', [0 0.3 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
xlabel('Amplitude (deg)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Count', 'FontSize', 12, 'FontWeight', 'bold');
title('Amplitude Distribution', 'FontSize', 13, 'FontWeight', 'bold');
legend('Rest', 'Action', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 11);

% Subplot 7: Statistical Summary
subplot(4,2,8);
axis off;
text(0.5, 0.95, 'STATISTICAL SUMMARY - FOREARM', 'FontSize', 12, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.2 0.2 0.2]);

summary_stats = {...
    '\bfREST PHASE (2-10s):'; ...
    sprintf('  Mean Amplitude: %.4f deg', mean(abs(forearm_rest))); ...
    sprintf('  Std Deviation: %.4f deg', std(forearm_rest)); ...
    sprintf('  RMS: %.4f deg', rms(forearm_rest)); ...
    sprintf('  Peak-to-Peak: %.4f deg', max(forearm_rest) - min(forearm_rest)); ...
    ''; ...
    '\bfACTION PHASE (22-60s):'; ...
    sprintf('  Mean Amplitude: %.4f deg', mean(abs(forearm_action))); ...
    sprintf('  Std Deviation: %.4f deg', std(forearm_action)); ...
    sprintf('  RMS: %.4f deg', rms(forearm_action)); ...
    sprintf('  Peak-to-Peak: %.4f deg', max(forearm_action) - min(forearm_action)); ...
    ''; ...
    '\bfCOMPARISON:'; ...
    sprintf('  RMS Increase: %.1f%%', 100*(rms(forearm_action)-rms(forearm_rest))/rms(forearm_rest)); ...
    sprintf('  Std Increase: %.1f%%', 100*(std(forearm_action)-std(forearm_rest))/std(forearm_rest))};

text(0.1, 0.85, summary_stats, 'FontSize', 10, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

%% FIGURE 3: Wrist time analysis
fig3 = figure('Position', [150 50 1700 950], 'Color', 'w', 'Name', 'Wrist Time-Domain Analysis');

% Subplot 1: Complete Signal / phase shading
subplot(4,1,1);
plot(time, wrist_signal, 'Color', [0.8 0.2 0.4], 'LineWidth', 1.8);
hold on;

y_lim = [min(wrist_signal)-1 max(wrist_signal)+1];
patch([0 restPhaseEnd restPhaseEnd 0], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)], ...
    'r', 'FaceAlpha', 0.12, 'EdgeColor', 'none');
patch([restPhaseEnd reachPhaseEnd reachPhaseEnd restPhaseEnd], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)], ...
    'y', 'FaceAlpha', 0.12, 'EdgeColor', 'none');
patch([reachPhaseEnd duration duration reachPhaseEnd], [y_lim(1) y_lim(1) y_lim(2) y_lim(2)], ...
    'b', 'FaceAlpha', 0.12, 'EdgeColor', 'none');

xline(restPhaseEnd, 'k--', 'LineWidth', 2.5);
xline(reachPhaseEnd, 'k--', 'LineWidth', 2.5);

xlim([0 60]);
ylim(y_lim);
ylabel('Angle (degrees)', 'FontSize', 13, 'FontWeight', 'bold');
title('WRIST FLEXION/EXTENSION - Complete 60-Second Signal', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
legend('Wrist Angle', 'Location', 'best', 'FontSize', 11);
set(gca, 'FontSize', 12);

% Subplot 2: Rest Phase Detail
subplot(4,2,3);
plot(time(rest_idx), wrist_rest, 'Color', [0.8 0 0], 'LineWidth', 2);
xlim([2 restPhaseEnd]);
ylabel('Detrended Amplitude (deg)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('REST TREMOR (2-10s)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.8 0 0]);
grid on;
set(gca, 'FontSize', 11);

env_rest_w = abs(hilbert(wrist_rest));
stat_text = sprintf('Mean: %.3f°\nStd: %.3f°\nPeak: %.3f°\nRMS: %.3f°', ...
    mean(abs(wrist_rest)), std(wrist_rest), max(abs(wrist_rest)), rms(wrist_rest));
text(0.97, 0.96, stat_text, 'Units', 'normalized', 'FontSize', 10, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'BackgroundColor', [1 1 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Subplot 3: Action Phase Detail
subplot(4,2,4);
plot(time(action_idx), wrist_action, 'Color', [0 0.3 0.8], 'LineWidth', 2);
xlim([reachPhaseEnd+2 60]);
ylabel('Detrended Amplitude (deg)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
title('ACTION TREMOR (22-60s)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0 0.3 0.8]);
grid on;
set(gca, 'FontSize', 11);

env_action_w = abs(hilbert(wrist_action));
stat_text = sprintf('Mean: %.3f°\nStd: %.3f°\nPeak: %.3f°\nRMS: %.3f°', ...
    mean(abs(wrist_action)), std(wrist_action), max(abs(wrist_action)), rms(wrist_action));
text(0.97, 0.96, stat_text, 'Units', 'normalized', 'FontSize', 10, ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'BackgroundColor', [0.9 0.95 1], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Subplot 4: Overlay Rest vs Action 
subplot(4,2,5);
rest_sample_idx = (time >= 5) & (time <= 10);
action_sample_idx = (time >= 25) & (time <= 30);

plot(time(rest_sample_idx) - 5, wrist_signal_detrend(rest_sample_idx), 'r-', 'LineWidth', 2);
hold on;
plot(time(action_sample_idx) - 25, wrist_signal_detrend(action_sample_idx), 'b-', 'LineWidth', 2);
xlim([0 5]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Amplitude (deg)', 'FontSize', 12, 'FontWeight', 'bold');
title('5-Second Comparison: Rest vs Action', 'FontSize', 13, 'FontWeight', 'bold');
legend('Rest (5-10s)', 'Action (25-30s)', 'Location', 'best', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 11);

% Subplot 5: Envelope Comparison
subplot(4,2,6);
plot(time(rest_idx), env_rest_w, 'Color', [0.8 0 0], 'LineWidth', 2.5);
hold on;
plot(time(action_idx), env_action_w, 'Color', [0 0.3 0.8], 'LineWidth', 2.5);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Tremor Envelope (deg)', 'FontSize', 12, 'FontWeight', 'bold');
title('Tremor Envelope - Rest vs Action', 'FontSize', 13, 'FontWeight', 'bold');
legend('Rest Envelope', 'Action Envelope', 'Location', 'best', 'FontSize', 11);
grid on;
set(gca, 'FontSize', 11);

% Subplot 6: Histogram Comparison
subplot(4,2,7);
histogram(wrist_rest, 50, 'FaceColor', [0.8 0 0], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
hold on;
histogram(wrist_action, 50, 'FaceColor', [0 0.3 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
xlabel('Amplitude (deg)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Count', 'FontSize', 12, 'FontWeight', 'bold');
title('Amplitude Distribution', 'FontSize', 13, 'FontWeight', 'bold');
legend('Rest', 'Action', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 11);

% Subplot 7: Statistical Summary
subplot(4,2,8);
axis off;
text(0.5, 0.95, 'STATISTICAL SUMMARY - WRIST', 'FontSize', 12, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.2 0.2 0.2]);

summary_stats_w = {...
    '\bfREST PHASE (2-10s):'; ...
    sprintf('  Mean Amplitude: %.4f deg', mean(abs(wrist_rest))); ...
    sprintf('  Std Deviation: %.4f deg', std(wrist_rest)); ...
    sprintf('  RMS: %.4f deg', rms(wrist_rest)); ...
    sprintf('  Peak-to-Peak: %.4f deg', max(wrist_rest) - min(wrist_rest)); ...
    ''; ...
    '\bfACTION PHASE (22-60s):'; ...
    sprintf('  Mean Amplitude: %.4f deg', mean(abs(wrist_action))); ...
    sprintf('  Std Deviation: %.4f deg', std(wrist_action)); ...
    sprintf('  RMS: %.4f deg', rms(wrist_action)); ...
    sprintf('  Peak-to-Peak: %.4f deg', max(wrist_action) - min(wrist_action)); ...
    ''; ...
    '\bfCOMPARISON:'; ...
    sprintf('  RMS Increase: %.1f%%', 100*(rms(wrist_action)-rms(wrist_rest))/rms(wrist_rest)); ...
    sprintf('  Std Increase: %.1f%%', 100*(std(wrist_action)-std(wrist_rest))/std(wrist_rest))};

text(0.1, 0.85, summary_stats_w, 'FontSize', 10, 'VerticalAlignment', 'top', 'Interpreter', 'tex');

%% FIGURE 4: Frequency analysis of the forearm
fig4 = figure('Position', [200 50 1700 950], 'Color', 'w', 'Name', 'Forearm Frequency Analysis');

% FFT 
L_rest = length(forearm_rest);
Y_rest = fft(forearm_rest .* hamming(L_rest));
P_rest = abs(Y_rest/L_rest);
P_rest = P_rest(1:floor(L_rest/2)+1);
P_rest(2:end-1) = 2*P_rest(2:end-1);
f_rest = Fs*(0:(floor(L_rest/2)))/L_rest;

L_action = length(forearm_action);
Y_action = fft(forearm_action .* hamming(L_action));
P_action = abs(Y_action/L_action);
P_action = P_action(1:floor(L_action/2)+1);
P_action(2:end-1) = 2*P_action(2:end-1);
f_action = Fs*(0:(floor(L_action/2)))/L_action;

% Finding couriers
mask_rest = (f_rest >= 3) & (f_rest <= 7);
[~, idx_rest] = max(P_rest(mask_rest));
f_rest_range = f_rest(mask_rest);
peak_freq_rest = f_rest_range(idx_rest);

mask_action = (f_action >= 4) & (f_action <= 8);
[~, idx_action] = max(P_action(mask_action));
f_action_range = f_action(mask_action);
peak_freq_action = f_action_range(idx_action);

% Subplot 1: FFT Comparison
subplot(3,2,1);
plot(f_rest, P_rest, 'Color', [0.8 0 0], 'LineWidth', 2.5);
hold on;
plot(f_action, P_action, 'Color', [0 0.3 0.8], 'LineWidth', 2.5);
xline(restTremorFreq, '--', 'Color', [0.8 0 0], 'LineWidth', 2, 'Alpha', 0.7);
xline(actionTremorFreq, '--', 'Color', [0 0.3 0.8], 'LineWidth', 2, 'Alpha', 0.7);
plot(peak_freq_rest, P_rest(find(f_rest >= 3 & f_rest <= 7, 1) + idx_rest - 1), 'ro', 'MarkerSize', 14, 'LineWidth', 3, 'MarkerFaceColor', [1 0.5 0.5]);
plot(peak_freq_action, P_action(find(f_action >= 4 & f_action <= 8, 1) + idx_action - 1), 'bo', 'MarkerSize', 14, 'LineWidth', 3, 'MarkerFaceColor', [0.5 0.7 1]);
xlim([0 15]);
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Magnitude', 'FontSize', 12, 'FontWeight', 'bold');
title('FFT Spectrum - Forearm', 'FontSize', 14, 'FontWeight', 'bold');
legend_text = {'Rest', 'Action', ...
    sprintf('Rest Target (%.1f Hz)', restTremorFreq), ...
    sprintf('Action Target (%.1f Hz)', actionTremorFreq), ...
    sprintf('Rest Peak (%.2f Hz)', peak_freq_rest), ...
    sprintf('Action Peak (%.2f Hz)', peak_freq_action)};
legend(legend_text, 'Location', 'northeast', 'FontSize', 9);
grid on;
set(gca, 'FontSize', 11);

% Subplot 2: Power Spectral Density
subplot(3,2,2);
[pxx_rest, f_pxx_rest] = pwelch(forearm_rest, hamming(256), 128, 1024, Fs);
[pxx_action, f_pxx_action] = pwelch(forearm_action, hamming(256), 128, 1024, Fs);

plot(f_pxx_rest, 10*log10(pxx_rest), 'Color', [0.8 0 0], 'LineWidth', 2.5);
hold on;
plot(f_pxx_action, 10*log10(pxx_action), 'Color', [0 0.3 0.8], 'LineWidth', 2.5);
xline(restTremorFreq, '--', 'Color', [0.8 0 0], 'LineWidth', 2, 'Alpha', 0.7);
xline(actionTremorFreq, '--', 'Color', [0 0.3 0.8], 'LineWidth', 2, 'Alpha', 0.7);
xlim([0 15]);
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Power (dB/Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('Welch Power Spectral Density - Forearm', 'FontSize', 14, 'FontWeight', 'bold');
legend('Rest PSD', 'Action PSD', 'Location', 'northeast', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 11);

% Subplot 3: Spectrogram - Rest
subplot(3,2,3);
window_spec = hamming(512);
noverlap_spec = 480;
nfft_spec = 1024;

[S_rest, F_rest, T_rest] = spectrogram(forearm_rest, window_spec, noverlap_spec, nfft_spec, Fs);
imagesc(T_rest, F_rest, 10*log10(abs(S_rest) + eps));
axis xy;
ylim([0 15]);
colormap('jet');
c = colorbar;
c.Label.String = 'Power (dB)';
c.Label.FontSize = 11;
hold on;
yline(restTremorFreq, 'w--', 'LineWidth', 3);
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('Spectrogram - REST PHASE', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 11);

% Subplot 4: Spectrogram - Action
subplot(3,2,4);
[S_action, F_action, T_action] = spectrogram(forearm_action, window_spec, noverlap_spec, nfft_spec, Fs);
imagesc(T_action, F_action, 10*log10(abs(S_action) + eps));
axis xy;
ylim([0 15]);
colormap('jet');
c = colorbar;
c.Label.String = 'Power (dB)';
c.Label.FontSize = 11;
hold on;
yline(actionTremorFreq, 'w--', 'LineWidth', 3);
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('Spectrogram - ACTION PHASE', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 11);

% Subplot 5: Complete Spectrogram
subplot(3,1,3);
signal_full = detrend(forearm_signal);
[S, F, T] = spectrogram(signal_full, window_spec, noverlap_spec, nfft_spec, Fs);

imagesc(T, F, 10*log10(abs(S) + eps));
axis xy;
ylim([0 15]);
colormap('jet');
c = colorbar;
c.Label.String = 'Power (dB)';
c.Label.FontSize = 12;

hold on;
yline(restTremorFreq, 'w--', 'LineWidth', 3, 'Label', sprintf('Rest: %.1f Hz', restTremorFreq));
yline(actionTremorFreq, 'w--', 'LineWidth', 3, 'Label', sprintf('Action: %.1f Hz', actionTremorFreq));
xline(restPhaseEnd, 'w-', 'LineWidth', 3);
xline(reachPhaseEnd, 'w-', 'LineWidth', 3);

text(restPhaseEnd/2, 14, 'REST', 'Color', 'w', 'FontSize', 13, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.3]);
text((restPhaseEnd+reachPhaseEnd)/2, 14, 'REACH', 'Color', 'w', 'FontSize', 13, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.3]);
text((reachPhaseEnd+60)/2, 14, 'ACTION', 'Color', 'w', 'FontSize', 13, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.3]);

xlabel('Time (seconds)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Frequency (Hz)', 'FontSize', 13, 'FontWeight', 'bold');
title('COMPLETE TIME-FREQUENCY ANALYSIS - Forearm (0-60s)', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 12);

%% FIGURE 5: Wrist frequency analysis

fig5 = figure('Position', [250 50 1700 950], 'Color', 'w', 'Name', 'Wrist Frequency Analysis');

% FFT 
L_rest_w = length(wrist_rest);
Y_rest_w = fft(wrist_rest .* hamming(L_rest_w));
P_rest_w = abs(Y_rest_w/L_rest_w);
P_rest_w = P_rest_w(1:floor(L_rest_w/2)+1);
P_rest_w(2:end-1) = 2*P_rest_w(2:end-1);
f_rest_w = Fs*(0:(floor(L_rest_w/2)))/L_rest_w;

L_action_w = length(wrist_action);
Y_action_w = fft(wrist_action .* hamming(L_action_w));
P_action_w = abs(Y_action_w/L_action_w);
P_action_w = P_action_w(1:floor(L_action_w/2)+1);
P_action_w(2:end-1) = 2*P_action_w(2:end-1);
f_action_w = Fs*(0:(floor(L_action_w/2)))/L_action_w;

% پیدا کردن پیک‌ها
mask_rest_w = (f_rest_w >= 3) & (f_rest_w <= 7);
[~, idx_rest_w] = max(P_rest_w(mask_rest_w));
f_rest_w_range = f_rest_w(mask_rest_w);
peak_freq_rest_w = f_rest_w_range(idx_rest_w);

mask_action_w = (f_action_w >= 4) & (f_action_w <= 8);
[~, idx_action_w] = max(P_action_w(mask_action_w));
f_action_w_range = f_action_w(mask_action_w);
peak_freq_action_w = f_action_w_range(idx_action_w);

% Subplot 1: FFT Comparison
subplot(3,2,1);
plot(f_rest_w, P_rest_w, 'Color', [0.8 0 0], 'LineWidth', 2.5);
hold on;
plot(f_action_w, P_action_w, 'Color', [0 0.3 0.8], 'LineWidth', 2.5);
xline(restTremorFreq, '--', 'Color', [0.8 0 0], 'LineWidth', 2, 'Alpha', 0.7);
xline(actionTremorFreq, '--', 'Color', [0 0.3 0.8], 'LineWidth', 2, 'Alpha', 0.7);
plot(peak_freq_rest_w, P_rest_w(find(f_rest_w >= 3 & f_rest_w <= 7, 1) + idx_rest_w - 1), 'ro', 'MarkerSize', 14, 'LineWidth', 3, 'MarkerFaceColor', [1 0.5 0.5]);
plot(peak_freq_action_w, P_action_w(find(f_action_w >= 4 & f_action_w <= 8, 1) + idx_action_w - 1), 'bo', 'MarkerSize', 14, 'LineWidth', 3, 'MarkerFaceColor', [0.5 0.7 1]);
xlim([0 15]);
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Magnitude', 'FontSize', 12, 'FontWeight', 'bold');
title('FFT Spectrum - Wrist', 'FontSize', 14, 'FontWeight', 'bold');
legend_text_w = {'Rest', 'Action', ...
    sprintf('Rest Target (%.1f Hz)', restTremorFreq), ...
    sprintf('Action Target (%.1f Hz)', actionTremorFreq), ...
    sprintf('Rest Peak (%.2f Hz)', peak_freq_rest_w), ...
    sprintf('Action Peak (%.2f Hz)', peak_freq_action_w)};
legend(legend_text_w, 'Location', 'northeast', 'FontSize', 9);
grid on;
set(gca, 'FontSize', 11);

% Subplot 2: Power Spectral Density
subplot(3,2,2);
[pxx_rest_w, f_pxx_rest_w] = pwelch(wrist_rest, hamming(256), 128, 1024, Fs);
[pxx_action_w, f_pxx_action_w] = pwelch(wrist_action, hamming(256), 128, 1024, Fs);

plot(f_pxx_rest_w, 10*log10(pxx_rest_w), 'Color', [0.8 0 0], 'LineWidth', 2.5);
hold on;
plot(f_pxx_action_w, 10*log10(pxx_action_w), 'Color', [0 0.3 0.8], 'LineWidth', 2.5);
xline(restTremorFreq, '--', 'Color', [0.8 0 0], 'LineWidth', 2, 'Alpha', 0.7);
xline(actionTremorFreq, '--', 'Color', [0 0.3 0.8], 'LineWidth', 2, 'Alpha', 0.7);
xlim([0 15]);
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Power (dB/Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('Welch Power Spectral Density - Wrist', 'FontSize', 14, 'FontWeight', 'bold');
legend('Rest PSD', 'Action PSD', 'Location', 'northeast', 'FontSize', 10);
grid on;
set(gca, 'FontSize', 11);

% Subplot 3: Spectrogram - Rest
subplot(3,2,3);
[S_rest_w, F_rest_w, T_rest_w] = spectrogram(wrist_rest, window_spec, noverlap_spec, nfft_spec, Fs);
imagesc(T_rest_w, F_rest_w, 10*log10(abs(S_rest_w) + eps));
axis xy;
ylim([0 15]);
colormap('jet');
c = colorbar;
c.Label.String = 'Power (dB)';
c.Label.FontSize = 11;
hold on;
yline(restTremorFreq, 'w--', 'LineWidth', 3);
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('Spectrogram - REST PHASE', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 11);

% Subplot 4: Spectrogram - Action
subplot(3,2,4);
[S_action_w, F_action_w, T_action_w] = spectrogram(wrist_action, window_spec, noverlap_spec, nfft_spec, Fs);
imagesc(T_action_w, F_action_w, 10*log10(abs(S_action_w) + eps));
axis xy;
ylim([0 15]);
colormap('jet');
c = colorbar;
c.Label.String = 'Power (dB)';
c.Label.FontSize = 11;
hold on;
yline(actionTremorFreq, 'w--', 'LineWidth', 3);
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Frequency (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('Spectrogram - ACTION PHASE', 'FontSize', 13, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 11);

% Subplot 5: Complete Spectrogram
subplot(3,1,3);
signal_full_w = detrend(wrist_signal);
[S_w, F_w, T_w] = spectrogram(signal_full_w, window_spec, noverlap_spec, nfft_spec, Fs);

imagesc(T_w, F_w, 10*log10(abs(S_w) + eps));
axis xy;
ylim([0 15]);
colormap('jet');
c = colorbar;
c.Label.String = 'Power (dB)';
c.Label.FontSize = 12;

hold on;
yline(restTremorFreq, 'w--', 'LineWidth', 3, 'Label', sprintf('Rest: %.1f Hz', restTremorFreq));
yline(actionTremorFreq, 'w--', 'LineWidth', 3, 'Label', sprintf('Action: %.1f Hz', actionTremorFreq));
xline(restPhaseEnd, 'w-', 'LineWidth', 3);
xline(reachPhaseEnd, 'w-', 'LineWidth', 3);

text(restPhaseEnd/2, 14, 'REST', 'Color', 'w', 'FontSize', 13, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.3]);
text((restPhaseEnd+reachPhaseEnd)/2, 14, 'REACH', 'Color', 'w', 'FontSize', 13, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.3]);
text((reachPhaseEnd+60)/2, 14, 'ACTION', 'Color', 'w', 'FontSize', 13, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.3]);

xlabel('Time (seconds)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Frequency (Hz)', 'FontSize', 13, 'FontWeight', 'bold');
title('COMPLETE TIME-FREQUENCY ANALYSIS - Wrist (0-60s)', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 12);

%% 

disp(' ');
disp('SUCCESS: Analysis Complete!');
disp(['Motion file saved: ', motionPath]);
disp(' ');
disp('Generated Figures:');
disp('  Figure 1: Protocol & Affected Muscles');
disp('  Figure 2: Forearm Time-Domain Analysis');
disp('  Figure 3: Wrist Time-Domain Analysis');
disp('  Figure 4: Forearm Frequency Analysis');
disp('  Figure 5: Wrist Frequency Analysis');
disp(' ');
disp('Quantitative Results:');
disp(['  Forearm - Rest Peak: ', num2str(peak_freq_rest, '%.2f'), ' Hz, Action Peak: ', num2str(peak_freq_action, '%.2f'), ' Hz']);
disp(['  Wrist - Rest Peak: ', num2str(peak_freq_rest_w, '%.2f'), ' Hz, Action Peak: ', num2str(peak_freq_action_w, '%.2f'), ' Hz']);
disp(' ');
disp('Statistical Summary:');
disp('FOREARM:');
disp(['  Rest RMS: ', num2str(rms(forearm_rest), '%.4f'), ' deg, Action RMS: ', num2str(rms(forearm_action), '%.4f'), ' deg']);
disp(['  RMS Increase: ', num2str(100*(rms(forearm_action)-rms(forearm_rest))/rms(forearm_rest), '%.1f'), '%']);
disp('WRIST:');
disp(['  Rest RMS: ', num2str(rms(wrist_rest), '%.4f'), ' deg, Action RMS: ', num2str(rms(wrist_action), '%.4f'), ' deg']);
disp(['  RMS Increase: ', num2str(100*(rms(wrist_action)-rms(wrist_rest))/rms(wrist_rest), '%.1f'), '%']);
