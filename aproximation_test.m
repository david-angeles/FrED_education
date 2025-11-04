%%%%%%%%%% TEST FOR THE PARAMETER ESTIMATION %%%%%%%%%%
%%% for a fisrt and second order system aproximation %%%
clc
clear
close all

%%%%%%%%%% VECTOR DATA ARE LOADED %%%%%%%%%%
%data_test = readmatrix('datos_minimos.csv');
%data_test = readmatrix('datos_escalon.csv');
data_test = readmatrix('FrED_dataResrs.txt');

time = data_test(:,1);           % time
rpm = ((2*pi)/60)*data_test(:,2);            % speed
control = data_test(:,4);        % voltage
input = (12/57)*data_test(:,4);          % input of the system

long = size (time);
time_sim = 0:0.02:time(end);


%Sample time is calculated
Tm = time(2) - time(1);        % tiempo de muestreo Tm

%%%%%%%%%% SECOND ORDER SYSTEM %%%%%%%%%%
a0 = -0.0844;%-0.0630; %-0.0644; %0.2916; %-0.3914; %0.3877; %0.2279; %0.0290;
a1 = 0.9664;%0.9351; %0.8474; %0.7061; %1.3798; %0.3717; %0.6796; %-0.2525;
b0 = 0.0584;%1.0611; %0.2123; %1.3091; %0.0147; %0.0676; %1.1650; %0.4480; %5.7563;

%%%%%%%%%% Firts ORDER SYSTEM %%%%%%%%%%
tau = 0.1477; %0.0898; %8.7174; %1.1578; %0.0546; %0.1375; %0.0266;
kD = 0.4954;%8.2975; %0.9799; %4.7942; %6.3643; %5.8620; %4.8410; %4.8487; %4.7049;
val1 = 0.8742;%0.8178;
val2 = 1.0441; %0.1786;
val3 = 0;

%%%%%%%%%% SIMULATION (IN CONTINUOS TIME)%%%%%%%%%%

numF = kD;
denF = [tau 1];
sysF = tf(numF, denF);
yFt = lsim(sysF, input, time_sim);

numS = 1731.3048;
denS = [1 472.6205 3495.7927];
sysS = tf(numS, denS);
ySt = lsim(sysS, input, time_sim);

%%%%%%%%%% SIMULATION (IN DISCRETE TIME)%%%%%%%%%%

for k = 1:1:long(1)
    if k == 1
        yS(k) = 0;
        yF(k) = 0 + val3;
    end
    if k == 2
        yS(k) = a1 * yS(k-1) + b0 * input(k-1);
        yF(k) = val1 * yF(k-1) + val2 * input(k-1) + val3;
    end
    if k >= 3
        yS(k) = a0 * yS(k-2) + a1 * yS(k-1) + b0 * input(k-1);
        yF(k) = val1 * yF(k-1) + val2 * input(k-1) +val3;
    end
end

figure;
plot(time, rpm, 'b', 'LineWidth', 1.5); hold on;
%plot(time, yF, 'm--', 'LineWidth', 1.5); hold on;
plot(time, yFt, 'm--', 'LineWidth', 1.5); hold on;
plot(time, ySt, 'c--', 'LineWidth', 1.5); hold on;
legend('Speed motor', 'First order estimation', 'Second order estimation', 'fontsize', 13);
xlabel('Time (s)', 'fontsize', 13);
ylabel('Speed (rad/s)', 'fontsize', 13);
%title('Estimation system');
grid on;
