%%%%%%%%%% PLOTS THE DATA FILE FROM FrED %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
close all

%%%%%%%%%% Data are loaded %%%%%%%%%%
%data = readmatrix('FrED_data3000-R.txt');
data = readmatrix('datos_graficas\FrED_data_chi02_PID2.txt');
time = data(:,1); 
rpm = ((2*pi)/60)*data(:,2);
control = data(:,3);
input = data(:,4);
PWM = data(:,5);

time100=0:0.1:10;
time500=0:0.5:10;
time1=0:1:10.2;
time2=0:2:10;


ref = 3.14;
set_time = 1;
ts21 = 3.14 - 3.14*0.02; 
ts22 = 3.14 + 3.14*0.02; 

%%%%%%%%%% PERFORMANCE EVALUATION %%%%%%%%%%
longitud = size(time);
IAE = 0;
IU = 0;

for ind = 1:1:longitud(1)
    IAE = IAE + abs(ref - rpm(ind));  % Integral of Absolute Error for PID1
    IU = IU + abs(input(ind));  % Integral of Absolute Error for PID1

end

IAE
IU


%%%%%%%%%% Gráficas %%%%%%%%%%
figure(1);
plot(time, rpm, 'b', 'LineWidth', 2); hold on;
yline(ref, '--g', 'LineWidth', 2);
xline(set_time, '--k');
yline(ts21, '--r');
yline(ts22, '--r');
legend('Motor speed', 'Reference', 'Settling time', 'Criteria 2%', 'fontsize', 13);
xlabel('Time (s)', 'fontsize', 13);
ylabel('Speed (rad/s)', 'fontsize', 13);
%title('Motor speed');
axis([0 10 0 4]);
grid on;

figure(2);
plot(time, control, 'r', 'LineWidth', 1.5); hold on;
legend('Motor voltage');
xlabel('Time (s)');
ylabel('Voltage');
title('Control (v)');
%axis([0 335 0 80]);
grid on;

figure(3);
plot(time, PWM, 'm', 'LineWidth', 1.5); hold on;
legend('Motor PWM');
xlabel('Time (s)');
ylabel('PWM');
title('Control (PWM)');
%axis([0 335 0 80]);
grid on;

figure(4);
plot(time, rpm, 'b', 'LineWidth', 1.5); hold on;
plot(time, input, 'r', 'LineWidth', 1.5); hold on;
%plot(time, signalF2, 'm', 'LineWidth', 1.5); hold on;
legend('Motor speed (RPM)', 'Motor input', 'Filtered 2');
xlabel('Time (s)');
ylabel('Signal');
title('Filters');
%axis([0 335 0 80]);
grid on;

figure(5);
plot(time, rpm, 'b', 'LineWidth', 1.5); hold on;
plot(time, PWM, 'r', 'LineWidth', 1.5); hold on;
%plot(time, signalF2, 'm', 'LineWidth', 1.5); hold on;
legend('Motor speed (RPM)', 'PWM', 'Filtered 2');
xlabel('Time (s)');
ylabel('Signal');
title('Filters');
%axis([0 335 0 80]);
grid on;

figure(6);
plot(input, rpm, 'b', 'LineWidth', 1.5); hold on;
%plot(time, PWM, 'r', 'LineWidth', 1.5); hold on;
%plot(time, signalF2, 'm', 'LineWidth', 1.5); hold on;
legend('Motor speed');
xlabel('PWM');
ylabel('Motor speed (rad/s)');
title('Motor speed vs PWM');
%axis([0 335 0 80]);
grid on;

iteracion=1;
for sample=1:1:101
    rpm100(sample)=rpm(iteracion);
    iteracion=iteracion+4;
end

iteracion=1;
for sample=1:1:21
    rpm500(sample)=rpm(iteracion);
    iteracion=iteracion+25;
end

iteracion=1;
for sample=1:1:11
    rpm1(sample)=rpm(iteracion);
    iteracion=iteracion+50;
end

iteracion=1;
for sample=1:1:6
    rpm2(sample)=rpm(iteracion);
    iteracion=iteracion+101;
end

figure(7);
plot(time, rpm, 'b', 'LineWidth', 2); hold on;
%plot(time100, rpm100, 'g--', 'LineWidth', 2); hold on;
plot(time500, rpm500, 'r', 'LineWidth', 2); hold on;
%plot(time1, rpm1, 'r', 'LineWidth', 2); hold on;
%plot(time2, rpm2, 'c--', 'LineWidth', 2); hold on;

legend('Sample time: 0.02 s', 'Sample time: 0.5 s', 'Sample time: 0.5 s', 'Sample time: 1 s', 'fontsize', 13);
xlabel('Time (s)', 'fontsize', 13);
ylabel('Speed (rad/s)', 'fontsize', 13);
%title('Motor speed');
axis([0 10 0 6]);
grid on;