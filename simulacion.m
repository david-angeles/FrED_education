%%%%%%%%%% SIMULATION PLOTS FOR FrED SYSTEM %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear
close all

time=0:0.01:10;

%%%%%%%%%% first order parameters %%%%%%%%%%
km = 0.4954;
tau = 0.1477;

%%%%%%%%%% second order parameters %%%%%%%%%%
b0 = 1731.3048;
a0 = 472.6205;
a1 = 3495.7927;

%%%%%%%%%% ganacias controladores %%%%%%%%%%
%%% parametros de diseño
mp = 0.05;  % maximo sobreimpulso
ts = 1;     % tiempo de establecimiento

chi = sqrt( (log(mp))^2 / (pi^2 + (log(mp))^2) );
%chi = 0.2;
wn = 4/(chi*ts);   %criterio del 2%

%extra_pole = real(roots_d(1)*5);

%Pe = extra_pole * (-1);

Pe = a0 + 50;

%%%%%% ganancias PID segundo orden %%%%%

kp_PID2 = (2*chi*wn*Pe+wn^2-a1)/b0;
kd_PID2 = (2*chi*wn+Pe-a0)/b0;
ki_PID2 = (wn^2*Pe)/b0;

%%%%%% ganancias PI segundo orden %%%%%

Pe = a0-2*chi*wn;
kp_PI2 = (2*chi*wn*Pe+wn^2-a1)/b0;
ki_PI2 = (wn^2*Pe)/b0;

%%%%%% ganancias PID primer orden %%%%%

kp_PID1 = (2*chi*wn-1)/km;
kd_PID1 = (1-tau)/km;
ki_PID1 = (wn^2)/km;

kd_PID12 = 0.03;
kp_PID12 = (2*chi*wn*(tau+km*kd_PID12)-1)/km;
ki_PID12 = (wn^2*(tau+km*kd_PID12))/km;

%%%%%% ganancias PI primer orden %%%%%

kp_PI1 = (2*chi*wn*tau-1)/km;
ki_PI1 = (wn^2*tau)/km;

%%%%%%%%%% Formacion de funciones de transferencia %%%%%%%%%%

numF = km;
denF = [tau 1];
sysF = tf(numF, denF);

numS = b0;
denS = [1 a0 a1];
sysS = tf(numS, denS);

PID1_num=[kd_PID1 kp_PID1 ki_PID1];
PID1_den=[1 0];
PID1_con=tf(PID1_num, PID1_den);

PI1_num=[kp_PI1 ki_PI1];
PI1_den=[1 0];
PI1_con=tf(PI1_num, PI1_den);

PID2_num=[kd_PID2 kp_PID2 ki_PID2];
PID2_den=[1 0];
PID2_con=tf(PID2_num, PID2_den);

PI2_num=[kp_PI2 ki_PI2];
PI2_den=[1 0];
PI2_con=tf(PI2_num, PI2_den);

%%%%%%%%%% closed loop systems %%%%%%%%%%
L_PID1 = PID1_con * sysF;               % PID open loop system with first order model
T_PID1 = feedback(L_PID1, 1);           % PID closed loop system with unity feedback

L_PI1 = PI1_con * sysF;                 % PI open loop system with first order model
T_PI1 = feedback(L_PI1, 1);             % PI closed loop system with unity feedback

L_PID2 = PID2_con * sysS;               % PID open loop system with second order model
T_PID2 = feedback(L_PID2, 1);           % PID closed loop system with unity feedback

L_PI2 = PI2_con * sysS;                 % PI open loop system with second order model
T_PI2 = feedback(L_PI2, 1);             % PI closed loop system with unity feedback

%%%%%%%%%% SYSTEM RESPONSE (IN CONTINUOS TIME)%%%%%%%%%%

ref = 3.14;
set_time = 1;
ts21 = 3.14 - 3.14*0.02; 
ts22 = 3.14 + 3.14*0.02; 
R_PID1 = ref * step(T_PID1, time);
R_PI1 = ref * step(T_PI1, time);
R_PID2 = ref * step(T_PID2, time);
R_PI2 = ref * step(T_PI2, time);

[u, t] = step(T_PID1, time);

%%%%%%%%%% PERFORMANCE EVALUATION %%%%%%%%%%
longitud = size(R_PID1);
IAE_PID1 = 0;
IAE_PI1 = 0;
IAE_PID2 = 0;
IAE_PI2 = 0;

for ind = 1:1:longitud(1)
    IAE_PID1= IAE_PID1 + abs(ref - R_PID1(ind));  % Integral of Absolute Error for PID1
    IAE_PI1 = IAE_PI1 + abs(ref - R_PI1(ind));  % Integral of Absolute Error for PID1

    IAE_PID2= IAE_PID2 + abs(ref - R_PID2(ind));  % Integral of Absolute Error for PID1
    IAE_PI2= IAE_PI2 + abs(ref - R_PI2(ind));  % Integral of Absolute Error for PID1

    %u_PID1= IAE_PID2 + abs(ref - R_PID2(ind));  % Integral of Absolute Error for PID1

end

IAE_PID1
IAE_PI1
IAE_PID2
IAE_PI2


% %%%%%%%%%% Gráficas %%%%%%%%%%
figure(1);
plot(time, R_PID1, 'b', 'LineWidth', 2); hold on;
yline(ref, '--g', 'LineWidth', 2);
xline(set_time, '--k');
yline(ts21, '--r');
yline(ts22, '--r');
legend('Motor speed', 'Reference', 'Settling time', 'Criteria 2%', 'fontsize', 13);
xlabel('Time (s)', 'fontsize', 13);
ylabel('Speed (rad/s)', 'fontsize', 13);
title('First order system responce PID');
%axis([0 10 0 4]);
grid on;

figure(2);
plot(time, R_PI1, 'b', 'LineWidth', 2); hold on;
yline(ref, '--g', 'LineWidth', 2);
xline(set_time, '--k');
yline(ts21, '--r');
yline(ts22, '--r');
legend('Motor speed', 'Reference', 'Settling time', 'Criteria 2%', 'fontsize', 13);
xlabel('Time (s)', 'fontsize', 13);
ylabel('Speed (rad/s)', 'fontsize', 13);
title('First order system responce PI');
%axis([0 335 0 80]);
grid on;

figure(3);
plot(time, R_PID2, 'b', 'LineWidth', 2); hold on;
yline(ref, '--g', 'LineWidth', 2);
xline(set_time, '--k');
yline(ts21, '--r');
yline(ts22, '--r');
legend('Motor speed', 'Reference', 'Settling time', 'Criteria 2%', 'fontsize', 13);
xlabel('Time (s)', 'fontsize', 13);
ylabel('Speed (rad/s)', 'fontsize', 13);
%title('Second order system responce PID');
%axis([0 335 0 80]);
grid on;

figure(4);
plot(time, R_PI2, 'b', 'LineWidth', 2); hold on;
yline(ref, '--g', 'LineWidth', 2);
xline(set_time, '--k');
yline(ts21, '--r');
yline(ts22, '--r');
legend('Motor speed', 'Reference', 'Settling time', 'Criteria 2%', 'fontsize', 13);
xlabel('Time (s)', 'fontsize', 13);
ylabel('Speed (rad/s)', 'fontsize', 13);
%title('Second order system responce PI');
%axis([0 335 0 80]);
grid on;

opts = bodeoptions;
%opts.LineWidth = 2;
%opts.PhaseLineWidth = 2;
opts.Title.FontSize = 13;
opts.XLabel.FontSize = 13;
opts.YLabel.FontSize = 13;

figure(5);
bode(sysF, opts)
%bode.LineWidth = 2;
title('');
grid on;
% Get the line handles
hline = findall(gcf, 'type', 'line');
% Set the linewidth
set(hline, 'LineWidth', 2);


figure(6);
bode(sysS, opts)
title('');
grid on;

% Get the line handles
hline = findall(gcf, 'type', 'line');
% Set the linewidth
set(hline, 'LineWidth', 2);

Cnum = [0.1472 9.0160] ;
Cden = [1 0];
Csys = tf(Cnum, Cden);
Cbode = sysS*Csys;

figure(7);
bode(Cbode, opts)
title('');
grid on;

% Get the line handles
hline = findall(gcf, 'type', 'line');
% Set the linewidth
set(hline, 'LineWidth', 2);

% buscar raices del denominador
denF_roots = roots(denF);
denS_roots = roots(denS);

den_PID1 = T_PID1.Denominator{1};
den_PI1 = T_PI1.Denominator{1};
den_PID2 = T_PID2.Denominator{1};
den_PI2 = T_PI2.Denominator{1};

denT11_roots = roots(den_PID1);
denT12_roots = roots(den_PI1);
denT21_roots = roots(den_PID2);
denT22_roots = roots(den_PI2);

% Plot the roots
figure(8);
plot(real(denF_roots), imag(denF_roots), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Real Part', 'fontsize', 13);
ylabel('Imaginary Part', 'fontsize', 13);
%title('Roots of the Denominator');
grid on;
%axis equal;


figure(9);
plot(real(denS_roots), imag(denS_roots), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Real Part', 'fontsize', 13);
ylabel('Imaginary Part', 'fontsize', 13);
%title('Roots of the Denominator');
grid on;
%axis equal;


figure(10);
plot(real(denT11_roots), imag(denT11_roots), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Real Part', 'fontsize', 13);
ylabel('Imaginary Part', 'fontsize', 13);
%title('Roots of the Denominator');
grid on;
%axis equal;

figure(11);
plot(real(denT12_roots), imag(denT12_roots), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Real Part', 'fontsize', 13);
ylabel('Imaginary Part', 'fontsize', 13);
%title('Roots of the Denominator');
grid on;
%axis equal;

figure(12);
plot(real(denT21_roots), imag(denT21_roots), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Real Part', 'fontsize', 13);
ylabel('Imaginary Part', 'fontsize', 13);
%title('Roots of the Denominator');
grid on;
%axis equal;

figure(13);
plot(real(denT22_roots), imag(denT22_roots), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Real Part', 'fontsize', 13);
ylabel('Imaginary Part', 'fontsize', 13);
%title('Roots of the Denominator');
grid on;
%axis equal;

%%%%%%%%%% Gráficas %%%%%%%%%%

