%%%%%%%%%% PARAMETER ESTIMATION USING LEAST SQUARES %%%%%%%%%%
%%% for a fisrt and second order system aproximation %%%
clc
clear
close all

%%%%%%%%%% VECTOR DATA ARE LOADED %%%%%%%%%%
%data = readmatrix('datos_minimos.csv');
%data = readmatrix('datos_escalon.csv');
data = readmatrix('FrED_dataRes.txt');
time = data(:,1);           % time
rpm = ((2*pi)/60)*data(:,2);            % speed
%rpm = data(:,2);            % speed
%input = data(:,3);        % voltage
input = (12/57) * data(:,4);          % input of the system
%input = data(:,3);

long = size (time);
%timef = 0:0.326:time(end)+0.326;
timef = 0:0.02:time(end);

%Sample time is calculated
Tm = time(5) - time(4);        % tiempo de muestreo Tm

%%%%%%%%%% SECOND ORDER SYSTEM %%%%%%%%%%
phiS = [rpm(2:end-1) rpm(1:end-2) input(3:end)];
YS = rpm(3:end); 
theta_S = (phiS' * phiS) \ (phiS' * YS);

a0 = (-theta_S(1) - 2*theta_S(2))/(Tm * theta_S(2));
a02 = (-(theta_S(1)/theta_S(2))-2)/Tm;
%a0 = (-theta_S(1) + 2*theta_S(2))/(-Tm * theta_S(2));
a1 = (-1-(1+a0*Tm)*theta_S(2))/(Tm * theta_S(2));
a12 = ((-1/theta_S(2))-1-a0*Tm)/(Tm*Tm);

b0 = ((1+a0*Tm+a1*Tm*Tm)*theta_S(3))/(Tm*Tm);
b02 = ((1+a0*Tm+a12*Tm*Tm)*theta_S(3))/(Tm*Tm);

% Mostrar función de transferencia estimada
fprintf('\nFunción de transferencia estimada segundo orden (rad/s/V):\n');
fprintf('G(s) = %.4f / (s^2 + %.4f s + %.4f)\n', b02, a02, a12);

%%%%%%%%%% SECOND ORDER SYSTEM (other aproximation)%%%%%%%%%%
phiSZ = [rpm(2:end-1) rpm(1:end-2) input(3:end) input(2:end-1) input(1:end-2)];
YSZ = rpm(3:end); 
theta_SZ = (phiSZ' * phiSZ) \ (phiSZ' * YSZ);

sb0 = (theta_SZ(3)-theta_SZ(4)+theta_SZ(5))/(1-theta_SZ(1)+theta_SZ(2));
sb1 = ((theta_SZ(3)-theta_SZ(5))/(1-theta_SZ(1)+theta_SZ(2)))*(4/Tm);
sb2 = ((theta_SZ(3)+theta_SZ(4)+theta_SZ(5))/(1-theta_SZ(1)+theta_SZ(2)))*(4/Tm^2);

sa0 = ((1-theta_SZ(2))/(1-theta_SZ(1)+theta_SZ(2)))*(4/Tm);
sa1 = ((1+theta_SZ(1)+theta_SZ(2))/(1-theta_SZ(1)+theta_SZ(2)))*(4/Tm^2);

% Mostrar función de transferencia estimada
fprintf('\nFunción de transferencia estimada segundo orden (rad/s/V):\n');
fprintf('G(s) = %.4fs^2 + %.4f s + %.4f / (s^2 + %.4f s + %.4f)\n', sb0, sb1, sb2, sa0, sa1);



%%%%%%%%%% Firts ORDER SYSTEM %%%%%%%%%%
bias = ones (long(1)-1,1);
%phiF = [rpm(1:end-1) input(1:end-1) bias];
phiF = [rpm(1:end-1) input(2:end)];
YF = rpm(2:end);
theta_F = (phiF' * phiF) \ (phiF' * YF);

val1 = theta_F(1);
val2 = theta_F(2);

tau=(val1*Tm)/(1-val1);
kD =((tau + Tm)*val2)/Tm;

% Mostrar función de transferencia estimada
fprintf('\nFunción de transferencia estimada primer orden (rad/s/V):\n');
fprintf('G(s) = %.4f / (%.4f s + 1)\n', kD, tau);


%%%%%%%%%% SIMULATION (IN CONTINUOS TIME)%%%%%%%%%%

numF = kD;
denF = [tau 1];
sysF = tf(numF, denF);
yFt = lsim(sysF, input, timef);

numS = b02;
denS = [1 a02 a12];
sysS = tf(numS, denS);
ySt = lsim(sysS, input, timef);

numS2 = sb2;
denS2 = [1 sa0 sa1];
sysS2 = tf(numS2, denS2);
ySt2 = lsim(sysS2, input, timef);

%%%%%%%%%% SIMULATION (IN DISCRETE TIME)%%%%%%%%%%

for k = 1:1:long(1)
    if k == 1
        yS(k) = theta_S(3) * input(k);
        ySZ(k) = theta_SZ(5) * input(k);
        yF(k) = theta_F(2) * input(k);
    end
    if k == 2
        yS(k) = theta_S(1) * yS(k-1) + theta_S(3) * input(k);
        ySZ(k) = theta_SZ(1) * ySZ(k-1) + theta_SZ(3) * input(k) + ...
                 theta_SZ(4) * input(k-1);
        yF(k) = theta_F(1) * yF(k-1) + theta_F(2) * input(k);
    end
    if k >= 3
        yS(k) = theta_S(1) * yS(k-1) + theta_S(2) * yS(k-2) + theta_S(3) * input(k);
        ySZ(k) = theta_SZ(1) * ySZ(k-1) + theta_SZ(2) * ySZ(k-2) + ...
                 theta_SZ(3) * input(k) + theta_SZ(4) * input(k-1) + theta_SZ(5) * input(k-2);
        yF(k) = theta_F(1) * yF(k-1) + theta_F(2) * input(k);
    end
end

figure(1);
plot(time, rpm, 'b', 'LineWidth', 1.5); hold on;
plot(time, yF, 'm--', 'LineWidth', 1.5); hold on;
plot(time, yS, 'c--', 'LineWidth', 1.5); hold on;
plot(time, ySZ, 'r--', 'LineWidth', 1.5); hold on;
legend('Speed motor', 'First order estimation', ...
                    'Second order estimation', 'Second order estimation 2');
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Estimation system');
grid on;

figure(2);
plot(time, rpm, 'b', 'LineWidth', 1.5); hold on;
plot(time, yFt, 'm--', 'LineWidth', 1.5); hold on;
plot(time, ySt, 'c--', 'LineWidth', 1.5); hold on;
%plot(time, ySt2, 'r--', 'LineWidth', 1.5); hold on;
legend('Speed motor', 'First order estimation', 'Second order estimation');
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Estimation system');
grid on;



%a0 = (-theta_S(1) - 2*theta_S(2))/(Tm * theta_S(2));
%a02 = (-(theta_S(1)/theta_S(2))-2)/Tm;
%%a0 = (-theta_S(1) + 2*theta_S(2))/(-Tm * theta_S(2));
%a1 = (-1-(1+a0*Tm)*theta_S(2))/(Tm * theta_S(2));
%a12 = ((-1/theta_S(2))-1-a0*Tm)/(Tm*Tm);

%b0 = ((1+a0*Tm+a1*Tm*Tm)*theta_S(3))/(Tm*Tm);
%b02 = ((1+a0*Tm+a12*Tm*Tm)*theta_S(3))/(Tm*Tm);