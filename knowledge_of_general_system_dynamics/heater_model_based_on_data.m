clc, clear, close all hidden;

% Show raw data

temperature_int = load('temperature_data.txt');
temperature = temperature_int ./ 1000; % int -> float
%temperature = temperature - temperature(1);
dt = 1; % one second
number_of_samples = length(temperature);
t = (0:number_of_samples-1)*dt;

figure;
plot(t, temperature, '.');
grid on;
ylabel('Temperature [Celsius]', 'Interpreter', 'LaTeX');
xlabel('Time [s]', 'Interpreter', 'LaTeX');

% Input signal
input_amplitude = 0.9; % 0.9 -> 90% of PWM duty (PWM1=90;)
input = input_amplitude*ones( 1 , number_of_samples); 

% LTI model (linear time-invariant model)
s = tf('s');
k = 7.7/input_amplitude;   %model gain
T = 325;    %model time constant
delay = 5;   %model delay
H = k/(1+s*T)*exp(-s*delay); % model
fprintf('Model parameters: k = %.2g, T = %g, delay = %g\n', k, T, delay);

% Model response
model_response = lsim(H,input,t);
model_response = model_response + temperature(1); %add offset

%Model error
residuum = temperature - model_response';
error_abs_sum = sum(abs(residuum));
fprintf('Model error sum(abs(residuum)) = %g\n', error_abs_sum);

hold on;
plot(t, model_response, '.');
legend('measurement samples', 'model reponse', Location='best');

figure;
plot(t,residuum, '.');
title('Residuum (measurement samples - model reponse)', 'Interpreter', 'LaTeX');
xlabel('Time [s]', 'Interpreter', 'LaTeX');
ylabel('Temperature [Celsius]', 'Interpreter', 'LaTeX');





