function [t, yout] = rk4(fcn, x0, u_control, dt, tend)
% Runge-Kutta 4 Integrator
% Function should be setup as y_dot = f(x, u, t)
% Hold control vector u piece-wise constant over the RK4 integration to the
% next time step
% Can update the control u at ever time step in advance to the RK4
% integration step

t = 0:dt:tend;
yout = zeros(length(x0), length(t));
yout(:, 1) = x0;
for i = 1:(length(t)-1)
    % Update control vector
    u = u_control;
    
    
    k1 = fcn(yout(:, i),u, i);
    k2 = fcn(yout(:, i) + k1 * dt/2, u, i + dt/2);
    k3 = fcn(yout(:, i) + k2 * dt/2, u, i + dt/2);
    k4 = fcn(yout(:, i) + k3 * dt,   u, i + dt);
    
    % Calculate y(i + dt)
    yout(:, i+1) = yout(:, i) + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
    
end
