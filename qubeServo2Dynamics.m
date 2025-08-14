function [f_CT, f_DT] = qubeServo2Dynamics(Ts)
% QUBESERVO2DYNAMICS   Returns function handles for continuous- and discrete-time dynamics of QUBE-Servo2.
%   [f_CT, f_DT] = qubeServo2Dynamics(Ts) returns:
%     f_CT: @(x,u) → ẋ for continuous-time dynamics,
%     f_DT: @(x,u) → x_{k+1} via RK4 with step Ts.
%
%   Ts: sampling period for discretization.
%
% Example:
%   [f_CT, f_DT] = qubeServo2Dynamics(0.01);
%   xnext = f_DT(x, u);

    %Rotary Arm
    mr = 0.095;  %[kg]
    r = 0.085;  %[m]
    Dr = 1e-3; %[Nms/rad]
    Jr = 1/3*mr*r^2; %[kgm^2]
    
    %Pendulum
    mp = 0.024;  %[kg]
    Lp = 0.129;   %[m]
    l = Lp/2;
    Dp = 5e-5; %[Nms/rad]
    Jp = 1/3*mp*Lp^2; %[kgm^2]
    
    %DC Motor
    Rm = 8.4;    %[ohm]
    kt = 0.042;  %[Nm/A]
    km = 0.042;  %[Vs/rad]
    
    %% Equations of motion
    %Constants
    g = 9.80665;    
    c1 = mp*l*r;
    c2 = mp*g*l;
    
    %CT Dynamics
    f1 = @(x)[Jr + Jp*(sin(x(3,:))^2)];
    f2 = @(x)[cos(x(3,:))];
    f3 = @(x)[sin(x(3,:))*cos(x(3,:))*x(2,:)*x(4,:)];
    f4 = @(x)[sin(x(3,:))*(x(4,:))^2];
    f5 = @(x)[sin(x(3,:))*cos(x(3,:))*x(2,:)^2];
    f6 = @(x)[sin(x(3,:))];
    f7 = @(x)[1/(f1(x)-(c1*f2(x))^2/Jp)];
    tauM = @(x,u)[km/Rm*(u-km*x(2,:))];
    f_Theta_double_dot = @(x,u)[f7(x)*(tauM(x,u) - Dr*x(2,:) - c1/Jp*f2(x)*(Jp*f5(x) - c2*f6(x) - Dp*x(4,:)) - 2*Jp*f3(x) + c1*f4(x))];
    f_Alpha_double_dot = @(x,u)[1/Jp*(-c1*f2(x)*f_Theta_double_dot(x,u) + Jp*f5(x) - c2*f6(x) - Dp*x(4,:))];
    f_CT = @(x,u)[x(2,:); f_Theta_double_dot(x,u); x(4,:); f_Alpha_double_dot(x,u)];
    
    %RK4 Discretization
    k1 = @(x,u)[f_CT(x,u)];
    k2 = @(x,u)[f_CT(x+Ts/2*k1(x,u),u)];
    k3 = @(x,u)[f_CT(x+Ts/2*k2(x,u),u)];
    k4 = @(x,u)[f_CT(x+Ts*k3(x,u),u)];
    f_DT = @(x,u)[x + Ts/6*(k1(x,u) + 2*k2(x,u) + 2*k3(x,u) + k4(x,u))];
    
    %f_DT = @(x,u) [x(:,:)+Ts*f_CT(x,u)];
end