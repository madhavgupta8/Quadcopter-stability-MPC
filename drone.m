%% Model of drone
% s: state vector
% u: input of 4 rotor-speed^2
function sdot = drone(s, u)
global Vx Vy             % Wind speed

hover = 5;               % ref input after re-scaling 
m = 1.568;
g = 9.81;
l = 0.25;
k0 = 7.73e-6;            % actual k
hover0 = m * g / 4 / k0; % actual ref input
k = m * g / hover / 4;   % k after re-scaling


b0 = 1.28e-7;            % actual b
b = b0 * hover0 / hover; % b after re-scaling

Ixx = 0.0119;
Iyy = 0.0119;
Izz = 0.0223;
Ir0 = 3.357e-4;
Ir = Ir0 * sqrt(hover0) / sqrt(hover);
Ax = 2.3;
Ay = 2.3;
Az = 2.3;

% x = s(1);
% y = s(s);
% z = s(3);
phi = s(4); Cphi = cos(phi); Sphi = sin(phi); %Tphi = tan(phi);
the = s(5); Cthe = cos(the); Sthe = sin(the); Tthe = tan(the);
psi = s(6); Cpsi = cos(psi); Spsi = sin(psi); %Tpsi = tan(psi);
xdot = s(7);
ydot = s(8);
zdot = s(9);
phidot = s(10);
thedot = s(11);
psidot = s(12);

sdot = zeros(12,1);
sdot(1) = xdot;
sdot(2) = ydot;
sdot(3) = zdot;
sdot(4) = phidot;
sdot(5) = thedot;
sdot(6) = psidot;

T = k * sum(u); % Thrust force
DragMatrix = [Ax 0 0;
              0 Ay 0;
              0 0 Az];
xddyddzdd = -g * [0;0;1]...
            +T/m * [Cpsi*Sthe*Cphi+Spsi*Sphi;Spsi*Sthe*Cphi-Cpsi*Sphi;Cthe*Cphi]...
            -1/m * DragMatrix * [xdot-Vx;ydot-Vy;zdot];
sdot(7) = xddyddzdd(1);
sdot(8) = xddyddzdd(2);
sdot(9) = xddyddzdd(3);

% Now for the last 3 varibles
f = [0 phidot*Cphi*Tthe+thedot*Sphi/Cthe^2 -phidot*Sphi*Cthe+thedot*Cphi/Cthe^2;
     0 phidot*Sphi -phidot*Cphi;
     0 phidot*Cphi/Cthe+phidot*Sphi*Tthe/Cthe -phidot*Sphi/Cthe+thedot*Cphi*Tthe/Cthe];

Wn = [1 0 -Sthe;
      0 Cphi Cthe * Sphi;
      0 -Sphi Cthe * Cphi];

pqr = Wn * [phidot;thedot;psidot];
p = pqr(1);
q = pqr(2);
r = pqr(3);
tau_phi = l * k * (-u(2)+u(4));
tau_the = l * k * (-u(1)+u(3));
tau_psi = b * (u(2)+u(4)-u(1)-u(3));
pdqdrd = [(Iyy-Izz)*q*r/Ixx;(Izz-Ixx)*p*r/Iyy;(Ixx-Iyy)*p*q/Izz] + ...
         [tau_phi/Ixx;tau_the/Iyy;tau_psi/Izz] - ...
         Ir * [q/Ixx;-p/Iyy;0] * (sqrt(u(1)) - sqrt(u(2)) + sqrt(u(3)) - sqrt(u(4)));

phiddtheddpsidd = f * pqr + Wn \ pdqdrd;
sdot(10) = phiddtheddpsidd(1);
sdot(11) = phiddtheddpsidd(2);
sdot(12) = phiddtheddpsidd(3);
end