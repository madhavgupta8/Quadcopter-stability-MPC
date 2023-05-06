clc;
clear;
global Duration Vx Vy
Vx = 5; % Wind speed
Vy = 5; % Wind speed

nx = 12;                   % # of state features
ny = 12;                   % # of output varibles
nu = 4;                    % # input of 4 squared angular velocity
nlobj = nlmpc(nx, ny, nu); % Create a MPC controller
% nlobj.Model.StateFcn = "QuadrotorStateFcn";
nlobj.Model.StateFcn = "drone";

%%
rng(0)                                     % Random number genarator
%validateFcns(nlobj,rand(nx,1),rand(nu,1)); % Check potential problems

Ts = 0.1;                    % sample time of 0.1 seconds
p = 28;                      % prediction horizon of 20 steps
m = 3;                       % control horizon of 3 steps (m < p)
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = m;

max = 10;
nlobj.MV = struct('Min',{0;0;0;0},'Max',{max;max;max;max});  % Constraints of manipulated varibles

% Limit the state varibles
nlobj.OV(4).Min = -50/180 * pi; % phi
nlobj.OV(4).Max = 50/180 * pi;
nlobj.OV(5).Min = -50/180 * pi; % the
nlobj.OV(5).Max = 50/180 * pi;

pos = 1.2;
ang = 0.5;
nlobj.Weights.OutputVariables = [pos pos pos ang ang ang 0 0 0 0 0 0];   % Weights of output varibles
w_mv= 0.01;
nlobj.Weights.ManipulatedVariables = [w_mv w_mv w_mv w_mv];      % Weights of manipulated varibles

w_mvr = 0.1;
nlobj.Weights.ManipulatedVariablesRate = [w_mvr w_mvr w_mvr w_mvr];  % Weights of the rate of manipulated varibles

% Specify the initial conditions
x = [5;-5;-5;0;0;0;0;0;0;0;0;0]; % 12 * 1
% Nominal control that keeps the quadrotor floating
nloptions = nlmpcmoveopt;

nloptions.MVTarget = [5, 5, 5, 5]; 
mv = nloptions.MVTarget;

% Simulate the closed-loop system using the nlmpcmove function, specifying simulation options using an nlmpcmove object.
Duration = 20;
hbar = waitbar(0,'Simulation Progress');
xHistory = x'; % 1 * 12
lastMV = mv;
uHistory = lastMV; % 1 * 4
for k = 1:(Duration/Ts)
    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p); % Predictive time span
    yref = QuadrotorReferenceTrajectory(t);

    % Compute the control moves with reference previewing.
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
    uHistory(k+1,:) = uk';
    lastMV = uk;

    % Update states.
    % ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    ODEFUN = @(t,xk) drone(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);
    waitbar(k*Ts/Duration,hbar);
end
% close(hbar)

%% Calculate the loss
calculate_loss;

%% Visualize
plotQuadrotorTrajectory;
% animateQuadrotorTrajectory;
