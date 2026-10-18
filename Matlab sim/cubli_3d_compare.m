clear;
close all;
clc;

% Simulation steps:
% 1. Load the paper parameters and the four initial conditions from Params.m.
% 2. Run the Simulink model; its To Workspace blocks output thetab,
%    thetabdot, thetabddot, thetaw, and thetawdot (wheel relative to body).
% 3. Run the same exact Eq. (9) model, with paper Eq. (36) friction,
%    using ode45 as an independent reference.
% 4. Resample the Simulink signals onto a uniform time grid.
% 5. Plot Simulink against ode45 (plus total energy) and animate.

scriptFolder = fileparts(mfilename("fullpath"));
run(fullfile(scriptFolder, "Params.m"));

modelName = "dynamic_of_reaction_wheel_mine";
animationSource = "simulink"; % "simulink" or "ode"
tEnd = 10;
dt = 0.01;
t = (0:dt:tEnd).';

% Simulink output: To Workspace blocks (or logged signals) named
% thetab, thetabddot, thetawdot, and optionally thetaw, all in radians.
out = sim(modelName, "StopTime", num2str(tEnd), ...
    "RelTol", "1e-8", "AbsTol", "1e-10", "MaxStep", num2str(dt));
simulinkData = readSimulinkOutput(out, t, thetaw0);

% ode45 reference with x = [thetab; thetabdot; thetaw; thetawdot].
p.Ibody = IcO_bar;
p.Iwheel = Iwg;
p.mgd = gravity_gain;
p.tau_c = tau_c;
p.bw = bw;
p.cd = cd;
p.w_eps = w_eps;
p.bc = bc;
p.tau = tau_input;
p.comOffset = com_angle_offset;
options = odeset("RelTol", 1e-8, "AbsTol", 1e-10);
[~, xOde] = ode45(@(t, x) cubliDynamics(t, x, p), t, x0, options);
thetabddotOde = zeros(size(t));
for k = 1:numel(t)
    dx = cubliDynamics(t(k), xOde(k, :).', p);
    thetabddotOde(k) = dx(2);
end
odeData = table(t, xOde(:, 1), xOde(:, 2), thetabddotOde, xOde(:, 3), xOde(:, 4), ...
    'VariableNames', simulinkData.Properties.VariableNames);

measurements = simulinkData;
plotComparison(simulinkData, odeData, p);

if animationSource == "simulink"
    animData = simulinkData;
else
    animData = odeData;
end
x = [animData.thetab, animData.thetabdot, animData.thetaw, animData.thetawdot];
animateCubli(animData.t, x, l, d, com_angle_offset);

function data = readSimulinkOutput(out, t, thetaw0)
    thetab = getSignal(out, "thetab", t, true);
    thetabddot = getSignal(out, "thetabddot", t, true);
    thetawdot = getSignal(out, "thetawdot", t, true);
    thetaw = getSignal(out, "thetaw", t, false);
    if isempty(thetaw)
        thetaw = thetaw0 + cumtrapz(t, thetawdot);
    end
    thetabdot = getSignal(out, "thetabdot", t, false);
    if isempty(thetabdot)
        thetabdot = gradient(thetab, t);
    end
    data = table(t, thetab, thetabdot, thetabddot, thetaw, thetawdot);
end

function y = getSignal(out, name, t, required)
    % Accepts To Workspace (Timeseries, Structure With Time, or Array) and
    % signal logging (out.logsout).
    y = [];
    if any(strcmp(out.who, name))
        v = out.get(name);
        if isa(v, "timeseries")
            ts = v.Time; ys = squeeze(v.Data);
        elseif isstruct(v)
            ts = v.time; ys = v.signals.values;
        else
            ts = out.tout; ys = v;
        end
    elseif any(strcmp(out.who, "logsout")) && ~isempty(out.logsout.find("Name", name))
        v = out.logsout.getElement(name).Values;
        ts = v.Time; ys = squeeze(v.Data);
    elseif required
        error("Simulink output '%s' not found. Add a To Workspace block " + ...
            "with Variable name '%s' (see README).", name, name);
    else
        return;
    end
    [ts, iu] = unique(ts(:));
    ys = ys(:);
    y = interp1(ts, ys(iu), t, "linear", "extrap");
end

function plotComparison(sl, ode, p)
    figure("Name", "Cubli measurements: Simulink vs ode45", "Color", "w");
    tiledlayout(4, 1);
    signals = ["thetab", "thetabddot", "thetawdot"];
    labels = ["\theta_b (deg)", "d^2\theta_b/dt^2 (rad/s^2)", ...
              "d\theta_w/dt relative to body (rad/s)"];
    scale = [180/pi, 1, 1];
    for i = 1:3
        nexttile;
        plot(sl.t, scale(i)*sl.(signals(i)), "LineWidth", 2);
        hold on;
        plot(ode.t, scale(i)*ode.(signals(i)), "--", "LineWidth", 1.5);
        grid on;
        ylabel(labels(i));
        legend("Simulink", "ode45", "Location", "best");
    end

    % Mechanical energy from paper Eq. (3) and Eq. (6). It can only fall
    % through friction/damping, or change through motor work.
    nexttile;
    plot(sl.t, totalEnergy(sl, p), "LineWidth", 2);
    hold on;
    plot(ode.t, totalEnergy(ode, p), "--", "LineWidth", 1.5);
    grid on;
    ylabel("Energy T+V (J)");
    legend("Simulink", "ode45", "Location", "best");
    xlabel("Time (s)");
end

function E = totalEnergy(data, p)
    E = 0.5*p.Ibody*data.thetabdot.^2 ...
      + 0.5*p.Iwheel*(data.thetabdot + data.thetawdot).^2 ...
      + p.mgd*sin(data.thetab + p.comOffset);
end

function tauF = motorFriction(ww, p)
    % Paper Eq. (36): sign(w)*(tau_c + bw*|w| + cd*w^2), sign smoothed by tanh.
    tauF = tanh(ww/p.w_eps)*(p.tau_c + p.bw*abs(ww) + p.cd*abs(ww)^2);

end

function dx = cubliDynamics(~, x, p)
    thetab = x(1);
    thetabdot = x(2);
    thetawdot = x(4);

    wheelTorque = p.tau - motorFriction(thetawdot, p);
    gravityTorque = p.mgd*cos(thetab + p.comOffset);

    % Exact paper Eq. (9), plus optional pivot damping bc (0 in the paper):
    % [Ibar Iw; 0 Iw]*[thetabddot; thetabddot+thetawddot]
    %     = [-mgd*cos(thetab+offset) - bc*thetabdot; tau - tau_f].
    accelerations = [p.Ibody p.Iwheel; 0 p.Iwheel] \ ...
        [-gravityTorque - p.bc*thetabdot; wheelTorque];
    thetabddot = accelerations(1);
    thetawddot = accelerations(2) - thetabddot;

    dx = [x(2); thetabddot; thetawdot; thetawddot];
end






function animateCubli(t, x, sideLength, centerDistance, comOffset)
    figure("Name", "Cubli 3D dynamics", "Color", "w");
    ax = axes;
    hold(ax, "on");
    grid(ax, "on");
    axis(ax, "equal");
    axis(ax, 1.25*[-sideLength sideLength -0.35*sideLength 1.5*sideLength ...
                   -0.35*sideLength 1.5*sideLength]);
    view(ax, 32, 20);
    xlabel(ax, "World X / pivot axis (m)");
    ylabel(ax, "World Y (m)");
    zlabel(ax, "World Z (m)");

    faceAngle = x(1, 1) + comOffset - pi/4;
    [vertices, faces] = cubeAtAngle(sideLength, faceAngle);
    body = patch(ax, "Faces", faces, "Vertices", vertices, ...
        "FaceColor", [0.30 0.65 0.95], "FaceAlpha", 0.28, ...
        "EdgeColor", [0.10 0.25 0.40], "LineWidth", 1.2);

    pivotLength = 1.25*sideLength;
    plot3(ax, [-pivotLength pivotLength], [0 0], [0 0], "r--", ...
        "LineWidth", 1.8);
    plot3(ax, 0, 0, 0, "ro", "MarkerFaceColor", "r", "MarkerSize", 8);
    text(ax, -pivotLength, 0, 0, "  pivot axis (X_p)", ...
        "Color", "r", "FontWeight", "bold");
    text(ax, 0, 0, 0, "  PIVOT", "Color", "r", "FontWeight", "bold");

    bodyAxisLength = 0.55*sideLength;
    bx = plot3(ax, nan, nan, nan, "Color", [0.85 0.15 0.15], "LineWidth", 2.5);
    by = plot3(ax, nan, nan, nan, "Color", [0.10 0.55 0.15], "LineWidth", 2.5);
    bz = plot3(ax, nan, nan, nan, "Color", [0.10 0.25 0.90], "LineWidth", 2.5);

    wheelRadius = 0.34*sideLength;
    wheelHalfWidth = 0.37*sideLength;
    wheel = plot3(ax, nan, nan, nan, "k-", "LineWidth", 2.2);
    wheelAxis = plot3(ax, nan, nan, nan, "m-", "LineWidth", 3);
    spoke = plot3(ax, nan, nan, nan, "Color", [0.85 0.35 0], "LineWidth", 2.5);
    centerMark = plot3(ax, nan, nan, nan, "ko", "MarkerFaceColor", "k");

    step = max(1, round(0.03/mean(diff(t))));
    circleAngle = linspace(0, 2*pi, 80);

    for k = 1:step:numel(t)
        frameStart = tic;
        % The center of mass lies on the cube diagonal, 45 deg from a face.
        comAngle = x(k, 1) + comOffset;
        faceAngle = comAngle - pi/4;
        bodyRotation = rotationX(faceAngle);
        [currentVertices, ~] = cubeAtAngle(sideLength, faceAngle);
        set(body, "Vertices", currentVertices);

        center = [0;
                  centerDistance*cos(comAngle);
                  centerDistance*sin(comAngle)];
        bodyAxes = bodyRotation;
        updateAxisLine(bx, center, bodyAxes(:, 1), bodyAxisLength);
        updateAxisLine(by, center, bodyAxes(:, 2), bodyAxisLength);
        updateAxisLine(bz, center, bodyAxes(:, 3), bodyAxisLength);

        wheelPointsBody = [zeros(size(circleAngle));
                           wheelRadius*cos(circleAngle);
                           wheelRadius*sin(circleAngle)];
        wheelPoints = center + bodyRotation*wheelPointsBody;
        set(wheel, "XData", wheelPoints(1, :), ...
            "YData", wheelPoints(2, :), "ZData", wheelPoints(3, :));

        axisEnds = center + [-wheelHalfWidth wheelHalfWidth; 0 0; 0 0];
        set(wheelAxis, "XData", axisEnds(1, :), ...
            "YData", axisEnds(2, :), "ZData", axisEnds(3, :));

        absoluteWheelAngle = x(k, 1) + x(k, 3);
        spokeEnd = center + [0;
                             wheelRadius*cos(absoluteWheelAngle);
                             wheelRadius*sin(absoluteWheelAngle)];
        set(spoke, "XData", [center(1) spokeEnd(1)], ...
            "YData", [center(2) spokeEnd(2)], ...
            "ZData", [center(3) spokeEnd(3)]);
        set(centerMark, "XData", center(1), "YData", center(2), "ZData", center(3));

        title(ax, sprintf("t = %.2f s,  \\theta_b = %.1f deg,  d\\theta_w/dt = %.1f rad/s", ...
            t(k), rad2deg(x(k, 1)), x(k, 4)));
        drawnow;
        if k + step <= numel(t) && strcmp(get(gcf, "Visible"), "on")
            pause(max(0, t(k + step) - t(k) - toc(frameStart)));
        end
    end

    legend(ax, [bx by bz wheel wheelAxis], ...
        ["body x_b", "body y_b", "body z_b", "reaction wheel", "wheel axis"], ...
        "Location", "northeastoutside");
end

function [vertices, faces] = cubeAtAngle(sideLength, angle)
    h = sideLength/2;
    bodyVertices = [-h 0 0; h 0 0; h sideLength 0; -h sideLength 0; ...
                    -h 0 sideLength; h 0 sideLength; ...
                     h sideLength sideLength; -h sideLength sideLength];
    vertices = (rotationX(angle)*bodyVertices.').';
    faces = [1 2 3 4; 5 8 7 6; 1 5 6 2; ...
             2 6 7 3; 3 7 8 4; 5 1 4 8];
end

function updateAxisLine(lineHandle, origin, direction, axisLength)
    endpoint = origin + axisLength*direction;
    set(lineHandle, "XData", [origin(1) endpoint(1)], ...
        "YData", [origin(2) endpoint(2)], ...
        "ZData", [origin(3) endpoint(3)]);
end

function R = rotationX(angle)
    R = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
end
