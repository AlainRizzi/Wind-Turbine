classdef app1_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        GridLayout                      matlab.ui.container.GridLayout
        LeftPanel                       matlab.ui.container.Panel
        ExitButton                      matlab.ui.control.Button
        FrictionCoefficientSlider       matlab.ui.control.Slider
        FrictionCoefficientSliderLabel  matlab.ui.control.Label
        StopButton                      matlab.ui.control.Button
        StartButton                     matlab.ui.control.Button
        RadiusSlider                    matlab.ui.control.Slider
        RadiusSliderLabel               matlab.ui.control.Label
        PitchangleSlider                matlab.ui.control.Slider
        PitchangleSliderLabel           matlab.ui.control.Label
        WindVelocitySlider              matlab.ui.control.Slider
        WindVelocitySliderLabel         matlab.ui.control.Label
        RightPanel                      matlab.ui.container.Panel
        UIAxes                          matlab.ui.control.UIAxes
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    
    properties (Access = private)
    vrWorld   % VR world object
    vrFigure  % VR figure
    bladeNode % Rotating blade node
    % lambda = 7;
    % beta = 0;
    % R = 20;
    % V = 10;
    running = false; % Description
    end
    
    methods (Access = private)
        
        function drawcyl(app,ax,p1,p2,rad,color)
            h=norm(p2-p1);
            [X,Y,Z]=cylinder(rad,30);
            Z=Z*h;
            dir=(p2-p1)/h;
            zAxis=[0;0;1];
            v=cross(zAxis,dir);
            s=norm(v);
            c=dot(zAxis,dir);
            if s<1e-6
                R=eye(3);
            else
                vx=[0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
                R=eye(3)+vx+vx^2*((1-c)/s^2);
            end
            for i=1:size(X,1)
                for j=1:size(X,2)
                    pt=R*[X(i,j);Y(i,j);Z(i,j)];
                    X(i,j)=pt(1)+p1(1);
                    Y(i,j)=pt(2)+p1(2);
                    Z(i,j)=pt(3)+p1(3);
                end
            end
            surf(ax,X,Y,Z,'FaceColor',color,'EdgeColor','none');
            
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            view(app.UIAxes, 3);
            axis(app.UIAxes, 'equal');
            grid(app.UIAxes, 'on');
            xlabel(app.UIAxes, 'X');
            ylabel(app.UIAxes, 'Y');
            zlabel(app.UIAxes, 'Z');

            app.vrWorld = vrworld('wind_turbine6.x3d');
            open(app.vrWorld);
            app.vrFigure = vrfigure(app.vrWorld);  % Open VR figure separately
            app.bladeNode = vrnode(app.vrWorld, 'Blades');  % Link to rotating blades
            camlight(app.UIAxes, 'headlight');
            lighting(app.UIAxes, 'gouraud');
            material(app.UIAxes, 'dull');
        end

        % Value changed function: RadiusSlider
        function RadiusSliderValueChanged(app, event)
            value = app.RadiusSlider.Value;
            
        end

        % Value changed function: WindVelocitySlider
        function WindVelocitySliderValueChanged(app, event)
            value = app.WindVelocitySlider.Value;
            
        end

        % Value changed function: PitchangleSlider
        function PitchangleSliderValueChanged(app, event)
            value = app.PitchangleSlider.Value;
            
        end

        % Button pushed function: StartButton
        function StartButtonPushed(app, event)
                app.running = true;  % Enable run flag

    beta = app.PitchangleSlider.Value;
    R = app.RadiusSlider.Value;
    V = app.WindVelocitySlider.Value;
    b= app.FrictionCoefficientSlider.Value*2500;
    dt=.05;
    % t=0.1;
    Tfinal=200;

    % t_end = 20;

    % Constants
    % rho = 1.225;
    % J = 50000;
    % dt = 0.05;
    % N = round(t_end / dt);
    % t = linspace(0, t_end, N);
    % omega = zeros(1, N); theta = zeros(1, N);
    % omega(1) = 0.1; theta(1) = 0;
    % theta=turbine_dynamics(beta,R,V,dt,Tfinal)
    theta = turbine_dynamics(beta,R,V,dt,Tfinal,b);
    time = 0:dt:Tfinal;
    blade_length = R; % Length of blade
    hub = [0; 0; 0];   % Turbine center
    % 
    % Precompute rotation angles
    angles_deg = [0, 120, 240]; % Three blades

    for i = 1:length(time)
        if ~app.running
            break;
        end
        tic;
        angle = theta(i);  % In radians
        app.bladeNode.rotation = [0 0 1 angle];  % Z-axis rotation
        vrdrawnow;  % Update VR scene
        % pause(t);  % Smooth animation
        elapsed=toc;
        pause_time=max(0,dt-elapsed);
        pause(pause_time);
    cla(app.UIAxes); % Clear previous frame
    hold(app.UIAxes, 'on');

    for j = 1:3
        angle = theta(i) + deg2rad(angles_deg(j));
        p1 = hub;
        p2 = hub + blade_length * [cos(angle); sin(angle); 0]; % in X-Y plane
        app.drawcyl(app.UIAxes, p1, p2, 0.2, 'b');
    end

    xlim(app.UIAxes, [-R R]*1.2);
    ylim(app.UIAxes, [-R R]*1.2);
    zlim(app.UIAxes, [-R R]*0.2);
    drawnow;
    end
       

    % for i = 1:N-1
    %     if app.running==false  % Stop condition
    %         break;
    %     end
        % [omega(i+1), theta(i+1)] = rk4_step(omega(i), theta(i), dt, rho, J, R, V, lambda, beta);
        % time=0:dt:Tfinal;
        % plot(app.UIAxes, time, theta);
    % end
        end

        % Button pushed function: StopButton
        function StopButtonPushed(app, event)
            app.running=false;
        end

        % Value changed function: FrictionCoefficientSlider
        function FrictionCoefficientSliderValueChanged(app, event)
            value = app.FrictionCoefficientSlider.Value;
            
        end

        % Button pushed function: ExitButton
        function ExitButtonPushed(app, event)
            selection=uiconfirm(app.UIFigure,'Are you sure you want to exit?','Exit Confirmation','Options',{'Yes','No'},'DefaultOption','No','CancelOption','No');
            if strcmp(selection,'Yes')
                try
                    if isvalid(app.vrFigure)
                        app.running=false;
                        close(app.vrFigure);
                    end
                catch
                end
                delete(app);
            end
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 2x1 grid
                app.GridLayout.RowHeight = {480, 480};
                app.GridLayout.ColumnWidth = {'1x'};
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 1;
            else
                % Change to a 1x2 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {220, '1x'};
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 2;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 640 480];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {220, '1x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.BackgroundColor = [0.949 0.949 0.949];
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;

            % Create WindVelocitySliderLabel
            app.WindVelocitySliderLabel = uilabel(app.LeftPanel);
            app.WindVelocitySliderLabel.HorizontalAlignment = 'right';
            app.WindVelocitySliderLabel.Position = [65 452 77 22];
            app.WindVelocitySliderLabel.Text = 'Wind Velocity';

            % Create WindVelocitySlider
            app.WindVelocitySlider = uislider(app.LeftPanel);
            app.WindVelocitySlider.ValueChangedFcn = createCallbackFcn(app, @WindVelocitySliderValueChanged, true);
            app.WindVelocitySlider.Position = [58 436 110 3];

            % Create PitchangleSliderLabel
            app.PitchangleSliderLabel = uilabel(app.LeftPanel);
            app.PitchangleSliderLabel.HorizontalAlignment = 'right';
            app.PitchangleSliderLabel.Position = [65 384 64 22];
            app.PitchangleSliderLabel.Text = 'Pitch angle';

            % Create PitchangleSlider
            app.PitchangleSlider = uislider(app.LeftPanel);
            app.PitchangleSlider.Limits = [0 30];
            app.PitchangleSlider.ValueChangedFcn = createCallbackFcn(app, @PitchangleSliderValueChanged, true);
            app.PitchangleSlider.Position = [58 370 110 3];

            % Create RadiusSliderLabel
            app.RadiusSliderLabel = uilabel(app.LeftPanel);
            app.RadiusSliderLabel.HorizontalAlignment = 'right';
            app.RadiusSliderLabel.Position = [65 303 42 22];
            app.RadiusSliderLabel.Text = 'Radius';

            % Create RadiusSlider
            app.RadiusSlider = uislider(app.LeftPanel);
            app.RadiusSlider.Limits = [1 31];
            app.RadiusSlider.ValueChangedFcn = createCallbackFcn(app, @RadiusSliderValueChanged, true);
            app.RadiusSlider.Position = [58 294 110 3];
            app.RadiusSlider.Value = 1;

            % Create StartButton
            app.StartButton = uibutton(app.LeftPanel, 'push');
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);
            app.StartButton.BackgroundColor = [0.8118 0.902 0.7804];
            app.StartButton.Position = [53 134 100 23];
            app.StartButton.Text = 'Start';

            % Create StopButton
            app.StopButton = uibutton(app.LeftPanel, 'push');
            app.StopButton.ButtonPushedFcn = createCallbackFcn(app, @StopButtonPushed, true);
            app.StopButton.BackgroundColor = [0.7294 0.7882 0.7098];
            app.StopButton.Position = [53 96 100 23];
            app.StopButton.Text = 'Stop';

            % Create FrictionCoefficientSliderLabel
            app.FrictionCoefficientSliderLabel = uilabel(app.LeftPanel);
            app.FrictionCoefficientSliderLabel.HorizontalAlignment = 'right';
            app.FrictionCoefficientSliderLabel.Position = [65 229 104 22];
            app.FrictionCoefficientSliderLabel.Text = 'Friction Coefficient';

            % Create FrictionCoefficientSlider
            app.FrictionCoefficientSlider = uislider(app.LeftPanel);
            app.FrictionCoefficientSlider.Limits = [0 50];
            app.FrictionCoefficientSlider.ValueChangedFcn = createCallbackFcn(app, @FrictionCoefficientSliderValueChanged, true);
            app.FrictionCoefficientSlider.Position = [58 215 110 3];

            % Create ExitButton
            app.ExitButton = uibutton(app.LeftPanel, 'push');
            app.ExitButton.ButtonPushedFcn = createCallbackFcn(app, @ExitButtonPushed, true);
            app.ExitButton.BackgroundColor = [0.6118 0.6588 0.5804];
            app.ExitButton.Position = [53 59 100 23];
            app.ExitButton.Text = 'Exit';

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.BackgroundColor = [0.949 0.949 0.949];
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;

            % Create UIAxes
            app.UIAxes = uiaxes(app.RightPanel);
            title(app.UIAxes, 'Blade')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.View = [37.5 30];
            app.UIAxes.YGrid = 'on';
            app.UIAxes.ZGrid = 'on';
            app.UIAxes.Position = [16 6 398 468];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = app1_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end