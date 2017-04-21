classdef MotorFriction < handle
  
    properties(SetAccess=private, GetAccess=private)
        p1Velocity;
        p1Torque;
        p1Noise;
        p2Velocity;
        p2Torque;
        p2PWM
        p2Voltage
        p2Friction;
        th_velocity;
    end
    
    properties
        name;
        KcP;
        KcN;
        KvP;
        KvN;
        KtFirmware;
        Kt;
    end
    
    methods
        function obj = MotorFriction(name)
            %% Constructor
            if ~exist('name','var')
                obj.name = name;
            else
                obj.name = '';
            end
            
            obj.KcP = 0;
            obj.KcN = 0;
            obj.KvP = 0;
            obj.KvN = 0;
            obj.KtFirmware = 0;
            obj.Kt = 0;
            obj.th_velocity = 500;
            
        end

        function obj = Phase1(obj, data, th_velocity)
            %% Load Data for PWM=0 phase
            if exist('th_velocity','var')
                obj.th_velocity = th_velocity;
            end
            
            velocity = data.velocity;
            torque = data.torque;
            obj.p1Velocity = velocity;
            obj.p1Torque = torque;
                        % Evaluate fricition
            AP = MotorFriction.linearRegression(velocity(velocity > obj.th_velocity/2), ...
                torque(velocity > obj.th_velocity/2));
            AN = MotorFriction.linearRegression(velocity(velocity < -obj.th_velocity/2), ...
                torque(velocity < -obj.th_velocity/2));
           
            obj.KcP = AP(2);
            obj.KvP = AP(1);
            obj.KcN = AN(2);
            obj.KvN = AN(1);

            obj.p1Noise = torque-obj.getFriction(velocity);
        end
        
        function obj = Phase2(obj, data)
            %% Load data for second estimation phase
            obj.p2Velocity = data.velocity;
            obj.p2Torque = data.torque;
            obj.p2PWM = data.PWM;
            obj.p2Voltage = data.voltage;
            %% Evaluate coefficient for Kt motor
            obj.p2Friction = obj.getFriction(obj.p2Velocity);
            A = Joint.linearRegression(obj.p2PWM,obj.p2Torque-obj.p2Friction);
            B = Joint.linearRegression(obj.p2Voltage,obj.p2Torque-obj.p2Friction);
            obj.KtFirmware = A(1);
            obj.Kt = B(1);
        end

        function plotPhase1(obj)
            figure;
            subplot(2,1,1);
            hold on;
            obj.plotFriction();
            obj.plotFrictionModel();
            hold off;
            title('Phase 1 Friction estimate');
            subplot(2,1,2);
            obj.plotPhase1Noise();
            title('Phase 1 Estimation Noise');
        end 
        
        function plotPhase2(obj)
            %% Plot Collected images - Motor and model
            figure;
            subplot(2,1,1);
            hold on
            obj.plotMotor('Firmware');
            obj.plotMotorModel('Firmware');
            hold off
            
            subplot(2,1,2);
            hold on
            obj.plotMotor('');
            obj.plotMotorModel('');
            hold off
        end
    end
    
    
    methods (Access = private)         
            
        function friction = getFriction(obj, qdot)
            %% Evaluate friction model
            friction = zeros(size(qdot,1),1);
            friction(qdot > 0) = obj.KcP + obj.KvP*qdot(qdot > 0);
            friction(qdot == 0) = (obj.KcP-obj.KcN)/2 + obj.KcN;
            friction(qdot < 0) = obj.KcN + obj.KvN*qdot(qdot < 0);
        end
        
        function plotFrictionModel(obj, option)
            %% Plot Friction Model
            if ~exist('option','var')
                option = 'r';
            end
            
            hold on;
            x = 0:0.01:max(obj.P1velocity);
            plot(x, x*obj.KvP + obj.KcP,option,'LineWidth',3);
            x = min(obj.velocity):0.01:0;
            plot(x, x*obj.KvN + obj.KcN,option,'LineWidth',3);
            plot([0 0], [obj.KcP obj.KcN] ,option,'LineWidth',3);
            hold off;
        end
        
       function plotPhase1Noise(obj)
            %% Plot noise
            plot(obj.p1Velocity,obj.p1Noise);
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end
        
        function plotFriction(obj)
            %% Plot friction
            
            plot(obj.p1Velocity,obj.p1Torque);

            xlabel('qdot [deg]/[s] (Velocity)','Interpreter','tex');
            ylabel('\tau [Nm] (Torque)','Interpreter','tex');
        end
        
        
        function plotMotorModel(obj, typedata)
            %% Plot Motor model
            if strcmp(typedata,'Firmware')
                unitX = 'PWM';
                unitY = 'Torque Machine';
                Ktvalue = obj.KtFirmware;
            else
                unitX = 'V';
                unitY = 'Nm';
                Ktvalue = obj.Kt;
            end
            title(sprintf('Kt: %12.8f [%s]/[%s] - Name motor: %s',Ktvalue, unitY, unitX, obj.name));
            
            if strcmp(typedata,'Firmware')
                plot(obj.p2PWM, obj.p2PWM*obj.KtFirmware,'r-','LineWidth',3);
            else
                plot(obj.p2Voltage, obj.p2Voltage*obj.Kt,'r-','LineWidth',3);
            end
        end
        
        function plotMotor(obj, typedata, option)
            %% Plot measure versus friction estimation
            if ~exist('option','var')
                option = '.';
            end
            if strcmp(typedata,'Firmware')
                plot(obj.p2PWM, obj.p2Torque-obj.p2Friction, option);
                xlabel('PWM','Interpreter','tex');
                ylabel('TorqueMachine','Interpreter','tex');
            else
                plot(obj.p2Voltage, obj.p2Torque-obj.p2Friction, option);
                xlabel('Voltage','Interpreter','tex');
                ylabel('\tau-\tau_{f}','Interpreter','tex');
            end
        end
        
    end
    
    methods (Access = public, Static)
        
        function a = linearRegression(x, y)
            %% Linear regression to evalute coefficent for friction
            % Line equal y = a(1)*x + a(2)
            a = zeros(2,1);
            r = corrcoef(x,y); % Corr coeff is the off-diagonal (1,2) element
            r = r(1,2);  % Sample regression coefficient
            xbar = mean(x);
            ybar = mean(y);
            sigx = std(x);
            sigy = std(y);
            a1 = r*sigy/sigx;   % Regression line slope
            a(1) = a1;
            a(2) = ybar - a1*xbar;
        end
    end
    
end

