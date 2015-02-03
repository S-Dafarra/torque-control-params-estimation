classdef Friction
    %FRICTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess=private, GetAccess=private)
        position;
        velocity;
        torque;
        time;
        step;
        noise;
    end
    
    properties
        th_velocity;
        KcP;
        KcN;
        KvP;
        KvN;
        KsP;
        KsN;
        offset = 0;
        cutoff;
    end
    
    methods
        function obj = Friction(position,velocity,torque,time,th_velocity, cutoff)
            obj.position = position;
            obj.velocity = velocity;
            obj.torque = torque;
            obj.time = time;
            obj.step = time(2)-time(1);
            
            if ~exist('cutoff','var')
                obj.cutoff = 1/obj.step;
            else
                obj.cutoff = cutoff;
            end
            
            obj = obj.evaluateCoeff(th_velocity);

        end
        
        function obj = evaluateCoeff(obj, th_velocity)
            %% Evaluate Coefficent
            if ~exist('th_velocity','var')
                obj.th_velocity = 1;
            else
                obj.th_velocity = th_velocity;
            end
            
            if obj.cutoff ~= 0
                fc = obj.cutoff;
                Fs = size(obj.time,1);
                Wn = (2/Fs)*fc;
                b = fir1(20,Wn,'low',kaiser(21,3));
                obj.torque = filter(b,1,obj.torque);
                obj.velocity = filter(b,1,obj.velocity);
            end
            
            % Evaluate fricition
            AP = obj.linearRegression(obj.velocity(obj.velocity > th_velocity/2), ...
                obj.torque(obj.velocity > th_velocity/2));
            AN = obj.linearRegression(obj.velocity(obj.velocity < -th_velocity/2), ...
                obj.torque(obj.velocity < -th_velocity/2));
            
            obj.KcP = AP(2);
            obj.KvP = AP(1);
            obj.KcN = AN(2);
            obj.KvN = AN(1);
            
            % if obj.cutoff ~= 0
            %     obj.noise = filter(b,1, obj.torque-obj.getFriction(obj.velocity));
            % else
            obj.noise = obj.torque-obj.getFriction(obj.velocity);
            % end
            obj.KsP = max(obj.noise);
            obj.KsN = min(obj.noise);
            
        end
        
        function obj = setToCenter(obj)
            %% Remove from data offset from wrong models
            if obj.KcP < 0
                obj.torque = -obj.torque;
                obj.KcP = -obj.KcP;
                obj.KcN = -obj.KcN;
                obj.KvP = -obj.KvP;
                obj.KvN = -obj.KvN;
            end
            obj.offset = mean(obj.torque);
            %Scaled Kc and Kv
            obj.offset = obj.KcP - obj.KcN;
            obj.torque = obj.torque-(obj.KcP-obj.offset/2);
            obj.KcP = obj.offset/2;
            obj.KcN = -obj.offset/2;
        end
        
        function friction = getFriction(obj, qdot)
            %% Evaluate friction model
            friction = zeros(size(qdot,1),1);
            friction(qdot < -obj.th_velocity/2) = obj.KcN + obj.KvN*qdot(qdot < -obj.th_velocity/2);
            friction(qdot > obj.th_velocity/2) = obj.KcP + obj.KvP*qdot(qdot > obj.th_velocity/2);
            friction(qdot >= -obj.th_velocity/2 & qdot <= obj.th_velocity/2) = obj.torque(qdot >= -obj.th_velocity/2 & qdot <= obj.th_velocity/2);
        end
        
        function plotFrictionModel(obj, option)
            %% Plot Friction Model
            if ~exist('option','var')
                option = 'r';
            end
            
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, x*obj.KvP + obj.KcP,option,'LineWidth',3);
            x = min(obj.velocity):obj.step:0;
            plot(x, x*obj.KvN + obj.KcN,option,'LineWidth',3);
            plot([0 0], [obj.KcP obj.KcN] ,option,'LineWidth',3);
            %plot([0 0], [obj.KsP obj.KsN] ,option,'LineWidth',3);
            hold off;
        end
        
        function saveToFile(joint, fileID)
            %% Save information about friction on file
            fprintf(fileID,'\n----------> Friction <----------\n');
            % Coefficients
            fprintf(fileID,'KcP: %12.8f [Nm] - KcN: %12.8f [Nm]\n',joint.friction.KcP, joint.friction.KcN);
            fprintf(fileID,'KsP: %12.8f [Nm][s]/[deg] - KvN: %12.8f [Nm][s]/[deg]\n',joint.friction.KvP, joint.friction.KvN);
            %fprintf(fileID,'KsP: %12.8f [Nm] - KsN %12.8f [Nm][s]/[deg]\n',joint.friction.KsP, joint.friction.KsN);
            
            fprintf(fileID,'\n---- Friction -> Latex ----\n');
            % To latex
            fprintf(fileID,'\\begin{equation}\n');
            fprintf(fileID,'\\label{eq:%sFrictionCoeffCoulomb}\n',joint.path);
            fprintf(fileID,'\\begin{array}{cccl}\n');
            fprintf(fileID,'\\bar K_{c+} & \\simeq & %12.8f & [Nm] %s\n',joint.friction.KcP,'\\');
            fprintf(fileID,'\\bar K_{c-} & \\simeq & %12.8f & [Nm]\n',joint.friction.KcN);
            fprintf(fileID,'\\end{array}\n');
            fprintf(fileID,'\\end{equation}\n');
            
            fprintf(fileID,'\n\\begin{equation}\n');
            fprintf(fileID,'\\label{eq:%sFrictionCoeffViscous}\n',joint.path);
            fprintf(fileID,'\\begin{array}{cccl}\n');
            fprintf(fileID,'\\bar K_{v+} & \\simeq & %12.8f & \\frac{[Nm][s]}{[deg]} %s\n',joint.friction.KvP,'\\');
            fprintf(fileID,'\\bar K_{v-} & \\simeq & %12.8f & \\frac{[Nm][s]}{[deg]}\n',joint.friction.KvN);
            fprintf(fileID,'\\end{array}\n');
            fprintf(fileID,'\\end{equation}\n');
            
%             fprintf(fileID,'\n\\begin{equation}\n');
%             fprintf(fileID,'\\label{eq:%sFrictionCoeffStiction}\n',joint.path);
%             fprintf(fileID,'\\begin{array}{cccl}\n');
%             fprintf(fileID,'\\bar K_{s+} & \\simeq & %12.8f & [Nm] %s\n',joint.friction.KsP,'\\');
%             fprintf(fileID,'\\bar K_{s-} & \\simeq & %12.8f & [Nm]\n',joint.friction.KsN);
%             fprintf(fileID,'\\end{array}\n');
%             fprintf(fileID,'\\end{equation}\n');
        end
        
        
        function plotNoise(obj, option)
            %% Plot noise
            if ~exist('option','var')
                option = '.';
            end
            plot(obj.velocity,obj.noise,option);
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end
        
        function plotParts(obj, option)
            %% Plot Parts
            if ~exist('option','var')
                option = '-';
            end
            
            subplot(1,2,1);
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, obj.KcP*ones(size(x,2),1),option);
            
            x = min(obj.velocity):obj.step:0;
            plot(x, obj.KcN*ones(size(x,2),1),option);
            
            plot([0 0], [obj.KcP obj.KcN] ,option);
            grid;
            hold off;
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
            
            subplot(1,2,2);
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, x*obj.KvP,option);
            x = min(obj.velocity):obj.step:0;
            plot(x, x*obj.KvN,option);
            grid;
            hold off;
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end

        function plotFriction(obj, option)
            %% Plot friction
            if ~exist('option','var')
                option = '.';
            end
            plot(obj.velocity, obj.torque, option);
            title('Relation between torque, velocity');
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end
        
        function savePictureToFile(obj, path, counter, figureName)
            %% Save Friction picture
            % FIGURE - Friction data and estimation
            if ~exist('counter','var')
                counter = 1;
            end
            hFig = figure(counter);
            set(hFig, 'Position', [0 0 800 600]);
            hold on
            grid;
            obj.plotFriction();
            obj.plotFrictionModel();
            clear friction_data;
            hold off
            % Save image
            currentFolder = pwd;
            cd(path);
            if ~exist('figureName','var')
                figureName = 'friction';
            end
            saveas(hFig,[figureName '.fig'],'fig');
            saveas(hFig,[figureName '.png'],'png');
            cd(currentFolder);
            clear currentFolder;
        end
        
        %% Get original data: Velocity and torque
        function data = getInfo(obj)
            data = struct;
            data.velocity = obj.velocity;
            data.torque = obj.torque;
        end
    end
    
    methods (Access = protected, Static)
        %% Linear regression to evalute coefficent for friction
        function a = linearRegression(x, y)
            N = size(x,1);
            % Add column of 1's to include constant term in regression
            X = [x ones(N,1)]; 
            % = [a1; a0]
            a = regress(y,X);
        end
    end
    
end

