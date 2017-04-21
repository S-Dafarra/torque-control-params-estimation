classdef FrictionEstimation
    
    properties
        jointMap;
        jointCheckList;
    end    
    
    
    methods
        function obj = FrictionEstimation()
        end
        
        function obj = setList(obj, jointsList)
            if ((size(jointsList,1) ~= 1) && (size(jointsList,2) ~= 1))
                error('jointsList is expected to be a vector');
            end
            nJoints = length(jointsList);
            
            %% Create a Map indexed by the name of the joint + a checklist
            % The value is a struct containing:
            % - a cell array containing a reference to the corresponding 
            %   MotorFriction object, 
            % - a list containing the name of the other joint involved in
            %   the coupled joint,
            % - the coupling matrix T (which can be eventually 1 in case of
            %   decoupled joints).
            
            obj.jointCheckList = cell(nJoints, 1);
            values = cell(nJoints, 1);
            for iJoint = 1:nJoints
                obj.jointCheckList{iJoint} = jointsList(iJoint);
                
                %% CODE TO BE IMPLEMENTED
                [motorsList, T, coupledJoints] = getMotors(jointsList(iJoint));%%TO BE IMPLEMENTED
                %%
                
                values{iJoint} = struct;
                values{iJoint}.T = T;
                values{iJoint}.coupledJoints = coupledJoints; %ordered set of joints involved in the coupling. If it is not coupled, it corresponds to the joint itself
                
                values{iJoint}.motors = cell(length(motorsList),1);
                
                if any(ismember(obj.jointCheckList,coupledJoints)) %%if any of the joint included into the coupled joint has already been parsed
                    coupledRoot = coupledJoints(find(ismember(obj.jointCheckList,coupledJoints)));
                    for iMotor = 1:length(motorsList)
                        values{iJoint}.motors{iMotor} = values{coupledRoot}.motors{iMotor}; %% reference the same motorFriction object
                    end
                else 
                    for iMotor = 1:length(motorsList)
                        values{iJoint}.motors{iMotor} = MotorFriction(motorsList(iMotor));
                    end
                end
            end
            obj.jointMap = containers.Map(obj.jointCheckList,values);
            
        end
        
        function run(obj)  %% A sort of scratch for the actual state machine
            while ~isempty(obj.jointCheckList)
                joint = obj.jointCheckList(1);
                selectedJoints = obj.jointMap{joint}.coupledJoints; %Thus using the order corresponding to the coupling matrix 
                
                fprintf('Starting calibration of joints: %s. \n', selectedJoints);
                
                phase1Completed = false;
                phase2Completed = false;
                
                while ~phase1Completed
                    %%SETUP PHASE 1 TO BE INSERTED %%
                    fprintf('Now move the selected joints');
                    
                    %%GATHER DATA TO BE INSERTED %%
                    % The data needed for this phase are velocity (motor side)
                    % and torque (joint side). The data is expected to
                    % have as many ROWS as the number of joints. The
                    % order MUST be the same of the T matrix.
                    motorTorque = transpose(obj.jointMap{joint}.T)*TORQUE; %TORQUE TO BE DEFINED
                    for iMotor = 1:length(obj.jointMap{joint}.motors)
                        data = struct;
                        data.torque = motorTorque(iMotor,:); %TORQUE TO BE DEFINED
                        data.velocity = VELOCITY(iMotor,:); %VELOCITY TO BE DEFINED
                        obj.jointMap{joint}.motors{iMotor} = obj.jointMap{joint}.motors{iMotor}.Phase1(data);
                        obj.jointMap{joint}.motors{iMotor}.plotPhase1();
                    end
                    
                    str = input('Accept the first phase? [Y/n]','s');
                    if isempty(str)|| ( str == 'Y')
                        printf('Data Saved');
                        %%DO SOMETHING TO SAVE THE RESULTS
                        phase1Completed = true;
                    else 
                        str = input('Do you want to restart this phase [1] or skip the current joint(s) [2]. [Default 1]', 's');
                        if  ( str == '2')
                            phase1Completed = true;
                            phase2Completed = true;
                        end
                    end
                end
                
                while ~phase2Completed
                    %%SETUP PHASE 2 TO BE INSERTED %%
                    fprintf('Now move the selected joints');
                    
                    %%GATHER DATA TO BE INSERTED %%
                    % The data needed for this phase are velocity (motor side)
                    % and torque (joint side), PWM (motorside), voltage 
                    % (motorside, we can discuss on the necessity of this 
                    % datum). The data is expected to have as many ROWS as 
                    % the number of joints. The order MUST be the same of 
                    % the T matrix.
                    motorTorque = transpose(obj.jointMap{joint}.T)*TORQUE; %TORQUE TO BE DEFINED
                    for iMotor = 1:length(obj.jointMap{joint}.motors)
                        data = struct;
                        data.torque = motorTorque(iMotor,:); %TORQUE TO BE DEFINED
                        data.velocity = VELOCITY(iMotor,:); %VELOCITY TO BE DEFINED
                        data.PWM = PWM(iMotor, :); %PWM TO BE DEFINED
                        data.voltage = VOLTAGE(iMotor, :);
                        obj.jointMap{joint}.motors{iMotor} = obj.jointMap{joint}.motors{iMotor}.Phase2(data);
                        obj.jointMap{joint}.motors{iMotor}.plotPhase2();
                    end
                    
                    str = input('Accept the second phase? [Y/n]','s');
                    if isempty(str)|| ( str == 'Y')
                        printf('Data Saved');
                        %%DO SOMETHING TO SAVE THE RESULTS
                        phase2Completed = true;
                    else
                        str = input('Do you want to restart this phase [1] or the first one too [2]. [Default 1]', 's');
                        if  ( str == '2')
                            phase1Completed = false; % In this way the joints are not eliminated from the checkList and thus are selected again at the next iteration
                            %%DO SOMETHING TO REMOVE THE DATA SAVED AFTER
                            %%THE FIRST PHASE!!
                        end
                    end
                end
                
                if(phase1Completed == true) && (phase2Completed == true)
                    obj.jointCheckList = setdiff(obj.jointCheckList,selectedJoints,'stable'); %remove from the checkList the joints already claibrated, keeping the order.
                end
            end
        end
        
    end
end