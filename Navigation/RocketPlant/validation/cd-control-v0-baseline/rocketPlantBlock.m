function rocketPlantBlock(block)
%ROCKETPLANTBLOCK Stateless Level-2 Simulink adapter for the rocket plant.
% Parameters: Rocket configuration, mode (1 step, 2 kinematics, 3 forces),
% Weather configuration. Weather gust is an explicit input to step/forces.
% Every flight state belongs to the external Unit Delay. The adapter has no
% DWork, persistent variables, file input, random stream or internal memory.
% Pure physics helpers can also be exercised directly from MATLAB.
    block.NumDialogPrms = 3;
    mode = block.DialogPrm(2).Data;
    if mode == 1
        inputWidths = [17 1 1 3]; outputWidths = 17;
    elseif mode == 2
        inputWidths = 17; outputWidths = [4 3 3 3];
    else
        inputWidths = [17 1 1 3 3]; outputWidths = [3 3 18];
    end
    block.NumInputPorts = numel(inputWidths);
    block.NumOutputPorts = numel(outputWidths);
    block.SetPreCompPortInfoToDefaults;
    for k = 1:numel(inputWidths)
        block.InputPort(k).Dimensions = inputWidths(k);
        block.InputPort(k).DatatypeID = 0;
        block.InputPort(k).Complexity = 'Real';
        block.InputPort(k).DirectFeedthrough = true;
    end
    for k = 1:numel(outputWidths)
        block.OutputPort(k).Dimensions = outputWidths(k);
        block.OutputPort(k).DatatypeID = 0;
        block.OutputPort(k).Complexity = 'Real';
    end
    block.SampleTimes = [block.DialogPrm(1).Data.Ts 0];
    block.SimStateCompliance = 'HasNoSimState';
    block.RegBlockMethod('Outputs',@outputs);
end

function outputs(block)
    Rocket = block.DialogPrm(1).Data;
    Weather = block.DialogPrm(3).Data;
    x = block.InputPort(1).Data;
    mode = block.DialogPrm(2).Data;
    if mode == 1
        block.OutputPort(1).Data = rocketPlantStep(x, ...
            block.InputPort(2).Data,block.InputPort(3).Data,Rocket,Weather,block.InputPort(4).Data);
    elseif mode == 2
        [q,w,p,v] = rocketPlantKinematics(x,Rocket,Weather);
        block.OutputPort(1).Data = q;
        block.OutputPort(2).Data = w;
        block.OutputPort(3).Data = p;
        block.OutputPort(4).Data = v;
    else
        [f,B,dbg] = rocketPlantForces(x,block.InputPort(2).Data, ...
            block.InputPort(3).Data,block.InputPort(4).Data,Rocket,Weather,block.InputPort(5).Data);
        block.OutputPort(1).Data = f;
        block.OutputPort(2).Data = B;
        block.OutputPort(3).Data = dbg;
    end
end
