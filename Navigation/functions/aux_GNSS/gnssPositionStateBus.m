function stateBus = gnssPositionStateBus(initialState)
%GNSSPOSITIONSTATEBUS Describe the explicit GNSS receiver feedback state.
%   This setup helper preserves each field's dimensions and logical type.
%   The receiver state lives in a Simulink Unit Delay, with initial values
%   supplied by gnssPositionInitialize for each simulation.

    names = fieldnames(initialState);
    for index = numel(names):-1:1
        value = initialState.(names{index});
        elements(index) = Simulink.BusElement;
        elements(index).Name = names{index};
        elements(index).Dimensions = size(value);
        if islogical(value)
            elements(index).DataType = 'boolean';
        else
            assert(isa(value, 'double'));
            elements(index).DataType = 'double';
        end
    end
    stateBus = Simulink.Bus;
    stateBus.Elements = elements;
end
