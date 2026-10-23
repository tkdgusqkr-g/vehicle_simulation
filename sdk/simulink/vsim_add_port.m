function h = vsim_add_port(model, signal, direction, signals)
%VSIM_ADD_PORT Add a root Inport or Outport for a vsim signal, named and with its unit.
%
%   s = vsim_signals('D:/projects/my_car_2026');
%   vsim_add_port('vcu_2026', 'drv.cmd.throttle', 'in', s)
%   vsim_add_port('vcu_2026', 'pt.motor.fl.torque_cmd', 'out', s)
%
% The port is named like the signal with '_' for '.', so vsim_export connects
% it without a mapping, and gets the signal's unit (vsim converts if the model
% works in other units: change the port's Unit and vsim scales the value).
%
% See also VSIM_SIGNALS, VSIM_EXPORT.
model = char(model);
signal = char(signal);
load_system(model);
match = find(strcmp({signals.name}, signal), 1);
if isempty(match)
    error('vsim_add_port:signal', 'No signal "%s" in the list (vsim_signals).', signal);
end
s = signals(match);
if strcmp(direction, 'in')
    library = 'simulink/Sources/In1';
    type = 'Inport';
    x = 30;
elseif strcmp(direction, 'out')
    library = 'simulink/Sinks/Out1';
    type = 'Outport';
    x = 600;
else
    error('vsim_add_port:direction', 'direction must be ''in'' or ''out''.');
end
existing = find_system(model, 'SearchDepth', 1, 'BlockType', type);
y = 30 + 40 * numel(existing);
h = add_block(library, [model '/' s.port], 'Position', [x, y, x + 30, y + 14]);
set_param(h, 'Description', sprintf('vsim signal %s: %s', s.name, s.description));
if ~isempty(s.unit) && ~strcmp(s.unit, '-')
    try
        set_param(h, 'Unit', s.unit);
    catch
        warning('vsim_add_port:unit', 'Simulink does not know the unit "%s"; set the port''s Unit yourself.', s.unit);
    end
end
end
