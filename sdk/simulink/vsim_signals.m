function signals = vsim_signals(source)
%VSIM_SIGNALS The vsim signal list: name, unit, description and the port name for it.
%
%   s = vsim_signals('D:/projects/my_car_2026')                 % reads <project>/vsim_signals.json
%   s = vsim_signals('D:/projects/my_car_2026/vsim_signals.json')
%
% The list comes from the simulator (it depends on the vehicle: motors,
% sensors, plugins):
%
%   vsim-core --scenario <scenario.yaml> --list-signals <project>/vsim_signals.json
%
% Each entry has .name (drv.cmd.throttle), .unit, .description and .port
% (drv_cmd_throttle): a root Inport or Outport named .port is connected to
% that signal by vsim_export without any mapping. vsim_add_port adds one.
%
% See also VSIM_ADD_PORT, VSIM_EXPORT.
source = char(source);
if isfolder(source)
    source = fullfile(source, 'vsim_signals.json');
end
if ~isfile(source)
    error('vsim_signals:file', ['%s not found. Write it with\n  vsim-core --scenario <scenario.yaml> ' ...
        '--list-signals %s'], source, source);
end
raw = jsondecode(fileread(source));
signals = struct('name', {raw.name}, 'unit', {raw.unit}, 'description', {raw.description}, ...
    'port', cellfun(@(n) strrep(n, '.', '_'), {raw.name}, 'UniformOutput', false));
end
