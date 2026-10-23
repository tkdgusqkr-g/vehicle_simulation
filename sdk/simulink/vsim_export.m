function result = vsim_export(model, varargin)
%VSIM_EXPORT Turn a Simulink model into a vsim plugin and register it in a vehicle project.
%
%   vsim_export('vcu_2026.slx', 'Project', 'D:/projects/my_car_2026')
%   vsim_export('vcu_2026', 'Project', p, 'Name', 'vcu', 'Slot', 'vcu')
%   vsim_export('tv_2026', 'Project', p, 'After', 'vcu', 'Route', 'fmu')
%
% Options
%   Project   vehicle project folder (contains project.yaml)            required
%   Name      component name in plugins/components.yaml      default: model name
%   Slot      'vcu' | 'driver' | '' : replace that built-in block   default: ''
%   After     'driver' | 'sensors' | 'vcu' : run after it            default: 'vcu'
%   Route     'auto' | 'native' | 'fmu'                               default: 'auto'
%   Signals   vsim signal list for mapping ports by name (see vsim_signals)
%             default: <Project>/vsim_signals.json if it exists
%   Build     build the native plugin with CMake right away           default: true
%   VsimRoot  the vsim source folder                default: two folders above this file
%
% Steps (design doc §11.3)
%   1. Checks the model: fixed-step solver, one sample time, scalar root ports.
%   2. Picks the route from your licenses: Embedded Coder -> C code with a vsim
%      wrapper -> native plugin (the same code that runs on the car, fastest);
%      otherwise FMU export (FMI 2.0 Co-Simulation).
%   3. Maps root ports to vsim signals: a port named like a signal with '_' for
%      '.' (drv_cmd_throttle -> drv.cmd.throttle) connects itself; others get
%      suggestions in the file and must be filled in. Port units (the Inport /
%      Outport Unit parameter) are converted by vsim.
%   4. Writes the component to <Project>/plugins/components.yaml with the model
%      version and file hash, and the plugin under <Project>/plugins/<Name>/.
%
% See also VSIM_SIGNALS, VSIM_ADD_PORT.

p = inputParser;
addRequired(p, 'model');
addParameter(p, 'Project', '');
addParameter(p, 'Name', '');
addParameter(p, 'Slot', '');
addParameter(p, 'After', 'vcu');
addParameter(p, 'Route', 'auto');
addParameter(p, 'Signals', '');
addParameter(p, 'Build', true);
addParameter(p, 'VsimRoot', '');
parse(p, model, varargin{:});
o = p.Results;

here = fileparts(mfilename('fullpath'));
if isempty(o.VsimRoot)
    o.VsimRoot = fileparts(fileparts(here));
end
project = char(o.Project);
if isempty(project) || ~isfile(fullfile(project, 'project.yaml'))
    error('vsim_export:project', 'Project must be a vehicle project folder with project.yaml (got "%s").', project);
end
[model_dir, mname, mext] = fileparts(char(model));
if isempty(mext)
    model_file = which(mname);
else
    model_file = fullfile(model_dir, [mname mext]);
end
load_system(char(model));
name = char(o.Name);
if isempty(name)
    name = mname;
end
if isempty(regexp(name, '^[A-Za-z][A-Za-z0-9_]*$', 'once'))
    error('vsim_export:name', 'Name "%s": use letters, digits and _ (it becomes a key in components.yaml).', name);
end
if ~isempty(o.Slot) && ~any(strcmp(o.Slot, {'vcu', 'driver'}))
    error('vsim_export:slot', 'Slot must be vcu, driver or empty.');
end
if ~any(strcmp(o.After, {'driver', 'sensors', 'vcu'}))
    error('vsim_export:after', 'After must be driver, sensors or vcu.');
end

%% 1. Model checks
if ~strcmp(get_param(mname, 'SolverType'), 'Fixed-step')
    error('vsim_export:solver', '%s: set the solver to fixed-step (Configuration Parameters > Solver).', mname);
end
step = slResolve(get_param(mname, 'FixedStep'), mname);
if ~isnumeric(step) || ~isscalar(step) || ~(step > 0)
    error('vsim_export:step', '%s: give a numeric fixed step size (the controller period, e.g. 0.01).', mname);
end
ts = Simulink.BlockDiagram.getSampleTimes(mname);
rates = [];
for k = 1:numel(ts)
    v = ts(k).Value;
    if isnumeric(v) && numel(v) == 2 && v(1) > 0 && isfinite(v(1))
        rates(end + 1) = v(1); %#ok<AGROW>
    end
end
rates = unique(rates);
if numel(rates) > 1
    error('vsim_export:rates', ['%s has several sample times (%s s). vsim calls the model at one rate: ' ...
        'use one sample time, or rate transitions inside an atomic subsystem.'], mname, mat2str(rates));
end
ports = root_ports(mname);
if isempty(ports.out)
    error('vsim_export:ports', '%s has no root Outport: nothing for vsim to read.', mname);
end
fprintf('vsim_export: %s, step %g s, %d inputs, %d outputs\n', mname, step, numel(ports.in), numel(ports.out));

%% 2. Route
has_ert = license('test', 'RTW_Embedded_Coder') && license('test', 'Real-Time_Workshop') && ...
    ~isempty(ver('embeddedcoder'));
has_fmu = exist('exportToFMU', 'file') == 2 || exist('exportToFMU2CS', 'file') == 2;
route = char(o.Route);
if strcmp(route, 'auto')
    if has_ert
        route = 'native';
    elseif has_fmu
        route = 'fmu';
    else
        error('vsim_export:license', ['Neither Embedded Coder (C code -> native plugin) nor FMU export is ' ...
            'available. Either one is needed: Embedded Coder + Simulink Coder + MATLAB Coder, or ' ...
            'Simulink Compiler / Simulink (R2023b+) FMU export.']);
    end
end
out_dir = fullfile(project, 'plugins', name);
if ~isfolder(out_dir)
    mkdir(out_dir);
end

switch route
    case 'native'
        if ~has_ert
            error('vsim_export:license', 'Route native needs Embedded Coder.');
        end
        [plugin_path, ports] = export_native(mname, name, step, ports, out_dir, o, here);
    case 'fmu'
        if ~has_fmu
            error('vsim_export:license', 'Route fmu needs FMU export (exportToFMU / exportToFMU2CS).');
        end
        plugin_path = export_fmu(mname, name, out_dir);
    otherwise
        error('vsim_export:route', 'Route must be auto, native or fmu.');
end

%% 3. Port mapping
signals = struct('name', {}, 'unit', {});
signal_file = char(o.Signals);
if isempty(signal_file) && isfile(fullfile(project, 'vsim_signals.json'))
    signal_file = fullfile(project, 'vsim_signals.json');
end
if ~isempty(signal_file)
    signals = vsim_signals(signal_file);
else
    warning('vsim_export:signals', ['No signal list: ports are mapped only by exact name. Write one with ' ...
        '"vsim-core --scenario <scenario> --list-signals %s".'], fullfile(project, 'vsim_signals.json'));
end
[in_lines, unmapped] = map_ports(ports.in, signals, true);
out_lines = map_ports(ports.out, signals, false);

%% 4. Register in the project
model_version = get_param(mname, 'ModelVersion');
lines = {sprintf('%s:', name), '  enabled: true', sprintf('  type: %s', route), ...
    sprintf('  path: %s', plugin_path)};
if ~isempty(o.Slot)
    lines{end + 1} = sprintf('  slot: %s', o.Slot);
else
    lines{end + 1} = sprintf('  after: %s', o.After);
end
lines{end + 1} = sprintf('  period: %.12g', step);
lines{end + 1} = sprintf('  source: {model: "%s", version: "%s", sha256: %s, route: %s, exported: "%s"}', ...
    strrep(model_file, '\', '/'), model_version, file_sha256(model_file), route, ...
    char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
lines{end + 1} = '  ports:';
if ~isempty(in_lines)
    lines = [lines, {'    in:'}, in_lines];
end
lines = [lines, {'    out:'}, out_lines];
components_file = fullfile(project, 'plugins', 'components.yaml');
write_component(components_file, name, lines);
ensure_project_components(project);

fprintf('vsim_export: %s registered in %s\n', name, components_file);
if ~isempty(unmapped)
    fprintf(2, 'vsim_export: connect these inputs in components.yaml before running: %s\n', strjoin(unmapped, ', '));
end
result = struct('name', name, 'route', route, 'path', plugin_path, 'period', step, ...
    'components', components_file, 'unmapped', {unmapped});
end

% ---------------------------------------------------------------------------------------------

function ports = root_ports(mname)
% Root Inports / Outports in port order: block name, unit.
ports = struct('in', struct('name', {}, 'unit', {}, 'field', {}, 'type', {}), ...
    'out', struct('name', {}, 'unit', {}, 'field', {}, 'type', {}));
kinds = {'in', 'Inport'; 'out', 'Outport'};
for k = 1:2
    blocks = find_system(mname, 'SearchDepth', 1, 'BlockType', kinds{k, 2});
    numbers = cellfun(@(b) str2double(get_param(b, 'Port')), blocks);
    [~, order] = sort(numbers);
    for b = blocks(order)'
        unit = get_param(b{1}, 'Unit');
        if any(strcmp(unit, {'inherit', ''}))
            unit = '';
        end
        ports.(kinds{k, 1})(end + 1) = struct('name', get_param(b{1}, 'Name'), 'unit', unit, 'field', '', 'type', '');
    end
end
end

function [plugin_path, ports] = export_native(mname, name, step, ports, out_dir, o, here)
gen_root = tempname;
mkdir(gen_root);
old_cfg = Simulink.fileGenControl('getConfig');
Simulink.fileGenControl('set', 'CodeGenFolder', gen_root, 'CacheFolder', gen_root, 'createDir', true);
restore_cfg = onCleanup(@() Simulink.fileGenControl('setConfig', 'config', old_cfg));

% Code generation settings on a copy of the active configuration (the model keeps its own).
cs = getActiveConfigSet(mname);
tmp = copy(cs);
tmp.Name = 'vsim_export';
attachConfigSet(mname, tmp, true);
setActiveConfigSet(mname, tmp.Name);
restore_cs = onCleanup(@() restore_config(mname, cs.Name, tmp.Name));
set_param(tmp, 'SystemTargetFile', 'ert.tlc');
set_param(tmp, 'TargetLang', 'C');
set_param(tmp, 'GenCodeOnly', 'on');
set_param(tmp, 'GenerateSampleERTMain', 'off');
set_param(tmp, 'CodeInterfacePackaging', 'Nonreusable function');
set_param(tmp, 'CombineOutputUpdateFcns', 'on');
set_param(tmp, 'MatFileLogging', 'off');
set_param(tmp, 'GenerateComments', 'on');
set_param(tmp, 'SimulinkBlockComments', 'on');
set_param(tmp, 'GenerateReport', 'off');
slbuild(mname);

gen = fullfile(gen_root, [mname '_ert_rtw']);
shared = fullfile(gen_root, 'slprj', 'ert', '_sharedutils');
src = fullfile(out_dir, 'src');
if isfolder(src)
    rmdir(src, 's');
end
mkdir(src);
for folder = {gen, shared}
    if isfolder(folder{1})
        files = [dir(fullfile(folder{1}, '*.c')); dir(fullfile(folder{1}, '*.h'))];
        for f = files'
            if ~strcmp(f.name, 'ert_main.c')
                copyfile(fullfile(f.folder, f.name), src);
            end
        end
    end
end

% Field names and C types of the root ports from the generated header:
%   real_T APPS;  /* '<Root>/APPS' */
header = fileread(fullfile(src, [mname '.h']));
ports.in = fill_fields(ports.in, header, ['ExtU_' mname '_T'], mname);
ports.out = fill_fields(ports.out, header, ['ExtY_' mname '_T'], mname);

% The wrapper (the same table as sdk/simulink/vsim_ert_wrapper.cmake writes).
text = fileread(fullfile(here, 'vsim_ert_wrapper.c.in'));
text = strrep(text, '@INPUTS@', port_table(ports.in, 'VSIM_IN', ['ExtU_' mname '_T']));
text = strrep(text, '@OUTPUTS@', port_table(ports.out, 'VSIM_OUT', ['ExtY_' mname '_T']));
text = strrep(text, '@N_IN@', sprintf('%d', numel(ports.in)));
text = strrep(text, '@N_OUT@', sprintf('%d', numel(ports.out)));
text = strrep(text, '@MODEL@', mname);
text = strrep(text, '@STEP@', sprintf('%.17g', step));
text = strrep(text, '@VERSION@', get_param(mname, 'ModelVersion'));
write_text(fullfile(src, [name '_vsim.c']), text);
copyfile(fullfile(o.VsimRoot, 'sdk', 'c', 'include', 'vsim_plugin.h'), src);

ml = strrep(matlabroot, '\', '/');
cmake = {
    'cmake_minimum_required(VERSION 3.20)'
    sprintf('project(%s C)', name)
    '# Generated by vsim_export: the Embedded Coder code of the model and its vsim wrapper.'
    'file(GLOB sources "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")'
    sprintf('add_library(%s MODULE ${sources})', name)
    sprintf('target_include_directories(%s PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"', name)
    sprintf('  "%s/simulink/include" "%s/rtw/c/src" "%s/extern/include")', ml, ml, ml)
    sprintf('set_target_properties(%s PROPERTIES PREFIX ""', name)
    '  LIBRARY_OUTPUT_DIRECTORY "$<1:${CMAKE_CURRENT_SOURCE_DIR}>"'
    '  RUNTIME_OUTPUT_DIRECTORY "$<1:${CMAKE_CURRENT_SOURCE_DIR}>")'
    sprintf('if(MSVC)\n  target_compile_definitions(%s PRIVATE _CRT_SECURE_NO_WARNINGS)\nendif()', name)
    sprintf('if(UNIX)\n  target_link_libraries(%s PRIVATE m)\nendif()', name)
    };
write_text(fullfile(out_dir, 'CMakeLists.txt'), strjoin(cmake, newline));

plugin_path = sprintf('plugins/%s/%s', name, name);
if o.Build
    build = fullfile(out_dir, 'build');
    [status, log] = system(sprintf('cmake -S "%s" -B "%s" && cmake --build "%s" --config Release', ...
        out_dir, build, build));
    if status ~= 0
        warning('vsim_export:build', ['Building the plugin failed (CMake and a C compiler are needed: ' ...
            'Visual Studio on Windows). Build it later with\n  cmake -S "%s" -B "%s"\n  ' ...
            'cmake --build "%s" --config Release\n\n%s'], out_dir, build, build, log);
    else
        fprintf('vsim_export: built %s\n', fullfile(out_dir, name));
    end
end
end

function restore_config(mname, original, temporary)
setActiveConfigSet(mname, original);
detachConfigSet(mname, temporary);
end

function ports = fill_fields(ports, header, struct_name, mname)
if isempty(ports)
    return
end
body = regexp(header, ['typedef struct\s*\{(.*?)\}\s*' struct_name ';'], 'tokens', 'once');
if isempty(body)
    error('vsim_export:codegen', 'Cannot find %s in %s.h (unexpected code generation settings).', struct_name, mname);
end
fields = regexp(body{1}, '(\w+)\s+(\w+)\s*(\[\d+\])?\s*;\s*/\*\s*''<Root>/([^'']+)''', 'tokens');
types = containers.Map({'real_T', 'real32_T', 'int8_T', 'uint8_T', 'int16_T', 'uint16_T', 'int32_T', ...
    'uint32_T', 'boolean_T', 'double', 'float'}, {'DOUBLE', 'SINGLE', 'INT8', 'UINT8', 'INT16', 'UINT16', ...
    'INT32', 'UINT32', 'BOOLEAN', 'DOUBLE', 'SINGLE'});
for k = 1:numel(ports)
    hit = [];
    for f = 1:numel(fields)
        if strcmp(fields{f}{4}, ports(k).name)
            hit = fields{f};
        end
    end
    if isempty(hit)
        error('vsim_export:codegen', 'Root port "%s" not found in %s (is it a bus or does it have no name?).', ...
            ports(k).name, struct_name);
    end
    if ~isempty(hit{3})
        error('vsim_export:ports', 'Root port "%s" is a vector %s; vsim ports are scalars (split it).', ...
            ports(k).name, hit{3});
    end
    if ~isKey(types, hit{1})
        error('vsim_export:ports', 'Root port "%s" has type %s; use double, single, (u)int8-32 or boolean.', ...
            ports(k).name, hit{1});
    end
    ports(k).field = hit{2};
    ports(k).type = types(hit{1});
end
end

function text = port_table(ports, direction, struct_name)
text = '';
for k = 1:numel(ports)
    text = [text, sprintf('    {{"%s", "%s", %s}, offsetof(%s, %s), VSIM_T_%s},\n', ports(k).name, ...
        ports(k).unit, direction, struct_name, ports(k).field, ports(k).type)]; %#ok<AGROW>
end
end

function plugin_path = export_fmu(mname, name, out_dir)
before = pwd;
back = onCleanup(@() cd(before));
cd(out_dir);
if exist('exportToFMU', 'file') == 2
    exportToFMU(mname, 'FMIVersion', '2', 'FMUType', 'CS');  % R2023b and later
else
    exportToFMU2CS(mname);
end
fmu = dir(fullfile(out_dir, [mname '.fmu']));
if isempty(fmu)
    error('vsim_export:fmu', 'The FMU export did not write %s.fmu.', mname);
end
plugin_path = sprintf('plugins/%s/%s.fmu', name, mname);
end

function [lines, unmapped] = map_ports(ports, signals, input)
lines = {};
unmapped = {};
names = {signals.name};
units = {signals.unit};
for k = 1:numel(ports)
    port = ports(k).name;
    unit_note = '';
    if ~isempty(ports(k).unit)
        unit_note = sprintf('   # port unit %s', ports(k).unit);
    end
    if contains(port, '.')
        warning('vsim_export:portname', 'Port "%s": dots in port names cannot be mapped in YAML; rename it.', port);
    end
    match = find(strcmp(names, port) | strcmp(strrep(names, '.', '_'), port), 1);
    if ~isempty(match)
        lines{end + 1} = sprintf('      %s: %s%s', port, names{match}, unit_note); %#ok<AGROW>
        if ~isempty(ports(k).unit) && ~isempty(units{match}) && ~strcmp(units{match}, ports(k).unit)
            lines{end} = sprintf('%s (signal %s, converted)', lines{end}, units{match});
        end
        continue
    end
    hints = suggestions(port, names);
    if input
        lines{end + 1} = sprintf('      %s: ""   # TODO: which signal?%s%s', port, hints, unit_note); %#ok<AGROW>
        unmapped{end + 1} = port; %#ok<AGROW>
    else
        lines{end + 1} = sprintf('      # %s: (written to plugin.<name>.%s; give a signal to drive one)%s', ...
            port, port, hints); %#ok<AGROW>
    end
end
if isempty(lines) && ~input
    lines = {'      {}'};
end
end

function text = suggestions(port, names)
text = '';
if isempty(names)
    return
end
key = lower(regexprep(port, '[^A-Za-z0-9]', ''));
score = zeros(1, numel(names));
for k = 1:numel(names)
    other = lower(regexprep(names{k}, '[^A-Za-z0-9]', ''));
    score(k) = 1 - edit_distance(key, other) / max([numel(key), numel(other), 1]);
end
[s, order] = sort(score, 'descend');
best = names(order(s > 0.5));
if ~isempty(best)
    text = [' candidates: ', strjoin(best(1:min(3, end)), ', ')];
end
end

function d = edit_distance(a, b)
row = 0:numel(b);
for i = 1:numel(a)
    prev = row;
    row(1) = i;
    for j = 1:numel(b)
        row(j + 1) = min([prev(j + 1) + 1, row(j) + 1, prev(j) + (a(i) ~= b(j))]);
    end
end
d = row(end);
end

function write_component(file, name, lines)
% Replaces the top-level `name:` block of the components file, or appends it.
old = {};
if isfile(file)
    old = splitlines(fileread(file))';
end
start = find(strcmp(old, [name ':']) | startsWith(old, [name ': ']), 1);
if isempty(start)
    while ~isempty(old) && isempty(strtrim(old{end}))
        old(end) = [];
    end
    out = [old, {''}, lines];
else
    stop = numel(old) + 1;
    for k = start + 1:numel(old)
        if ~isempty(regexp(old{k}, '^[A-Za-z_]\w*:', 'once'))
            stop = k;
            break
        end
    end
    out = [old(1:start - 1), lines, {''}, old(stop:end)];
end
write_text(file, [strjoin(out, newline), newline]);
end

function ensure_project_components(project)
file = fullfile(project, 'project.yaml');
text = fileread(file);
if isempty(regexp(text, '(?m)^components:', 'once'))
    if text(end) ~= newline
        text = [text, newline];
    end
    write_text(file, [text, 'components: plugins/components.yaml', newline]);
end
end

function write_text(file, text)
fid = fopen(file, 'w', 'n', 'UTF-8');
if fid < 0
    error('vsim_export:write', 'Cannot write %s.', file);
end
fwrite(fid, text, 'char');
fclose(fid);
end

function h = file_sha256(file)
fid = fopen(file, 'r');
data = fread(fid, inf, '*uint8');
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
md.update(data);
digest = typecast(md.digest(), 'uint8');
h = lower(reshape(dec2hex(digest, 2)', 1, []));
end
