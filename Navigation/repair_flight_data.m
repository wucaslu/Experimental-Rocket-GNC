function report = repair_flight_data(sourceFile, targetFile, diagnosticDirectory)
%REPAIR_FLIGHT_DATA Repair only the main-parachute part of the flight export.
%   report = repair_flight_data() writes flight_data_corrected.csv beside the
%   original. It repairs ENU velocity and acceleration only within 154-163 s
%   of source flight time. The first parachute event and all other fields,
%   including attitude and body angular rates, remain unchanged.
%
%   The source remains unchanged. This is a kinematic repair of the existing
%   trajectory around the second parachute; it retains the source attitude
%   and angular-rate assumptions throughout the flight.
%
%   Optional diagnosticDirectory saves a JSON report and comparison figure.
%   The event window is CSV flight time, before the 50 s startup pad.

    fileDirectory = fileparts(mfilename('fullpath'));
    if nargin < 1 || isempty(sourceFile)
        sourceFile = fullfile(fileDirectory, 'flight_data.csv');
    end
    if nargin < 2 || isempty(targetFile)
        targetFile = fullfile(fileDirectory, 'flight_data_corrected.csv');
    end
    if nargin < 3, diagnosticDirectory = ''; end
    sourceFile = char(java.io.File(char(sourceFile)).getCanonicalPath());
    targetFile = char(java.io.File(char(targetFile)).getCanonicalPath());
    assert(~strcmpi(sourceFile,targetFile), ...
        'FlightRepair:Original', 'The output must differ from the original.');

    originalText = fileread(sourceFile);
    lines = splitlines(string(originalText));
    lines(strlength(lines)==0) = [];
    header = lines(1);
    tokens = split(lines(2:end), ',');
    sourceTokens = tokens;
    original = str2double(tokens);
    assert(size(original,2)==20 && all(isfinite(original(:))), ...
        'FlightRepair:Schema', 'Expected the finite, 20-column flight export.');
    time = original(:,1);
    step = median(diff(time));
    assert(numel(time)>=5 && all(diff(time)>0) && ...
        max(abs(diff(time)-step))<1e-9, ...
        'FlightRepair:Time', 'Expected a strictly increasing uniform time grid.');

    % Cover the main-parachute export artifacts, including their precursor.
    % Preserve the first parachute and every sample outside this window.
    linearWindows = [154.00,163.00];
    assert(time(1)<=linearWindows(1,1) && time(end)>=linearWindows(end,2), ...
        'FlightRepair:Coverage', 'The configured event windows exceed the data.');
    linearMask = false(size(time));
    for index=1:size(linearWindows,1)
        linearMask = linearMask | ...
            (time>=linearWindows(index,1) & time<=linearWindows(index,2));
    end

    [velocityFromPosition,accelerationFromPosition] = ...
        derivativesFivePoint(original(:,2:4),step);

    corrected = original;
    corrected(linearMask,5:7) = velocityFromPosition(linearMask,:);
    corrected(linearMask,8:10) = accelerationFromPosition(linearMask,:);

    % Keep original header and all unedited fields as their original strings.
    % Extra decimal places in repaired channels avoid rounding derivatives.
    tokens(linearMask,5:10) = compose('%.9f',corrected(linearMask,5:10));
    lineEnding = sprintf('\r\n');
    outputText = char(join([header;join(tokens,',',2)],lineEnding,1));
    fid = fopen(targetFile,'w');
    assert(fid>=0,'FlightRepair:Output','Cannot write the corrected CSV.');
    fileCleanup = onCleanup(@()fclose(fid));
    fwrite(fid,unicode2native([outputText,lineEnding],'UTF-8'),'uint8');
    clear fileCleanup

    output = readmatrix(targetFile,'NumHeaderLines',1);
    protectedColumns = [1:4,11:20];
    assert(isequal(output(:,protectedColumns),original(:,protectedColumns)), ...
        'FlightRepair:Protected', 'A protected trajectory field changed.');
    assert(isequal(output(~linearMask,:),original(~linearMask,:)), ...
        'FlightRepair:Scope', 'Data changed outside the main-parachute window.');
    outputLines = splitlines(string(fileread(targetFile)));
    outputLines(strlength(outputLines)==0) = [];
    outputTokens = split(outputLines(2:end),',');
    assert(isequal(outputLines(1),header) && ...
        isequal(outputTokens(:,protectedColumns),sourceTokens(:,protectedColumns)) && ...
        isequal(outputTokens(~linearMask,:),sourceTokens(~linearMask,:)), ...
        'FlightRepair:TextScope', 'An unedited CSV field changed textually.');
    assert(all(isfinite(output(:))) && isequal(size(output),size(original)), ...
        'FlightRepair:Output', 'Output contains invalid or missing values.');
    assert(strcmp(originalText,fileread(sourceFile)), ...
        'FlightRepair:Original', 'The source CSV changed during repair.');

    report = struct('sourceFile',sourceFile,'correctedFile',targetFile, ...
        'sampleCount',numel(time),'samplePeriod_s',step, ...
        'endTime_s',time(end),'linearRepairWindows_s',linearWindows, ...
        'linearRowsProcessed',sum(linearMask), ...
        'changedColumns',5:10, ...
        'firstParachutePreserved',true,'angularRatesPreserved',true, ...
        'originalPreserved',true,'protectedColumnsPreserved',true, ...
        'units','Original ENU metres, m/s, m/s^2; body rates rad/s.', ...
        'attitudeAssumption','Source attitude and body rates unchanged.');
    for index=1:size(linearWindows,1)
        mask = time>=linearWindows(index,1) & time<=linearWindows(index,2);
        originalCheck = integralCheck(time(mask),original(mask,:));
        correctedCheck = integralCheck(time(mask),output(mask,:));
        % Independent trapezoid integration checks the complete event, not
        % only agreement with the differentiation stencil used for repair.
        assert(correctedCheck.maxPositionResidual_m<0.02 && ...
            correctedCheck.maxVelocityResidual_m_s<0.15, ...
            'FlightRepair:Kinematics','Event integration residual is excessive.');
        report.event(index) = struct('window_s',linearWindows(index,:), ...
            'original',originalCheck,'corrected',correctedCheck);
    end
    mainMask = linearMask;
    mainIndices = find(mainMask);
    [minimum,index] = min(original(mainMask,10));
    report.mainOriginalMinimumAz_m_s2 = minimum;
    report.mainOriginalMinimumTime_s = time(mainIndices(index));
    [maximum,index] = max(output(mainMask,10));
    report.mainCorrectedPeakAz_m_s2 = maximum;
    report.mainCorrectedPeakTime_s = time(mainIndices(index));

    if ~isempty(diagnosticDirectory)
        if ~isfolder(diagnosticDirectory), mkdir(diagnosticDirectory); end
        fid = fopen(fullfile(diagnosticDirectory,'repair-report.json'),'w');
        assert(fid>=0,'FlightRepair:Report','Cannot write diagnostics.');
        reportCleanup = onCleanup(@()fclose(fid));
        fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
        clear reportCleanup
        save(fullfile(diagnosticDirectory,'repair-evidence.mat'), ...
            'report','original','output','velocityFromPosition', ...
            'accelerationFromPosition');
        comparisonFigure(time,original,output,diagnosticDirectory);
    end
end

function [first,second] = derivativesFivePoint(values,step)
% Degree-four local polynomial derivatives; no global smoothing or clipping.
    first = zeros(size(values));
    second = zeros(size(values));
    first(3:end-2,:) = (values(1:end-4,:)-8*values(2:end-3,:) + ...
        8*values(4:end-1,:)-values(5:end,:))/(12*step);
    second(3:end-2,:) = (-values(1:end-4,:)+16*values(2:end-3,:) - ...
        30*values(3:end-2,:)+16*values(4:end-1,:) - ...
        values(5:end,:))/(12*step^2);
    w1 = [-25,48,-36,16,-3;-3,-10,18,-6,1];
    w2 = [35,-104,114,-56,11;11,-20,6,4,-1];
    first(1:2,:) = w1*values(1:5,:)/(12*step);
    second(1:2,:) = w2*values(1:5,:)/(12*step^2);
    first(end:-1:end-1,:) = -w1*values(end:-1:end-4,:)/(12*step);
    second(end:-1:end-1,:) = w2*values(end:-1:end-4,:)/(12*step^2);
end

function result = integralCheck(time,data)
    positionResidual = data(1,2:4)+cumtrapz(time,data(:,5:7))-data(:,2:4);
    velocityResidual = data(1,5:7)+cumtrapz(time,data(:,8:10))-data(:,5:7);
    result = struct('maxPositionResidual_m', ...
        max(vecnorm(positionResidual,2,2)), ...
        'maxVelocityResidual_m_s',max(vecnorm(velocityResidual,2,2)), ...
        'finalPositionResidual_m',positionResidual(end,:), ...
        'finalVelocityResidual_m_s',velocityResidual(end,:));
end

function comparisonFigure(time,original,corrected,directory)
    figureHandle = figure('Visible','off','Color','w','Position',[50,50,1200,850]);
    figureCleanup = onCleanup(@()close(figureHandle));
    layout = tiledlayout(figureHandle,2,2,'Padding','compact','TileSpacing','compact');
    a = nexttile(layout);
    plot(a,time,[original(:,7),corrected(:,7)],'LineWidth',1.1);
    xlim(a,[154,163]); grid(a,'on'); ylabel(a,'Up velocity [m/s]');
    title(a,'Main deployment: velocity'); legend(a,{'Original','Corrected'},'Location','best');
    a = nexttile(layout);
    plot(a,time,[original(:,10),corrected(:,10)],'LineWidth',1.1);
    xlim(a,[154,163]); grid(a,'on'); ylabel(a,'Up acceleration [m/s^2]');
    title(a,'Exported acceleration artifact'); legend(a,{'Original','Corrected'},'Location','best');
    a = nexttile(layout);
    plot(a,time,corrected(:,10),'LineWidth',1.2);
    xlim(a,[161.3,162]); grid(a,'on'); ylabel(a,'Up acceleration [m/s^2]');
    xlabel(a,'CSV flight time [s]'); title(a,'Corrected braking impulse retained');
    a = nexttile(layout);
    plot(a,time,[original(:,10),corrected(:,10)],'LineWidth',1.1);
    xlim(a,[24,28]); grid(a,'on');
    ylabel(a,'Up acceleration [m/s^2]');
    xlabel(a,'CSV flight time [s]'); title(a,'First parachute event preserved');
    legend(a,{'Original','Corrected'},'Location','best');
    exportgraphics(figureHandle,fullfile(directory,'flight-data-repair.png'),'Resolution',150);
    clear figureCleanup
end
