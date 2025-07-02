function plant_sfun(block)
    setup(block);
end


function setup(block)
    block.NumDialogPrms = 4;
    validateParameters(block.DialogPrm(1).Data, block.DialogPrm(2).Data, block.DialogPrm(3).Data);

    odeFcnHandle = block.DialogPrm(1).Data;
    x0_vec       = block.DialogPrm(2).Data(:);
    u0_vec       = block.DialogPrm(3).Data(:);
    idxListParam = block.DialogPrm(4).Data;

    x0_vec = x0_vec(:);
    u0_vec = u0_vec(:);

    if ~isa(odeFcnHandle, 'function_handle')
        error('Parameter 1 must be a function handle.');
    end
    if ~isvector(x0_vec)
        error('Parameter 2 (x0) must be a vector.');
    end
    if ~isvector(u0_vec)
        error('Parameter 3 (u0) must be a vector.');
    end

    nStates = numel(x0_vec);
    nInputs = numel(u0_vec);

    try
        xdot0 = odeFcnHandle(0, x0_vec, u0_vec);
        xdot0 = xdot0(:);
        
        if numel(xdot0) ~= nStates
            error('ODE handle must return an %d×1 derivative vector; got %d×1.', ...
                  nStates, numel(xdot0));
        end
    catch ME
        error('Error calling ODE handle during setup:\n  %s', ME.message);
    end

    if isempty(idxListParam)
        idxList = 1:nStates;
    else
        idxList = idxListParam(:).';
        
        if any(idxList < 1) || any(idxList > nStates) || any(floor(idxList) ~= idxList)
            error('Parameter 4 (idxList) must be integers in the range 1..%d (or empty).', nStates);
        end
    end

    nOutputs = numel(idxList);

    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    block.InputPort(1).Dimensions = nInputs;
    block.InputPort(1).DirectFeedthrough = true;

    block.OutputPort(1).Dimensions = nOutputs;

    block.NumContStates = nStates;

    block.SampleTimes = [0 0];

    block.SimStateCompliance = 'DefaultSimState';
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Derivatives', @Derivatives);
end

function InitializeConditions(block)
    x0_vec = block.DialogPrm(2).Data(:);

    for i = 1:block.NumContStates
        block.ContStates.Data(i) = x0_vec(i);
    end
end


function Outputs(block)
    x_full = block.ContStates.Data(:);

    idxListParam = block.DialogPrm(4).Data;
    nStates      = block.NumContStates;

    if isempty(idxListParam)
        idxList = 1:nStates;
    else
        idxList = idxListParam(:).';
        
        idxList = idxList(idxList >= 1 & idxList <= nStates);
        
        if isempty(idxList)
            idxList = 1:nStates;
        end
    end

    block.OutputPort(1).Data = x_full(idxList);
end

function Derivatives(block)
    f = block.DialogPrm(1).Data;
    t = block.CurrentTime;
    x = block.ContStates.Data(:);
    u = block.InputPort(1).Data(:);

    xdot = f(t, x, u);
    block.Derivatives.Data = xdot(:);
end

function validateParameters(odeFcnHandle, x0, u0)
    if ~isa(odeFcnHandle, 'function_handle')
        error('Parameter 1 must be a function handle.');
    end
    
    if ~isvector(x0)
        error('Parameter 2 (x0) must be a vector.');
    end
    
    if ~isvector(u0)
        error('Parameter 3 (u0) must be a vector.');
    end
end
