function x0 = GetHandleX0(handle, tRange, initialValues)
    % Use only for systems that stabilize over time
    [~, S_sol] = ode45(handle, tRange, initialValues);
    x0 = S_sol(end, :)';
end