function [data_train, data_test, t0, x0eq] = collectDataAfterStab( ...
    odeHandle, x0, u0, Ts, maxSimTime, tol, trainTime, testTime, uTrainFun, uTestFun, noise_std)
  
    function [value, isterminal, direction] = settleEvent(t, x)
    dx = odeHandle(t, x, u0);
    value = norm(dx) - tol;
    isterminal= 1;
    direction = -1;
  end

  opts = odeset('Events', @settleEvent, 'RelTol', 1e-8, 'AbsTol', 1e-10);

  [~, ~, TE, XE, ~] = ode45(@(t, x) odeHandle(t, x, u0), [0, maxSimTime], x0, opts);

  if isempty(TE)
    error('System did not settle within %g seconds (tol=%g).', maxSimTime, tol);
  end

  t0   = TE(1);
  x0eq = XE(1, :)';

  t_train = (0:Ts:trainTime)' + t0;
  
  u_dev_train = uTrainFun(t_train);
   
  [~, x_train] = ode45(@(t,x) odeHandle(t, x, u0 + interp1(t_train, u_dev_train, t)), t_train, x0eq);
  
  y_train     = x_train(:,end);
  y_dev_train = y_train - x0eq(end);
  train_noise_std = noise_std * max(y_dev_train);

  data_train = iddata(y_dev_train + train_noise_std * randn(size(y_dev_train)), u_dev_train, Ts);

  t_test = (0: Ts: testTime)' + t0 + trainTime;
  u_dev_test = uTestFun(t_test);

  [~, x_test] = ode45(@(t,x) odeHandle(t, x, u0 + interp1(t_test, u_dev_test, t)), t_test, x_train(end, :)');
  
  y_test = x_test(:, end);
  y_dev_test = y_test - x0eq(end);
  test_noise_std = noise_std * max(y_dev_test);

  data_test = iddata(y_dev_test + test_noise_std * randn(size(y_dev_test)), u_dev_test, Ts);
end