function [] = assemble_CBFs(u_max, tv, ic, folder)
    
    % Create gen folder if it doesnt exist
    if ~isfolder(folder)
        mkdir(folder)
    end
    
    % Define general symbolic variables
    m = sym('m', [1 1], 'real');
    J = sym('J', [1 1], 'real');
    
    x = sym('x', [6 1], 'real');
    xt = sym('xt', [6 1], 'real');
    u = sym('u', [3 1], 'real');      
    
    Rt = [cos(xt(3)) -sin(xt(3)); 
         sin(xt(3))  cos(xt(3))];
    
    Rc = [cos(x(3)) -sin(x(3)); 
          sin(x(3))  cos(x(3))];
    
    
    % Define dynamics functions
    f = [x(4:6); 0;0;0];
    g = [         zeros(3,3);
         diag([1/m; 1/m; 1/J]) ];
    
    ft = [xt(4:6); 0;0;0];
    
    % Elliptical Keep-out-Zone 
    r_KOZ = sym('r_KOZ', [2 1], 'real'); 
    
    S = diag(1./r_KOZ.^2);
    rK = x(1:2) - xt(1:2);
    
    h_KOZ = rK'*Rt*S*Rt'*rK - 1;
    
    [A_KOZ, b_KOZ, a_KOZ] = build_cbf(h_KOZ, f, g, x, u, ...
        'TimeVarying', tv, 'TimeVaryingVars', xt, 'TimeVaryingFcn', ft, ...
        'InputConstrained', ic, 'InputLimits', u_max);
    
    % Line of Sight
    e_cam  = sym('e_cam', [2 1],'real');    % boresight (body)
    r_cam  = sym('r_cam', [2 1], 'real');    % camera offset
    r_look = sym('r_look', [2 1], 'real');   % target feature pt
    th_FOV = sym('th_FOV', [1 1], 'real');   % sensor FOV
    
    rL = (xt(1:2) + Rt*r_look) - (x(1:2) + Rc*r_cam);
    eL = Rc*e_cam;
    
    h_LOS = (rL'*eL)^2 - (rL.'*rL)*(cos(th_FOV)^2);       

    [A_LOS, b_LOS, a_LOS] = build_cbf(h_LOS, f([3,6]), g([3,6],3), x([3,6]), u(3), ...
        'TimeVarying', tv, 'TimeVaryingVars', [x([1:2,4:5]); xt], 'TimeVaryingFcn', [f([1:2,4:5]); ft], ...
        'InputConstrained', ic, 'InputLimits', u_max(3));
    A_LOS = [0, 0, A_LOS];
    
    % 
    % [A_LOS, b_LOS, a_LOS] = build_cbf(h_LOS, f, g, x, u, ...
    %     'TimeVarying', tv, 'TimeVaryingVars', xt, 'TimeVaryingFcn', ft, ...
    %     'InputConstrained', ic, 'InputLimits', u_max);

    % Save CBFs
    matlabFunction(A_KOZ , b_KOZ, h_KOZ, 'File', strcat(folder, 'cbf_KOZ.m'), ...
        'Vars',{x, xt, m, J, r_KOZ, a_KOZ}, ...
        'Outputs',{'A','b', 'h'}, 'Optimize', true);

    matlabFunction(A_LOS , b_LOS, h_LOS, 'File', strcat(folder, 'cbf_LOS.m'), ...
        'Vars',{x, xt, m, J, e_cam, r_cam, r_look, th_FOV, a_LOS}, ...
        'Outputs',{'A', 'b', 'h'}, 'Optimize', true);

    % Clear temporary symbolic variables
    clear x xt u m J Rt Rc f g ft ...
        r_KOZ S rK h_KOZ A_KOZ b_KOZ a_KOZ ...
        e_cam r_cam r_look th_FOV rL eL h_LOS A_LOS b_LOS a_LOS ...
        ans
end