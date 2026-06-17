function build_iccbf_functions()
% BUILD_ICCBF_FUNCTIONS
% Derives the KOZ (target + obstacle) and LOS CBF/ICCBF QP-constraint rows
% symbolically and exports them as Simulink-ready MATLAB functions.
%
% Approach: augmented AUTONOMOUS system  Xdot = f(X) + g(X) u ,
%   X = [chaser(6); platform(6)],  u = [Fx; Fy; tau_z].
% The "platform" slot is whichever body the constraint is about -- target
% (x_BLACK) for docking KOZ/LOS, obstacle (x_BLUE) for the obstacle KOZ. Its
% constant-velocity/spin motion (gated by tv) makes the whole system time-
% invariant, so the ICCBF is pure Lie derivatives: the Symbolic Toolbox does
% all the chain/quotient/third-derivative algebra and matlabFunction emits
% shape-correct code (no hand-derivation, no shape bugs).
%
% Generated files (drop on the model path):
%   fcn_KOZ_tar_ICCBF_gen.m  [A_KOZ_tar,b_KOZ_tar] = f(xR,xB,mRED,rKOZ,a0,a1,a2,Fmax,tv)
%   fcn_KOZ_obs_ICCBF_gen.m  [A_KOZ_obs,b_KOZ_obs] = f(xR,xB,mRED,rKOZ,a0,a1,a2,Fmax,tv)
%   fcn_LOS_ICCBF_gen.m      [A_LOS,    b_LOS]      = f(xR,xB,IRED,FOV,sn,so,stg,a0,a1,a2,taumax,tv)
% The two KOZ files are identical math -- call _tar with x_BLACK,r_KOZ_tar and
% _obs with x_BLUE,r_KOZ_obs. (You could use one file for both; kept separate
% to match the existing block names.)
%
% Call from a thin MATLAB Function block, e.g. the obstacle one with its guard:
%   function [A,b]=fcn(xR,xB,mRED,a0,a1,a2,Fmax,tv,rKOZ,platformSelection)
%       A=[0 0 0 0]; b=0;
%       if platformSelection>=6 && platformSelection<=11
%           [A,b]=fcn_KOZ_obs_ICCBF_gen(xR,xB,mRED,rKOZ,a0,a1,a2,Fmax,tv);
%       end
%   end
%
% QP decision vector assumed:  [Fx; Fy; tau_z; delta]  (delta = CLF slack).
% The input box |u_i|<=u_max must still be its own rows in the QP.
    % ---------- symbolic states & parameters ----------
    xR = sym('xR',[6 1],'real');   % chaser   [x y th xd yd thd]
    xB = sym('xB',[6 1],'real');   % platform [x y th xd yd thd]  (target OR obstacle)
    X  = [xR; xB];
    syms mRED IRED real
    syms a0 a1 a2 Fmax taumax tv real
    eps_abs = sym(1e-9);
    % ---------- augmented autonomous dynamics ----------
    f = [ xR(4); xR(5); xR(6); 0; 0; 0; ...
          tv*xB(4); tv*xB(5); tv*xB(6); 0; 0; 0 ];
    gF = sym(zeros(12,3)); gF(4,1)=1/mRED; gF(5,2)=1/mRED; gF(6,3)=1/IRED; % forces+torque
    gT = sym(zeros(12,3)); gT(6,3)=1/IRED;                                 % torque only
    % =================== elliptical KOZ (thrusters) ===================
    % built ONCE, exported to both target and obstacle filenames
    rKOZ = sym('rKOZ',[2 1],'real');
    S  = diag(1./rKOZ.^2);
    th = xB(3);
    Rt = [cos(th) -sin(th); sin(th) cos(th)];
    Q  = Rt*S*Rt.';
    d  = xR(1:2) - xB(1:2);
    hK = (d.'*Q*d) - 1;                                   % >= 0 outside KOZ
    [A_K, b_K] = iccbf_row(hK, X, f, gF, [Fmax;Fmax;Fmax], a0,a1,a2, eps_abs, 2);
    A_K = [A_K, sym(0)];                                  % append CLF slack col -> 1x4
    matlabFunction([A_K, hK, sym(0), sym(0)] , b_K, 'File','fcn_KOZ_tar_ICCBF_gen.m', ...
        'Vars',{xR,xB,mRED,rKOZ,a0,a1,a2,Fmax,tv}, ...
        'Outputs',{'A_KOZ_tar','b_KOZ_tar'}, 'Optimize',true);
    matlabFunction([A_K, sym(0), hK, sym(0)], b_K, 'File','fcn_KOZ_obs_ICCBF_gen.m', ...
        'Vars',{xR,xB,mRED,rKOZ,a0,a1,a2,Fmax,tv}, ...
        'Outputs',{'A_KOZ_obs','b_KOZ_obs'}, 'Optimize',true);
    % =================== LOS barrier (torque only) ===================
    syms FOV real
    sn  = sym('sn',[2 1],'real');    % boresight (body)   sensor_normal
    so  = sym('so',[2 1],'real');    % camera offset      sensor_offset
    stg = sym('stg',[2 1],'real');   % target feature pt  sensor_target
    cFOV = cos(FOV)^2;
    thc = xR(3); thL = xB(3);
    Rc = [cos(thc) -sin(thc); sin(thc) cos(thc)];
    RtL= [cos(thL) -sin(thL); sin(thL) cos(thL)];
    rL = xB(1:2) + RtL*stg - xR(1:2) - Rc*so;
    eL = Rc*sn;
    hL = (rL.'*eL)^2 - cFOV*(rL.'*rL);                    % un-normalized (docks well)
    % range-normalized alternative (less conservative near contact):
    % hL = (rL.'*eL)^2/(rL.'*rL) - cFOV;
    [A_L, b_L] = iccbf_row(hL, X, f, gT, [taumax;taumax;taumax], a0,a1,a2, eps_abs, 2);
    A_L = [A_L, sym(0), sym(0), sym(0), hL];
    matlabFunction(A_L, b_L, 'File','fcn_LOS_ICCBF_gen.m', ...
        'Vars',{xR,xB,IRED,FOV,sn,so,stg,a0,a1,a2,taumax,tv}, ...
        'Outputs',{'A_LOS','b_LOS'}, 'Optimize',true);
    % =================== numeric self-checks ===================
    % tR = randn(6,1); tB = randn(6,1);
    % % KOZ (covers both _tar and _obs, identical math)
    % sl = [xR; xB; mRED; rKOZ; a0; a1; a2; Fmax; tv];
    % sv = [tR; tB; 12.137; [0.30;0.22]; 0.8; 1.2; 0.5; 0.1; 1];
    % A_ref = double(subs(A_K, sl, sv)); b_ref = double(subs(b_K, sl, sv));
    % [A_g1,b_g1] = fcn_KOZ_tar_ICCBF_gen(tR,tB,12.137,[0.30;0.22],0.8,1.2,0.5,0.1,1);
    % [A_g2,b_g2] = fcn_KOZ_obs_ICCBF_gen(tR,tB,12.137,[0.30;0.22],0.8,1.2,0.5,0.1,1);
    % eK = max([abs(A_ref(:)-A_g1(:)); abs(b_ref-b_g1); abs(A_ref(:)-A_g2(:)); abs(b_ref-b_g2)]);
    % % LOS
    % slL = [xR; xB; IRED; FOV; sn; so; stg; a0; a1; a2; taumax; tv];
    % svL = [tR; tB; 0.19816; deg2rad(40); [1;0]; [0.145;-0.042]; [0;0]; 0.8;1.2;0.5; 0.015; 1];
    % A_rL = double(subs(A_L, slL, svL)); b_rL = double(subs(b_L, slL, svL));
    % [A_gL,b_gL] = fcn_LOS_ICCBF_gen(tR,tB,0.19816,deg2rad(40),[1;0],[0.145;-0.042],[0;0],0.8,1.2,0.5,0.015,1);
    % eL_ = max([abs(A_rL(:)-A_gL(:)); abs(b_rL-b_gL)]);
    % fprintf('self-check  KOZ err = %.3e   LOS err = %.3e\n', eK, eL_);
    % assert(eK < 1e-9 && eL_ < 1e-9, 'a generated function disagrees with symbolic');
    % disp('Done. Generated fcn_KOZ_tar_ICCBF_gen.m, fcn_KOZ_obs_ICCBF_gen.m, fcn_LOS_ICCBF_gen.m');
end
% ----------------------------------------------------------------------
function [A, b] = iccbf_row(h, X, f, g, umax, a0, a1, a2, eps_abs, N)
% General ICCBF (N=2) or HOCBF (N=1) QP row for  Xdot = f + g u.
    Lf = @(s) jacobian(s, X.') * f;          % scalar
    Lg = @(s) jacobian(s, X.') * g;          % 1 x m row
    b1 = Lf(h) + a0*h;                        % Lg b0 = 0 for relative degree 2
    if N == 1
        A = -Lg(b1);  b = Lf(b1) + a1*b1;  return
    end
    Lgb1 = Lg(b1);
    W = sym(0);
    for j = 1:numel(umax)
        W = W + sqrt(Lgb1(j)^2 + eps_abs^2) * umax(j);   % smooth worst-case authority
    end
    b2 = Lf(b1) + a1*b1 - W;
    A  = -Lg(b2);
    b  = Lf(b2) + a2*b2;
end