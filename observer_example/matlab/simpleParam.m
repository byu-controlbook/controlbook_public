% system parameters
P.A = [0, 1; 2, 3];
P.B = [0; 4];
P.C = [5, 0];

% sample rate of controller
P.Ts = 0.01;  
P.t_start = 0;
P.t_end = 40;
P.t_plot = 0.1;

%P.control_mode = 1;  % no integrator, no disturbance estimate
%P.control_mode = 2;  % integrator, no disturbance estimate
%P.control_mode = 3;  % no integrator, disturbance estimate
P.control_mode = 4;  % integrator, disturbance estimate

% pick poles of controller
wn_ctrl = 1;
zeta_ctrl = 0.707;
charpoly = [1,2*zeta_ctrl*wn_ctrl,wn_ctrl^2];
des_ctrl_poles = roots(charpoly);
integrator_pole = -1;  % pole for integrator

% pick poles of observer
wn_obsv = 10;
zeta_obsv = 0.707;
charpoly = [1,2*zeta_obsv*wn_obsv,wn_obsv^2];
des_obsv_poles = roots(charpoly);
dis_obsv_pole = -10; % pole for disturbance observer

switch P.control_mode
    case 1  % no integrator, no disturbance estimate 
        % is the system controllable?
        if rank(ctrb(P.A,P.B))~=2 
            disp('System Not Controllable'); 
        else
            P.K = place(P.A,P.B,des_ctrl_poles); 
            P.kr = -1/(P.C*inv(P.A-P.B*P.K)*P.B);
        end
        % is the system observable?
        if rank(obsv(P.A,P.C))~=2 
            disp('System Not Observable'); 
        else % if so, compute gains
            P.L = place(P.A',P.C',des_obsv_poles)'; 
        end
        
    case 2 % integrator, no disturbance estimate
        
        % augment system to add integrator
        P.A1 = [P.A, zeros(2,1); -P.C, 0];
        P.B1 = [P.B; 0];
        % is the system controllable?
        if rank(ctrb(P.A1,P.B1))~=3 
            disp('System Not Controllable'); 
        else
            K1 = place(P.A1,P.B1,[des_ctrl_poles;integrator_pole]); 
            P.K  = K1(1:2);
            P.ki = K1(3);
        end

        % is the system observable?
        if rank(obsv(P.A,P.C))~=2 
            disp('System Not Observable'); 
        else % if so, compute gains
            P.L = place(P.A',P.C',des_obsv_poles)'; 
        end
        
    case 3 % no integrator, disturbance estimate
        % is the system controllable?
        if rank(ctrb(P.A,P.B))~=2 
            disp('System Not Controllable'); 
        else
            P.K = place(P.A,P.B,des_ctrl_poles); 
            P.kr = -1/(P.C*inv(P.A-P.B*P.K)*P.B);
        end

        % augment system to disturbance observer
        P.A2 = [P.A, P.B; zeros(1,2), 0];
        P.C2 = [P.C, 0];
        % is the system observable?
        if rank(obsv(P.A2,P.C2))~=3 
            disp('System Not Observable'); 
        else % if so, compute gains
            P.L2  = place(P.A2',P.C2',[des_obsv_poles;dis_obsv_pole])'; 
            %P.L = L2(1:2);
            %P.Ld = L2(3);
        end
        
    case 4 % integrator, disturbance estimate

        % augment system to add integrator
        P.A1 = [P.A, zeros(2,1); -P.C, 0];
        P.B1 = [P.B; 0];
        % is the system controllable?
        if rank(ctrb(P.A1,P.B1))~=3 
            disp('System Not Controllable'); 
        else
            K1 = place(P.A1,P.B1,[des_ctrl_poles;integrator_pole]); 
            P.K  = K1(1:2);
            P.ki = K1(3);
        end
        % augment system to disturbance observer
        P.A2 = [P.A, P.B; zeros(1,2), 0];
        P.C2 = [P.C, 0];
        % is the system observable?
        if rank(obsv(P.A2,P.C2))~=3 
            disp('System Not Observable'); 
        else % if so, compute gains
            P.L2  = place(P.A2',P.C2',[des_obsv_poles;dis_obsv_pole])'; 
            %P.L = L2(1:2);
            %P.Ld = L2(3);
        end
end

