clear all; close all; 
n_flow = 3;

flow1 = NonOverlappedFlowInstance();
flow1.offset = 0;
flow1.period = 2;
flow1.delay = 2;
flow1.arrival_prob = 1;
flow1.success_prob = 0.5;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
flow2.offset = 1;
flow2.period = 2;
flow2.delay = 2;
flow2.arrival_prob = 0.8;
flow2.success_prob = 0.9;
flow2.constructEverything();

flow3 = NonOverlappedFlowInstance();
flow3.offset = 0;
flow3.period = 3;
flow3.delay = 3;
flow3.arrival_prob = 1;
flow3.success_prob = 0.25;
flow3.constructEverything();

flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;
flow_array{3} = flow3;



obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();

utility_coeff = [0.3, 0.7, 0.8];
%utility_coeff = [0.3, 0.7];
utility_coeff = utility_coeff./sum(utility_coeff);
utility_form = 'weighted_sum';


tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = ...
    getOptimalSolutionRAC(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);

tic;
[optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx, optimal_utility_RAC_approx, optimal_throughput_per_flow_RAC_approx] ...
= getApproximateSolutionRAC(obj, utility_coeff, utility_form);
fprintf('\nFinish getApproximateSolutionRAC with time %f seconds\n', toc);


tic;
[optimal_nu_dual, optimal_lambda_dual, optimal_utility_dual] = ...
    getOptimalSolutionRACDual(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRACDual with time %f seconds\n', toc);


tic;
[optimal_nu_dual_relax, optimal_lambda_dual_relax, optimal_mur_dual_relax, optimal_utility_dual_relax] = ...
    getOptimalSolutionRACDualRelax(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRACDual with time %f seconds\n', toc);

optimal_utility_RAC
optimal_utility_dual
optimal_utility_RAC_approx
optimal_utility_dual_relax
x1 = squeeze(optimal_policy_RAC(1,:,:))
%x2 = squeeze(optimal_policy_RAC(2,:,:))
optimal_nu_dual
optimal_nu_dual_relax
optimal_lambda_dual
optimal_lambda_dual_relax
mur1 = squeeze(optimal_mur_dual_relax(1,:,:))
%mur2 = squeeze(optimal_mur_dual_relax(2,:,:))

 
% 
% T = 100;
% 
% % tic;
% % [theoretical_utlity_full_dual, theoretical_throughput_full_dual,...
% %     primal_rate_user_full_dual, primal_state_full_dual, primal_action_full_dual, primal_state_action_full_dual, primal_state_action_distribution_full_dual, ...
% %     dual_queue_user_full_dual, dual_lambda_full_dual] ...
% %     = getFullDualDistributedSolution(obj, utility_coeff, utility_form, T);
% % fprintf('\nFinish getFullDualDistributedSolution with time %f seconds\n', toc);
% 
% tic;
% [theoretical_utlity_full_dual_v, theoretical_throughput_full_dual_v,...
%     primal_rate_user_full_dual_v, primal_state_action_full_dual_v, primal_state_action_distribution_full_dual_v, ...
%     dual_queue_user_full_dual_v, dual_lambda_full_dual_v] ...
%     = getFullDualDistributedSolution_v(obj, utility_coeff, utility_form, T);
% fprintf('\nFinish getFullDualDistributedSolution_v with time %f seconds\n', toc);
% 
% tic;
% [theoretical_utlity_full_dual_approx, theoretical_throughput_full_dual_approx,...
%     primal_rate_user_full_dual_approx, primal_state_full_dual_approx, primal_action_full_dual_approx, primal_state_action_full_dual_approx, primal_state_action_distribution_full_dual_approx, ...
%     dual_queue_user_full_dual_approx, dual_mu_full_dual_approx] ...
%     = getFullDualApproxDistributedSolution(obj, utility_coeff, utility_form, T);
% toc;
% 
% %vectorize scheduling policy
% primal_state_action_distribution_full_dual_approx_v = zeros(obj.n_state_action, obj.period_lcm, T);
% for idx=1:obj.n_state_action
%     [ss, aa] = obj.getStateActionFromIdx(idx);
%     for hh=1:obj.period_lcm
%         primal_state_action_distribution_full_dual_approx_v(idx,hh,:) = primal_state_action_distribution_full_dual_approx(hh,ss,aa,:);
%     end
% end
% 
% tic;
% [successful_transmission_full_dual, ...
%     state_action_distribution_full_dual, system_state_full_dual, system_action_full_dual, state_action_per_slot_full_dual ] ...
%     = fullDualSchedule(obj, T, primal_state_action_distribution_full_dual_v);
% fprintf('\nFinish fullDualSchedule with time %f seconds\n', toc);
% 
% 
% [successful_transmission_full_dual_approx, ...
%     state_action_distribution_full_dual_approx, system_state_full_dual_approx, system_action_full_dual_approx, state_action_per_slot_full_dual_approx] ...
%     = fullDualSchedule(obj, T, primal_state_action_distribution_full_dual_approx_v);
% 
% 
% tic;
%  [successful_transmission_half_dual, ...
%     state_action_distribution_half_dual, system_state_half_dual, system_action_half_dual, state_action_per_slot_half_dual, ...
%     primal_rate_user_half_dual, primal_state_action_half_dual, primal_state_action_distribution_half_dual, ...
%     dual_queue_user_half_dual, dual_lambda_half_dual] ...
%     = halfDualSchedule(obj, utility_coeff, utility_form, T);
% fprintf('\nFinish halfDualSchedule2 with time %f seconds\n', toc);
% 
% 
% [successful_transmission_half_dual_approx, ...
%     state_action_distribution_half_dual_approx, system_state_half_dual_approx, system_action_half_dual_approx, state_action_per_slot_half_dual_approx, ...
%     primal_rate_user_half_dual_approx, primal_state_half_dual_approx, primal_action_half_dual_approx, primal_state_action_half_dual_approx, ...
%     dual_queue_user_half_dual_approx, dual_mu_half_dual_approx] ...
%     = halfDualApproxSchedule(obj, utility_coeff, utility_form, T);
% 
% 
% cummean_successful_transmission_full_dual = cummean(successful_transmission_full_dual,2);
% cummean_successful_transmission_full_dual_approx = cummean(successful_transmission_full_dual_approx,2);
% cummean_successful_transmission_half_dual = cummean(successful_transmission_half_dual,2);
% cummean_successful_transmission_half_dual_approx = cummean(successful_transmission_half_dual_approx,2);
% 
% cummean_utlity_full_dual = zeros(T,1);
% cummean_utlity_full_dual_approx = zeros(T,1);
% cummean_utlity_half_dual = zeros(T,1);
% cummean_utlity_half_dual_approx = zeros(T,1);
% 
% for tt=1:T
%     for kk=1:obj.n_flow
%         if(isequal(utility_form, 'weighted_sum'))
%             cummean_utlity_full_dual(tt) =  cummean_utlity_full_dual(tt) + utility_coeff(kk)*(cummean_successful_transmission_full_dual(kk,tt));
%             cummean_utlity_full_dual_approx(tt) = cummean_utlity_full_dual_approx(tt) + utility_coeff(kk)*(cummean_successful_transmission_full_dual_approx(kk,tt));
%             cummean_utlity_half_dual(tt) =  cummean_utlity_half_dual(tt) + utility_coeff(kk)*(cummean_successful_transmission_half_dual(kk,tt));
%             cummean_utlity_half_dual_approx(tt) = cummean_utlity_half_dual_approx(tt) + utility_coeff(kk)*(cummean_successful_transmission_half_dual_approx(kk,tt));
%         elseif(isequal(utility_form, 'weighted_log_sum'))
%             cummean_utlity_full_dual(tt) =  cummean_utlity_full_dual(tt) + utility_coeff(kk)*log(cummean_successful_transmission_full_dual(kk,tt));
%             cummean_utlity_full_dual_approx(tt) = cummean_utlity_full_dual_approx(tt) + utility_coeff(kk)*log(cummean_successful_transmission_full_dual_approx(kk,tt));
%             cummean_utlity_half_dual(tt) =  cummean_utlity_half_dual(tt) + utility_coeff(kk)*log(cummean_successful_transmission_half_dual(kk,tt));
%             cummean_utlity_half_dual_approx(tt) = cummean_utlity_half_dual_approx(tt) + utility_coeff(kk)*log(cummean_successful_transmission_half_dual_approx(kk,tt));
%         else
%             error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
%         end
%     end
% end
% 
% for nn=1:obj.n_flow
%     figure;
%     font_size = 12;
%     line_width = 3;
%     set(gca,'FontSize',font_size);
%     hold on;
%     %optimal
%     plot(1:T,theoretical_throughput_full_dual_v(nn,:), '-g', 'Linewidth', line_width);
%     plot(1:T,cummean_successful_transmission_full_dual(nn,:), '-b', 'Linewidth', line_width);
%     plot(1:T,cummean_successful_transmission_half_dual(nn,:), '-k', 'Linewidth', line_width);
%     %approx
%     plot(1:T,theoretical_throughput_full_dual_approx(nn,:),'-c','Linewidth', line_width);
%     plot(1:T,cummean_successful_transmission_full_dual_approx(nn,:), '-m', 'Linewidth', line_width);
%     plot(1:T,cummean_successful_transmission_half_dual_approx(nn,:), '-y', 'Linewidth', line_width);
%     %RAC optimal
%     plot(1:T,optimal_throughput_per_flow_RAC(nn).*ones(1,T), '-.r', 'Linewidth', line_width);
%     hold off;
%     xlabel('Slot','FontSize', font_size, 'FontName', 'Arial');
%     ylabel('Running Timely Throu.','FontSize', font_size, 'FontName', 'Arial');
%     legend('Full-Dual-Theory', 'Full-Dual-Schedule',  'Half-Dual-Schedule', ...
%            'Full-Dual-Approx-Theory', 'Full-Dual-Approx-Schedule', 'Half-Dual-Approx-Schedule', ...
%            'Optimal', 'Location', 'Southeast');
% %    legend('Half-Dual-Schedule', 'Optimal', 'Location', 'Southeast');
%  %   ylim([0.1,0.3]);
% %    xlim([1,T]);
%     %title(sprintf('%d flows, ',obj.n_flow));
%     box on;
%     grid off;
% end
% 
% 
% figure;
% font_size = 12;
% line_width = 3;
% set(gca,'FontSize',font_size);
% hold on;
% %optimal 
% plot(1:T,theoretical_utlity_full_dual_v, '-g', 'Linewidth', line_width);
% plot(1:T,cummean_utlity_full_dual, '-b', 'Linewidth', line_width);
% plot(1:T,cummean_utlity_half_dual, '-k', 'Linewidth', line_width);
% %approx
% plot(1:T,theoretical_utlity_full_dual_approx, '-c', 'Linewidth', line_width);
% plot(1:T,cummean_utlity_full_dual_approx, '-m', 'Linewidth', line_width);
% plot(1:T,cummean_utlity_half_dual_approx, '-y', 'Linewidth', line_width);
% %RAC optimal
% plot(1:T,optimal_utility_RAC.*ones(1,T), '-.r', 'Linewidth', line_width);
% hold off;
% xlabel('Slot','FontSize', font_size, 'FontName', 'Arial');
% ylabel('Running Utility','FontSize', font_size, 'FontName', 'Arial');
% legend('Full-Dual-Theory', 'Full-Dual-Schedule', 'Half-Dual-Schedule', ...
%        'Full-Dual-Approx-Theory', 'Full-Dual-Approx-Schedule',  'Half-Dual-Approx-Schedule', ...
%        'Optimal', 'Location', 'Southeast');
% %legend('Half-Dual-Schedule', 'Optimal', 'Location', 'Southeast');
% %    ylim([0.1,0.25]);
% %    xlim([1,T]);
% %title(sprintf('%d flows, ',obj.n_flow));
% box on;
% grid off;

