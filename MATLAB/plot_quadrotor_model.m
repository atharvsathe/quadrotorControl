% Tim McNamara
% 18-776 Project Part 1

function plot_quadrotor_model(x,t, u, x_true, t_true, u_norm, plot_pos, plot_velocity, plot_angle, plot_angvel, plot_input, legend_labels)
    tmin = 0;
    tmax = max(t_true);
    if plot_pos
        figure()
        subplot(3,1,1)
        plot(t, x(:,2), t_true, x_true(:,2))
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('x position (m)')
        title('x trajectory')
        ylim([-15, 15])
        xlim([tmin tmax])

        subplot(3,1,2)
        plot(t, x(:,4), t_true, x_true(:,4))
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('y position (m)')
        title('y trajectory')
        ylim([-15, 15])
        xlim([tmin tmax])

        subplot(3,1,3)
        plot(t, x(:,6), t_true, x_true(:,6))
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('z position (m)')
        title('z trajectory')
        ylim([-15, 15])
        xlim([tmin tmax])
    end

    % plot velocities
    if plot_velocity
        figure()
        subplot(3,1,1)
        plot(t, x(:,1), t_true, x_true(:,1))
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('x velocity (m/s)')
        xlim([tmin tmax])
        title('xdot trajectory')

        subplot(3,1,2)
        plot(t, x(:,3), t_true, x_true(:,3))
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('y velocity (m/s)')
        xlim([tmin tmax])
        title('ydot trajectory')

        subplot(3,1,3)
        plot(t, x(:,5), t_true, x_true(:,5))
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('z velocity (m/s)')
        xlim([tmin tmax])
        title('zdot trajectory')
    end
    
    % plot angles
    if plot_angle
        figure()
        subplot(3,1,1)
        plot(t, x(:,8)*180/pi, t_true, x_true(:,8)*180/pi)
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('\phi (deg)')
        title('\phi trajectory')
        ylim([-15, 15])
        xlim([tmin tmax])

        subplot(3,1,2)
        plot(t, x(:,10)*180/pi, t_true, x_true(:,10)*180/pi)
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('\theta (deg)')
        title('\theta trajectory')
        ylim([-15, 15])
        xlim([tmin tmax])

        subplot(3,1,3)
        plot(t, x(:,12)*180/pi, t_true, x_true(:,12)*180/pi)
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('\psi (deg)')
        title('\psi trajectory')
        ylim([-15, 15])
        xlim([tmin tmax])
    end
    
    % plot angular velocities
    if plot_angvel
        figure(4)
        subplot(3,1,1)
        plot(t, x(:,7)*180/pi, t_true, x_true(:,7)*180/pi)
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('p (deg/s)')
        title('p trajectory')
        xlim([tmin tmax])
        ylim([-50 50])

        subplot(3,1,2)
        plot(t, x(:,9)*180/pi, t_true, x_true(:,9)*180/pi)
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('q (deg/s)')
        title('q trajectory')
        xlim([tmin tmax])
        ylim([-50 50])

        subplot(3,1,3)
        plot(t, x(:,11)*180/pi, t_true, x_true(:,11)*180/pi)
        legend(legend_labels(1), legend_labels(2));
        grid on
        xlabel('time (s)')
        ylabel('r (deg/s)')
        title('r trajectory')
        xlim([tmin tmax])
        ylim([-50 50])
    end
    
    if plot_input
        figure(5)
        subplot(2,1,1)
        plot(t, u(1,:))
        grid on
        ylabel("Thrust (N)")
        ylim([0 20])
        xlabel('time (s)');
        subplot(2,1,2)
        plot(t, u(2,:), t, u(3,:), t, u(4,:));
        grid on
        ylim([-1,1])
        ylabel('torques (N m)')
        legend('\tau \phi','\tau \theta','\tau \psi')
    
        figure(6)
        plot(t, u_norm(1,:), t, u_norm(2,:), t, u_norm(3,:), t, u_norm(4,:))
        grid on
        ylabel("Input values (normalized)")
        legend('F', '\tau \phi','\tau \theta','\tau \psi')
        ylim([-1 1])
        xlabel('time (s)');
    end
    
end
