function [t,theta,phi,u] = leer_arduino(port,N)

    s = serialport(port,115200);
    flush(s);

    theta = zeros(1,N);
    phi   = zeros(1,N);
    u     = zeros(1,N);
    t     = zeros(1,N);

    for k = 1:N
        if read(s,1,"uint8") == 97
            if read(s,1,"uint8") == 98
                if read(s,1,"uint8") == 99
                    if read(s,1,"uint8") == 100

                        data = read(s,12,"uint8");
                        theta(k) = typecast(uint8(data(1:4)),'single');
                        phi(k)   = typecast(uint8(data(5:8)),'single');
                        u(k)     = typecast(uint8(data(9:12)),'single');

                        t(k) = k*0.02;
                        k
                    end
                end
            end
        end
    end

    % Gráfico
    figure;
    subplot(3,1,1), plot(t,theta), title('\theta')
    subplot(3,1,2), plot(t,phi), title('\phi')
    subplot(3,1,3), plot(t,u),   title('u')

end
