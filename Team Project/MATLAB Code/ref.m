function [h_2d,h_2d_dot,h_2d_Ddot] = ref(t)

    global h_max;
    global h_2e;
    global ref_sig;
    
    A = 0.05/4;

    
    if ref_sig == 2
        a = 1/60;
        h_2d = A*sin(a*t)+h_max/2;
        h_2d_dot = a*A*cos(a*t);
        h_2d_Ddot = -a^2*A*sin(a*t); 
    
    elseif ref_sig == 1
        pauseTime = 4*120;
        NofPeriod = floor(t./pauseTime);
        h_2d = zeros(size(t));
        h_2d_dot = zeros(size(t));
        h_2d_Ddot = zeros(size(t));
        for i = 1:length(t)
            if ~mod(NofPeriod(i),2)
                %First Quarter of the Signal
                h_2d(i) = A+h_max/2;
                h_2d_dot(i) = 0;
                h_2d_Ddot(i) = 0;
            else
                h_2d(i) = -A+h_max/2;
                h_2d_dot(i) = 0;
                h_2d_Ddot(i) = 0;
            end
            if t(i) == 0
                h_2d(i) =  h_2e;
                h_2d_dot(i) = 0;  
                h_2d_Ddot(i) = 0;
            end
        end
    
    elseif ref_sig == 3 
        
        PeriodTime = 4*120;
        NofPeriod = floor(t./PeriodTime);
        b = PeriodTime/4;
        h_2d = zeros(size(t));
        h_2d_dot = zeros(size(t));
        h_2d_Ddot = zeros(size(t));
    
        for i = 1:length(t)
            
            t_period = (t(i) - NofPeriod(i)*PeriodTime);
            
            if t_period <= b
                
                %First Quarter of the Signal
                h_2d(i) =  h_max/2 + A/b*t_period;
                h_2d_dot(i) = A/b;
                h_2d_Ddot(i) = 0;
           
            elseif (t_period > b) && (t_period <= 2*b)
                
                %Second Quarter of the Signal
                h_2d(i) = A+h_max/2;
                h_2d_dot(i) = 0;
                h_2d_Ddot(i) = 0;
            
            elseif (t_period > 2*b) && (t_period <= 3*b)
                
                %Third Quarter of the Signal
                h_2d(i) = A + h_max/2 - A/b*(t_period - 2*b);
                h_2d_dot(i) = A/b;
                h_2d_Ddot(i) = 0;
            
            else
                
                %Forth Quarter of the Signal
                h_2d(i) = h_max/2;
                h_2d_dot(i) = 0;
                h_2d_Ddot(i) = 0;
            
            end
            
            if t(i)==0
                
                h_2d(i) = h_2e;
                h_2d_dot(i) = 0;  
                h_2d_Ddot(i) = 0;
            
            end
        end
    end
end