classdef signalGenerator
    %   produces: square_wave, sawtooth_wave, random_wave, sin_wave.
    %-----------------------------------
    properties
        amplitude
        frequency
        y_offset
    end
    %-----------------------------------
    methods
        %---constructor--------------------------------
        function self = signalGenerator(amplitude, frequency, y_offset)
            self.amplitude = amplitude;
            if nargin>1
                self.frequency = frequency;
            else
                self.frequency = 1;
            end
            if nargin>2
                self.y_offset = y_offset;
            else
                self.y_offset = 0;
            end
        end
        %-----------------------------------
        function out = square(self, t)
            if mod(t, 1/self.frequency)<=0.5/self.frequency
                out = self.amplitude + self.y_offset;
            else
                out = -self.amplitude + self.y_offset;
            end
        end
        %-----------------------------------
        function out = sawtooth(self, t)
            out = 4*self.amplitude*self.frequency * mod(t, 0.5/self.frequency)...
                - self.amplitude + self.y_offset;
        end
        %-----------------------------------
        function out = random(self, t)
            out = sqrt(self.amplitude)*randn + self.y_offset;
        end
        %-----------------------------------
        function out = sin(self, t)
            out = self.amplitude*sin(2*pi*self.frequency*t) + self.y_offset;
        end
    end
end