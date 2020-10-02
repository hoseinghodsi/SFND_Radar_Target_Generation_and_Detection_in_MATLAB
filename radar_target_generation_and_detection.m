clear all
clc;

%% Udacity Sensor Fusion Nanoprogram

%  ********************************************************
%  **  Project 4: Radar Target Generation and Detection  **
%  **                                                    **
%  **  Completed by Hossein Ghodsi                       **
%  **  July 15, 2020                                     **
%  ********************************************************


%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
 
Velocity = -28;         % m/s
InitialRange = 75;    % m
c = 3e8;               % m/s  (speed of light)

fprintf("Initial velocity = %d and range = %d\n", Velocity, InitialRange);

%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
Bsweep= c / (2*1);             % Bsweep in Hz           
Tchirp= 5.5 * (2 * 200 / c);   % Tchirp in seconds
slope = Bsweep/Tchirp;

%Operating carrier frequency of Radar 
fc= 77e9;             % carrier freq (Hz)

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = InitialRange + Velocity*t(i);
    td(i) = 2 * r_t(i)/c; 
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + slope*(t(i)^2)/2 ));
    delay = t(i) - td(i);
    Rx (i) = cos(2*pi*( fc*delay + slope * (delay^2)/2 ));% + 2.0*randn();
   
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i)*Rx(i);
    
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix = reshape(Mix, [Nr, Nd]);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
signal_fft = fft(Mix, Nr); 
L = Tchirp * Bsweep;

 % *%TODO* :
% Take the absolute value of FFT output
signal_fft = abs(signal_fft/L);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
signal_fft = signal_fft(1:L/2+1);

 % *%TODO* :
 % plot FFT output 
[fBeat, fBeatInd] = max(signal_fft); 
 
f = Bsweep*(0:(L/2))/L;
subplot(2,1,1)
plot(f,signal_fft)
title('Single-Sided Amplitude Spectrum')
xlabel('Frequency (Hz)')

fBeatTxt = ['Beat Frequency: ', num2str(fBeat), ' Hz'];
text(1.15*f(fBeatInd), 0.75*signal_fft(fBeatInd), fBeatTxt);

% Calculated Range 
R = (c*Tchirp*f)/(2*Bsweep);
subplot(2,1,2);
plot(R,signal_fft)
title('Range from Single-Sided Amplitude Spectrum')
xlabel('Range (m)') 
axis ([0 200 0 1]);

RangeTxt = ['Estimated Range: ', num2str(R(fBeatInd)), ' m'];
text(.6*R(fBeatInd), 1.25*signal_fft(fBeatInd), RangeTxt);



%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.


% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

%Select the number of Training Cells in both the dimensions.
Tr = 3;
Td = 12;

%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 2;
Gd = 3;

% offset the threshold by SNR value in dB
offset = 22;

%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
cellNum = (2*Tr+2*Gr+1)*(2*Td+2*Gd+1) - (2*Gr+1)*(2*Gd+1);
signalCFAR = zeros(Nr/2,Nd);

for i = 1:(Nr/2 - (2*Gr+2*Tr))
    for j = 1:(Nd - (2*Gd+2*Td))
        s1 = sum(db2pow(RDM(i:i+2*Tr+2*Gr, j:j+2*Td+2*Gd)),'all');
        s2 = sum(db2pow(RDM(i+Tr:i+Tr+2*Gr, j+Td:j+Td+2*Gd)),'all');    
        noiseLevel = s1 - s2;
        
        threshold = noiseLevel/cellNum;      
        threshold = db2pow(pow2db(threshold)) * offset;

        signal = db2pow(RDM(i+Tr+Gr, j+Td+Gd));
        if (signal <= threshold)
            signalCFAR(i+Tr+Gr,j+Td+Gd) = 0;
        else 
            signalCFAR(i+Tr+Gr,j+Td+Gd) = 1;
        end

    end
end


% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
 

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure('Name', '2d-CFAR');
surf(doppler_axis,range_axis,signalCFAR);
xlabel('Speed (m/s)')
ylabel('Range (m)')
zlabel('Detection') 
colorbar;



 
 