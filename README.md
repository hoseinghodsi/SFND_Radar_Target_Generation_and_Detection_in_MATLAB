# SFND_Radar_Target_Generation_and_Detection_in_MATLAB

This project uses Matlab to introduce frequency modulated continuous-wave (FMCW) radar and some post-processing techniques. 
The following topics are covered:

- Fast Fourier transforms (FFT) and 2D FFT
- Clutter v. target discrimination
- Sizing chirp bandwith to meet system requirements for range resolution
- Phased array beam steering to determine angle of arrival (AoA)
- Constant false alarm rate (CFAR) noise suppression
- Signal-to-noise ratio (SNR) and dynamic thresholding

## Project report:

### 2D CFAR process implementation

False alarms are problematic since frequent detection of unreal objects could make the entire autonomous driving impossible. One way to resolve the False alarm issue is called Constant False Alarm Rate (CFAR). CFAR varies the detection threshold based on the vehicle surroundings. The CFAR technique estimates the level of interference in radar range and doppler cells “Training Cells” on either or both the side of the “Cell Under Test”. The estimate is then used to decide if the target is in the Cell Under Test (CUT).

The key steps are as follows:
1. Loop over all cells in the range and doppler dimensions, starting and ending at indices which leave appropriate margins
2. Slice the training cells (and exclude the guard cells) surrounding the CUT
3. Convert the training cell values from decibels (dB) to power, to linearize
4. Find the mean noise level among the training cells
5. Convert this average value back from power to dB
6. Add the offset (in dB) to set the dynamic threshold
7. Apply the threshold and store the result in a binary array of the same dimensions as the range doppler map (RDM)

```
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
```
### Selection of Training, Guard cells and offset

I selected these values by trial and error. It seems that finding a right offset value was critical in filtering out the false alarms.

'''

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
'''

### Steps taken to suppress the non-thresholded cells at the edges

The CFAR process will generate a thresholded block, which is smaller than the Range Doppler Map as the CUT cannot be located at the edges of the matrix. Hence, a few cells will not be thresholded. To keep the map size the same, set those values to 0.

'''
signalCFAR = zeros(Nr/2,Nd);
'''

