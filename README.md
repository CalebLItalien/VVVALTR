# VVVALTR Project

This codebase contains code to run and experiment with our tensegrity tobor VVVALTR, or VVireless Vibrationally Actuated Limbless Tensegrity Robot. It also contains a small amount of information on the hardware used for VVVALTR and catalogs VVVALTR's current state in terms of both hardware and software. 

## Hardware

Go to the Hardware Section of the Wiki to see how a functional strut should look. All of the connections are labeled and have what their expected voltage should be. 

## Software

## Assimilating a new strut with the rest of the project

Setting up your arduino environment:
1. Inside your Arduino IDE preferenes (file -> preferences), add 'http://www.buntsoft.com/simblee/' to the list of Additional Boards. 
2. Open the board manager (tools -> boards -> board manager) and search for Simblee
3. Install the Simblee board
4. (Possibly) Still in the board manager search for SAM
5. (Possibly) Install the Cortex-M3 board
4. Do 'git clone http://github.com/jrowberg/i2cdevlib' (or otherwise obtain the i2c dev lib from https://www.i2cdevlib.com/usage)
5. Copy i2cdev/Arduino/I2Cdev , i2dev/Arduino/MPU6050 , and i2cdev/STM32 to ~/Arduino/Libraries/
6. Restart the arduino ide

Follow these instructions if you are using a new RFDuino in either a new strut, or for replacement purposes:

1. Use the USB Shield to connect the RFDuino to the computer. 
2. In Desktop/VVValtr, open the folder called brushedRFDMotorControl, and open the Arduino file inside of it.
3. In the section titled "//BLUETOOTH VARIABLES" find the line that says '#define DEV_NAME "ValtrExampleName"'.
4. Enter a new name for the strut. This should be VALTR + the next number available. (e.g. VALTR9).
5. Verify and upload the code to the RFDuino. 
6. Press the Bluetooth button by the top right of the toolbar, and go to Bluetooth Settings.
7. By the bottom of the left column press the "+" button. This should open a new window and be searching for Bluetooth devices. The device you just named the board should show up shortly.
8. Once the name of the device shows up, click on it and press "Next".
9. This should take a few seconds and say that the device was successfully connected. It will show you a 12 character address separated by colons (e.g. D3:8E:9E:80:21:BD).
10. Add the board name and its address in the "Checking if a strut is communicating with the computer" section of the README.
11. In order to use this board for testing, go to Desktop/VVValtr/tensLearning.py, and go to the "#Brushless Motors" Section.
12. Add the name of your board to the code along with its Bluetooth Address with the same format of the previous boards (e.g. VALTR9 = 'E2:D8:73:53:F4:34').
13. Go to the main function of this code and replace one of the boards already called, with the name of your board (e.g. From BO = BayesianOptimizer([VALTR5, VALTR6, VALTR8]) to BO = BayesianOptimizer([VALTR5, VALTR6, VALTR9])).

## Checking if a strut is communicating with the computer
    
To see if an individual strut is communicating with the computer, follow these instructions:
    
1. Open terminal.
2. Type in one of the following commands based on the number of the strut that is being tested and hit `ENTER`:
        
    For #5: 
     ```
gatttool --device=E2:22:B3:44:73:F0 --addr-type=random -I
    ```
    For #6:
     ```
gatttool --device=D5:0D:E4:1F:30:8F --addr-type=random -I
    ```
    For #7: 
    ```
gatttool --device=C2:72:E6:99:E3:9E --addr-type=random -I
    ```
    For #8:
    ```
gatttool --device=E1:3F:AE:94:34:AF --addr-type=random -I
    ```
    For #9:
    ```
gatttool --device=E2:D8:73:53:F4:34 --addr-type=random -I
    ```
    For the prototype on the breadboard:
    ```
gatttool --device=DB:05:9E:7A:DB:67 --addr-type=random -I
    ```
    For Encoder 1:
    ```
gatttool --device=E2:EC:2E:0B:BE:B9 --addr-type=random -I
    ```
    For Encoder 2:
    ```
gatttool --device=C0:17:4D:9F:1C:38 --addr-type=random -I
    ```

3. The terminal should display [Device number][LE]. Then type `connect` and hit `ENTER`

4. The terminal should then return Attempting to connect to + the device number of the strut. It should then return "Connection successful" almost immediately if the strut is properly communicating. 
5. To check if data is being recorded, type `char-read-hnd 0x11` and hit `ENTER`.

6. This should return "Characteristic value/descriptor: 00 00 00". Otherwise, data is not being recorded. If this is the case, either there is no SD card, or you might have to unplug and replug in one of the battery connections.
7. To check if data can be sent from the computer to the board, type:       
    ```
char-write-req 0x14 0x'Number in hex from 00-ff'. (Check link in Wiki for table converting decimal to hex)
    ```
An example of this command would be:
```
char-write-req 0x14 0xff
``` 
and hit `ENTER`.
This should move the motor very fast. If it is moving slowly, there may be a hardware problem.

8. If a high value and a low value are moving at the same low speed, there may be a hardware problem.

If any of these steps are not working, check the hardware to see if there are any loose connections on the board.

## Collecting accelerometer data

To collect data from the accelerometer connected to the testing spring follow these steps:

1. First find the cable that allows connection from the microSD card to USB and plug this into the computer.
2. Open the folder that the microSD is in. There should be a test.txt file.
3. Open the file and delete all of its contents.
4. Place the microSD back in the RFDuino microSD shield and mount this on the RFDuino.
5. Repeat instructions 1-6 in the section above (Checking if a strut is communicating with the computer).
6. Write the same command in step 7 as the section above except replacing "0xff" with any value in hex which in decimal is between 0-255.
7. Remove the microSD card and place it back in the cable to connect it to the computer, and open the test.txt file. (If there is a problem opening this file, there may be a problem with the microSD card.)
8. Copy this folders contents into the folder /home/Documents/MATLAB/test_d022618_t0248am_s130.txt (if there are already numbers in this file clear its contents).
9. Scroll to the last line of this file and if it is a row of dashes (--, --, --) delete this line.
10. Open the program found in /home/Documents/MATLAB/ExpFreqAnalysis.m and run it.

## Running Bayesian Optimization code (tensLearning)

To run tensLearning tests, follow these steps.
1. Make sure the table is level.
2. Remove everything from the testing table, so that it is completely clear (including VVVALTR).
3. Run the following commands in a terminal window.
    ```
    cd ~/Desktop/VVValtr
    python tensLearning.py
    ```
(Make sure all the proper modules are downloaded...if tensLearning returns that it is missing a specific module, exit out of python and use the command
```
pip install --user (insert module name without the parantheses)
```
4. Follow steps 3-10 of the "Running Manual Tests" Section.
5. After the initial messages mentioned in step 10 of "Running Manual Tests", the terminal will print the location of the file in which the data from the test is being stored. Navigate to the file after the test runs to see the data.

   If there is a connection error, go to the above section (Checking if a strut is communicating with the computer).
   If the problem persists, there may be a problem with the hardware connections.

6. The beginning of the test will look like this:
 ```
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
init   	 [ 178.   58.  176.]. 	  38.9486841883 	 45.3100430368
init   	 [  64.   12.  228.]. 	  45.3100430368 	 45.3100430368
init   	 [ 157.   23.   52.]. 	  1.41421356237 	 45.3100430368
```
This is a sign the test is functioning. 


8. This is what the terminal will display if the test is running properly:
```
1      	 [  81.   11.  212.]. 	  69.8569967863 	 69.8569967863
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
2      	 [ 199.  248.   19.]. 	  24.4131112315 	 69.8569967863
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
3      	 [  87.   65.  231.]. 	  70.7742891169 	 70.7742891169
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
4      	 [  80.00000153    7.00000612  202.00001531]. 	  58.9406481132 	 70.7742891169
```

7. If the terminal displays:
```
Characteristic value was written successfully
Characteristic value was written successfully
Characteristic value was written successfully
```
and no motors are running for a more than a minute, then restart the test.


## Running Manual Tests (runTens)

To run a manual test on the latest model of VVVALTR (with struts VALTR5, VALTR6, and VALTR7), follow these steps.

1. Remove everything from the testing table, so that it is completely clear (including VVVALTR). 
2. Run the following commands in a terminal window.
    ```
    cd ~/Desktop/VVValtr
    python runTens.py
    ```
This will bring up a window with a portion of an image. This should look like part of the tensegrity table. If it doesn't, hit `CTRL + C` in the terminal window and repeat 2. If the problem persists, contact your friendly neighborhood CSC student. 
3. The terminal should prompt you with
    ```
    Is this the correct camera?
    ```
Type `y` and hit `ENTER`.
4. The terminal should then say
    ```
    Using camera 1
    Base image obtained. Place tensegrity
    ```
Place VVVALTR in the middle of the table and step back.
5. About ten seconds later, a window with six sliders and a mostly black image should pop up. The upper right hand corner of the image should have an orange blob, and the lower left hand corner should have a pink blob. 
6. The terminal will say
    ```
    Select the upper left boundary puffball
    ```
To do so, adjust the "upper hue" slider down by approximately 30. Only the orange ball (and maybe some small bits of blue in the center) should be left in the image.
7. Click on the image itself, then hit the `ESC` key. The terminal should prompt you with 
    ```
    Done (Y/N)
    ```
Type `Y` and hit `ENTER`. Something like 
    ```
    (array([  0, 140,   0], dtype=uint8), array([ 89, 255, 255], dtype=uint8))
    ```
will be printed on the terminal. Ignore this.
8. A similar window should pop up again, this time alongside a terminal prompt saying
    ```
    Select the bottom right boundary puffball
    ```
Repeat Step 6, but this time adjust the "lower hue" slider up by approximately 30. Then adjust the "upper hue" slider down by 10. Similar to last time, only the pink puff ball (and some small blue in the center) should be in the image.
9. Click on the image itself, then hit the `ESC` key. The terminal should prompt you with 
    ```
    Done (Y/N)
    ```
Type `Y` and hit `ENTER`. Something like 
    ```
    (array([  0, 140,   0], dtype=uint8), array([ 89, 255, 255], dtype=uint8))
    ```
will be printed on the terminal. Ignore this.
10. Once the window closes, the terminal will print debug output looking something like
    ```
    Motor 0 Start...
    SD Card Shield: Failure
    Data File Open: Failure
    Accelerometer Start: Success
    Motor 1 Start...
    SD Card Shield: Failure
    Data File Open: Failure
    Accelerometer Start: Success
    Motor 2 Start...
    SD Card Shield: Failure
    Data File Open: Failure
    Accelerometer Start: Success
    ```
This is all fine, even if it says `Failure` for some parts. 
11. The terminal will then prompt you for a motor frequency for each active strut as follows:
    ```
    Frequency for motor n >> 
    ```
where `n` is the motor number. This should happen three times. For each one, type a positive integer between `0` and `255`, then hit `ENTER`. 
12. Once all values have been entered, the tensegrity should run for about ten seconds. About five seconds after it stops, the console should print out its ending location and distance traveled, then prompt you for more frequencies to test.
