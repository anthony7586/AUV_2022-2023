# CSUF Senior Design RoboSub 2022 AUV

AUV Team Members (2021-2022 School Year):
* Nate Ruppert (Software, Code Documentation, Repository Management)
* Tyler Groom (Hardware)
* Douglas Shimizu (Software)
* Kayla Lee (Software, Project Documenting)
* Henry Lin (Software)

AUV Team MEmbers (2022-2023 school year):
* Anthony Ruiz 
* Izaiah Odunzu
* Kenneth Ayala
* Connor  

## For Future Teams: How to Run
This code is designed to operate and work on a Raspberry Pi System (4B)

The Pi you should receive within the AUV will have all necessary code under `~/AUV_2022`,
or the home directory for user pi as literal pathing `/home/pi/AUV_2022`. The directory matches
this project directory exactly. Meaning any path located respective to this repository matches the pi.

As in: (`<PROJECT_ROOT>/scripts/auv.py` <==> `/home/pi/AUV_2022/scripts/auv.py` etc.)

### Running the Code
Each major aspect of the project `thruster`, `camera`, `gripper`, `led` have their own test functions.
To run them individually, simply run `python <path_to_python_file`.

As an example, say we want to run thrusters by themselves as a test.

On the Raspberry Pi, navigate to the directory of the thruster code:

    cd /home/pi/AUV_2022/scripts/auv_lib

Then run the code by typing: `python thruster.py`. Should you know what you're doing, you may also create
an interactive session by typing `python -i thruster.py`.

### Run the main system
Similar to running code, the main system is located under `/scripts` and may be run by typing `python auv.py`.

# Final Note to Future Team
Good luck guys! As I'm sure Dr. George will tell you, your goal is to get this to work for RoboSub or any
other future competitions. Keep persevering and you'll get there. Our team aimed to make this code and hardware
much easier to take-off from than the state that we received the project in. I hope that I have succeeded at least
on the software side to that goal.

~ Nate
