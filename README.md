# About

Software developed as a Robotics project for class IF826 in CIn-UFPE (Brazil, 2016).

This project includes software for a raspiberry-powered robot to execute the tasks of moving inside a maze and detecting objects with its camera using image processing algorithms. It also includes an API to simulate the robot's mechanics using the software V-REP.

# Development Team

- Anderson Urbano (aafu@cin.ufpe.br)
- Danilo Alfredo  (dams@cin.ufpe.br)
- Diogo Rodrigues (drs3@cin.ufpe.br)
- Heitor Rapela   (hrm@cin.ufpe.br)

# How to Run

The implemented API allows for the same code to be compiled tu run on V-REP simulator or on the real robot.

To compile the source code on `src` folder for V-REP, just use the command `make sim` and for the real robot use `make real`. Executable files will be generated at `bin`.

To execute a robot program from the root folder use `./bin/<nome do programa>.bin`.

Example: `$ ./bin/remoteControl.bin` (on the real robot it is necessary to use `sudo`).

There are many initialization files at `ini`. The file `hardware.ini` controls a few PID parameters for the robot.