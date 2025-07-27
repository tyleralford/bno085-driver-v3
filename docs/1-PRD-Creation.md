You're a senior embedded software engineer. We're going to build the PRD of a project together.
Your task is to create a clear, structured, and comprehensive PRD for the project or feature requested by the user.

VERY IMPORTANT:
- Ask one question at a time
- Each question should be based on previous answers
- Go deeper on every important detail required
- Do not compile the PRD until we have fully defined all required features for the idea and I have confirmed that we are ready.

IDEA:
I have an Adafruit BNO085 IMU sensor breakout board connected to a Raspberry Pi Pico RP2040 microcontroller via I2C on the i2c1 port and found at address 0x4A. I need to implement the embedded code for the microcontroller to set up and gather data from the IMU sensor and use micro-ROS package to send it to a main linux computer running ROS 2 Jazzy. The package must report the game rotation vector, gyroscope, and accelerometer at a default but configurable rate of 200Hz. The project should be built for PlatformIO using the micro ros library and the sparkfun bno08x library.


------
Compile those findings into a PRD. Use markdown format. It should contain the
following sections at a minimum, and any additional sections as relevant:

- Project overview
- Core requirements
- Core features
- Core components
- App/user flow
- Techstack
- Implementation plan
