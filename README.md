Bacpot repo for 2025 for the STM L432KC. THis should take in UART(of the direction and speed), each a byte.
Then it will control the omni wheels to move in that direction. less context switching overhead. 
So parts of this are 
PID
MOTOR CONTROL
UART CONTROL
QUADRATURE ENCODER
