// int control_motors(char command, int speed, int ticks){
//     printf("function called\n");
//     if(command == 'f'){ //forward
//         //enable speed
//         change_duty(1, 0); //top
//         change_duty(2, speed); //left
//         change_duty(3, 0); //bottom
//         change_duty(4, speed); //right

//         //phase direction
//         set_pin(GPIOB, 0, 1); //J1 left forward
//         set_pin(GPIOB, 1, 0); //J2 bottom not moving
//         set_pin(GPIOB, 4, 0); //J3 top not moving
//         set_pin(GPIOB, 5, 1); //J4 right forward
//     }else if(command == 'b'){ //backward
//         //speed
//         change_duty(1, 0); //top
//         change_duty(2, speed); //left
//         change_duty(3, 0); //bottom
//         change_duty(4, speed); //right
//         //directions
//         set_pin(GPIOB, 0, 0); //J1 left backward
//         set_pin(GPIOB, 1, 1); //J2 bottom not moving
//         set_pin(GPIOB, 4, 1); //J3 top not moving
//         set_pin(GPIOB, 5, 0); //J4 right backward
//     }else if(command == 'r'){ //right
//         //speed
//         change_duty(1, speed); //top
//         change_duty(2, 0); //left
//         change_duty(3, speed); //bottom
//         change_duty(4, 0); //right
//         //direction
//         set_pin(GPIOB, 0, 0); //J1 left not moving
//         set_pin(GPIOB, 1, 1); //J2 bottom forward
//         set_pin(GPIOB, 4, 1); //J3 top forward
//         set_pin(GPIOB, 5, 0); //J4 right not moving
//     }else if(command = 'l'){ //left
//         //speed
//         change_duty(1, speed); //top
//         change_duty(2, 0); //left
//         change_duty(3, speed); //bottom
//         change_duty(4, 0); //right
//         //direction
//         set_pin(GPIOB, 0, 1); // J1 left not moving
//         set_pin(GPIOB, 1, 0); //J2 bottom backward
//         set_pin(GPIOB, 4, 0); //J3 top backward
//         set_pin(GPIOB, 5, 1); //J4 right not moving
//     }
// }

// //git add .
// //git commit -m "message"
// //git push origin master


// //in enable pwm function
// set_gpio_alt_func(GPIOA,9,1); //A9 = enable B1 channel: 2, timer: 1 (left)
//     set_gpio_alt_func(GPIOA,10,1); //A10 = enable A1 channel: 3, timer: 1 (bottom)
//     set_gpio_alt_func(GPIOA,11,1); //A11 = enable A2 channel: 4, timer: 1 (right)
//     set_gpio_alt_func(GPIOA,8,1); //A8 = enable B2 channel: 1, timer: 1 (top)