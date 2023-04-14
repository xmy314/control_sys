from machine import Pin, PWM # type:ignore

class Servo:
    """ A simple class for controlling a 9g servo with the Raspberry Pi Pico.

    Attributes:

        minVal: An integer denoting the minimum duty value for the servo motor.

        maxVal: An integer denoting the maximum duty value for the servo motor.

    """

    def __init__(self, pin: int or Pin or PWM, minAng=0, maxAng=3.1415926, minVal=1650, maxVal=7950):
        """ Creates a new Servo Object.

        args:

            pin (int or machine.Pin or machine.PWM): Either an integer denoting the number of the GPIO pin or an already constructed Pin or PWM object that is connected to the servo.

            minVal (int): Optional, denotes the minimum duty value to be used for this servo.

            maxVal (int): Optional, denotes the maximum duty value to be used for this servo.

        """

        if isinstance(pin, int):
            pin = Pin(pin, Pin.OUT)  # type: ignore
        if isinstance(pin, Pin):
            self.__pwm = PWM(pin)
        if isinstance(pin, PWM):
            self.__pwm = pin

        self.__pwm.freq(50) # type: ignore
        self.minAng = minAng
        self.maxAng = maxAng
        self.minVal = minVal
        self.maxVal = maxVal
        self.neutral = int((maxVal+minVal)/2)
        self.current_angle = 0

    def deinit(self):
        """ Deinitializes the underlying PWM object.

        """
        self.__pwm.deinit() # type: ignore

    def goto(self, angle: float or int):
        """ Moves the servo to the specified position.

        args:

            value (int): The position to move to, represented by a value from 0 to pi (inclusive).

        """
        if angle<self.minAng:
            angle=self.minAng
        elif angle>self.maxAng:
            angle=self.maxAng
        
        target = int((self.maxVal-self.minVal)*(angle-self.minAng)/(self.maxAng-self.minAng)+self.minVal)
        self.__pwm.duty_u16(target) # type: ignore
        self.current_angle=angle

    def middle(self):
        """ Moves the servo to the middle.
        """
        self.__pwm.duty_u16(self.neutral) # type: ignore
        self.current_angle=(self.maxAng+self.minAng)/2
        
    def free(self):
        """ Make servo free to move.
        """
        self.__pwm.duty_u16(0) # type: ignore
