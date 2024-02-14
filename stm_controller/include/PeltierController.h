#include <Arduino.h>


class PeltierController {
    public:
        enum PinMode                {PWM, ONOFF, ARDUINOPWM};
        enum RegulateCoolingUntil   {REFERENCE_GREATER, REFERENCE_LOWER};
        // enum CompareMode            {FIXED, PERCENTAGE_REFERENCE, PERCENTAGE_MOVINGAVG};

		PeltierController(PinName controlPin, PinMode controlMode = ONOFF);
		void init();
		bool start(uint32_t currentValue, uint32_t referenceValue, RegulateCoolingUntil compareDirection = REFERENCE_LOWER);
		
	private:
		PinName controlPin;
        PinMode controlMode;
        uint32_t pwmChannel;
        HardwareTimer *pwmTimer;

        // see https://stackoverflow.com/questions/45386009/assign-function-pointer-inside-class
        bool (PeltierController::*regulateFunction) (uint32_t, uint32_t, RegulateCoolingUntil);
        bool regulatePWM(uint32_t currentValue, uint32_t referenceValue, RegulateCoolingUntil compareDirection); // wouldn't need to be a member function
        bool regulateONOFF(uint32_t currentValue, uint32_t referenceValue, RegulateCoolingUntil compareDirection);
        bool regulateARDUINOPWM(uint32_t currentValue, uint32_t referenceValue, RegulateCoolingUntil compareDirection);
};