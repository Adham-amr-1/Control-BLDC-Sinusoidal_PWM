import math
#Timer Count -> 20 KHz
Counter = 1439
# Size of LookUp Table 
Sine_Steps = 1024 
# Array for PWM Values
PWM = []
for i in range(Sine_Steps):
    Thetai = ( 2 * math.pi * i / Sine_Steps ) 
    Sini = math.sin(Thetai)
    Normalizedi = (Sini + 1.0) / 2.0
    PWMi = round(Counter * Normalizedi)
    PWM.append(PWMi)
print()
print(PWM)
print()