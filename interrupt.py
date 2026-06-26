from machine import Pin
import time

Interrupt_triggered = False

def button_callback(pin):
    global Interrupt_triggered
    pin.irq(handler=None)
    if pin.value() == 1:
        Interrupt_triggered = True
    else:
        pin.irq(trigger=Pin.IRQ_RISING, handler=button_callback)
        print("Debounce gefaald, interrupt opnieuw ingesteld")

Interrupt_Pin = Pin(23, Pin.IN)
Interrupt_Pin.irq(trigger=Pin.IRQ_RISING, handler=button_callback)


while True:
    if Interrupt_triggered:
        Interrupt_triggered = False
        print("Triggered!")
        
        while Interrupt_Pin.value() == 1:
            time.sleep_ms(10)
        
        time.sleep_ms(20)
        Interrupt_Pin.irq(trigger=Pin.IRQ_RISING, handler=button_callback)
        print("Interrupt ended")