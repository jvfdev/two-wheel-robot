void init_PID() {
  // Sets up timer to run control loop at 200 Hz
  noInterrupts(); // disable interrupts before setting registers
  TCCR1A = 0; //sets register to zero, not yet sure why this is here.
  TCCR1B = 0; // same as above
  OCR1A = 1999; // sets compare match register
  TCCR1B |= (1 << WGM12); // turns on CTC
  TCCR1B |= (1 << CS11); // sets prescaler to 8
  TIMSK1 |= (1 << OCIE1A); // enable the interrupt
  interrupts();
}
