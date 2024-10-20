 AVR program to respond to the Davis Vantage green dot Console data 
 logger challenge and "PLL lock" loop.  The program size is such that
 it should work with ATTINY85 and ATTINY45 (not tested) devices.                
                                                                 
 The program is based on work and code created by Torkel M. Jodalen 
 and others which is documented here:                            
       https://www.annoyingdesigns.com/meteo/DavisSPI.pdf        
                                                                 
 Additional information and the original ATTINY85 hex files can be
 found here.                                                     
      https://github.com/SpiceWire/davis-vantage-pro2-datalogger-emulator

NOTE: If powering the ATTiny85 from the console (3.0V) the clock should not
      exceed 8MHz.  Setting the low fuse bits to 0xD2 will acheive this
