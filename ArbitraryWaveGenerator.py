# Arbitrary waveform generator for Rasberry Pi Pico
# Requires 8-bit R2R DAC on pins 0-7. Works for R=1kOhm
# Achieves 125Msps when running 125MHz clock
# Rolf Oldeman, 13/2/2021. CC BY-NC-SA 4.0 licence
# tested with rp2-pico-20210205-unstable-v1.14-8-g1f800cac3.uf2
from machine import Pin,mem32
from rp2 import PIO, StateMachine, asm_pio
from array import array
from utime import sleep
from math import pi,sin,exp,sqrt,floor
from uctypes import addressof
from random import random

fclock=125000000 #clock frequency of the pico

DMA_BASE=0x50000000
CH0_READ_ADDR  =DMA_BASE+0x000
CH0_WRITE_ADDR =DMA_BASE+0x004
CH0_TRANS_COUNT=DMA_BASE+0x008
CH0_CTRL_TRIG  =DMA_BASE+0x00c
CH0_AL1_CTRL   =DMA_BASE+0x010
CH1_READ_ADDR  =DMA_BASE+0x040
CH1_WRITE_ADDR =DMA_BASE+0x044
CH1_TRANS_COUNT=DMA_BASE+0x048
CH1_CTRL_TRIG  =DMA_BASE+0x04c
CH1_AL1_CTRL   =DMA_BASE+0x050

PIO0_BASE      =0x50200000
PIO0_TXF0      =PIO0_BASE+0x10
PIO0_SM0_CLKDIV=PIO0_BASE+0xc8

#state machine that just pushes bytes to the pins
@asm_pio(out_init=(PIO.OUT_HIGH,PIO.OUT_HIGH,PIO.OUT_HIGH,PIO.OUT_HIGH,PIO.OUT_HIGH,PIO.OUT_HIGH,PIO.OUT_HIGH,PIO.OUT_HIGH),
         out_shiftdir=PIO.SHIFT_RIGHT, autopull=True, pull_thresh=32)
def stream():
    out(pins,8)

sm = StateMachine(0, stream, freq=125000000, out_base=Pin(0))
sm.active(1)

#2-channel chained DMA. channel 0 does the transfer, channel 1 reconfigures
p=array('I',[0]) #global 1-element array
def startDMA(ar,nword):
    #first disable the DMAs to prevent corruption while writing
    mem32[CH0_AL1_CTRL]=0
    mem32[CH1_AL1_CTRL]=0
    #setup first DMA which does the actual transfer
    mem32[CH0_READ_ADDR]=addressof(ar)
    mem32[CH0_WRITE_ADDR]=PIO0_TXF0
    mem32[CH0_TRANS_COUNT]=nword
    IRQ_QUIET=0x1 #do not generate an interrupt
    TREQ_SEL=0x00 #wait for PIO0_TX0
    CHAIN_TO=1    #start channel 1 when done
    RING_SEL=0
    RING_SIZE=0   #no wrapping
    INCR_WRITE=0  #for write to array
    INCR_READ=1   #for read from array
    DATA_SIZE=2   #32-bit word transfer
    HIGH_PRIORITY=1
    EN=1
    CTRL0=(IRQ_QUIET<<21)|(TREQ_SEL<<15)|(CHAIN_TO<<11)|(RING_SEL<<10)|(RING_SIZE<<9)|(INCR_WRITE<<5)|(INCR_READ<<4)|(DATA_SIZE<<2)|(HIGH_PRIORITY<<1)|(EN<<0)
    mem32[CH0_AL1_CTRL]=CTRL0
    #setup second DMA which reconfigures the first channel
    p[0]=addressof(ar)
    mem32[CH1_READ_ADDR]=addressof(p)
    mem32[CH1_WRITE_ADDR]=CH0_READ_ADDR
    mem32[CH1_TRANS_COUNT]=1
    IRQ_QUIET=0x1 #do not generate an interrupt
    TREQ_SEL=0x3f #no pacing
    CHAIN_TO=0    #start channel 0 when done
    RING_SEL=0
    RING_SIZE=0   #no wrapping
    INCR_WRITE=0  #single write
    INCR_READ=0   #single read
    DATA_SIZE=2   #32-bit word transfer
    HIGH_PRIORITY=1
    EN=1
    CTRL1=(IRQ_QUIET<<21)|(TREQ_SEL<<15)|(CHAIN_TO<<11)|(RING_SEL<<10)|(RING_SIZE<<9)|(INCR_WRITE<<5)|(INCR_READ<<4)|(DATA_SIZE<<2)|(HIGH_PRIORITY<<1)|(EN<<0)
    mem32[CH1_CTRL_TRIG]=CTRL1



def setupwave(buf,f,w):
    div=fclock/(f*maxnsamp) # required clock division for maximum buffer size
    if div<1.0:  #can't speed up clock, duplicate wave instead
        dup=int(1.0/div)
        nsamp=int((maxnsamp*div*dup+0.5)/4)*4 #force multiple of 4
        clkdiv=1
    else:        #stick with integer clock division only
        clkdiv=int(div)+1
        nsamp=int((maxnsamp*div/clkdiv+0.5)/4)*4 #force multiple of 4
        dup=1

    #fill the buffer
    for isamp in range(nsamp):
        buf[isamp]=max(0,min(255,int(256*eval(w,dup*(isamp+0.5)/nsamp))))

    #set the clock divider
    clkdiv_int=min(clkdiv,65535) 
    clkdiv_frac=0 #fractional clock division results in jitter
    mem32[PIO0_SM0_CLKDIV]=(clkdiv_int<<16)|(clkdiv_frac<<8)

    #start DMA
    startDMA(buf,int(nsamp/4))


#evaluate the content of a wave
def eval(w,x):
    m,s,p=1.0,0.0,0.0
    if 'phasemod' in w.__dict__:
        p=eval(w.phasemod,x)
    if 'mult' in w.__dict__:
        m=eval(w.mult,x)
    if 'sum' in w.__dict__:
        s=eval(w.sum,x)
    x=x*w.replicate-w.phase-p
    x=x-floor(x)  #reduce x to 0.0-1.0 range
    v=w.func(x,w.pars)
    v=v*w.amplitude*m
    v=v+w.offset+s
    return v

#some common waveforms. combine with sum,mult,phasemod
def sine(x,pars):
    return sin(x*2*pi)
def pulse(x,pars): #risetime,uptime,falltime
    if x<pars[0]: return x/pars[0]
    if x<pars[0]+pars[1]: return 1.0
    if x<pars[0]+pars[1]+pars[2]: return 1.0-(x-pars[0]-pars[1])/pars[2]
    return 0.0
def gaussian(x,pars):
    return exp(-((x-0.5)/pars[0])**2)
def sinc(x,pars):
    if x==0.5: return 1.0
    else: return sin((x-0.5)/pars[0])/((x-0.5)/pars[0])
def exponential(x,pars):
    return exp(-x/pars[0])
def noise(x,pars): #p0=quality: 1=uniform >10=gaussian
    return sum([random()-0.5 for _ in range(pars[0])])*sqrt(12/pars[0])
    

#make buffers for the waveform.
#large buffers give better results but are slower to fill
maxnsamp=4096 #must be a multiple of 4. maximum size is 65536
wavbuf={}
wavbuf[0]=bytearray(maxnsamp)
wavbuf[1]=bytearray(maxnsamp)
ibuf=0

#empty class just to attach properties to
class wave:
    pass


wave1=wave()
wave1.amplitude=0.5
wave1.offset=0.5
wave1.phase=0.0
wave1.replicate=1
wave1.func=sine
wave1.pars=[]

#step through frequencies
for freq in (1e5,2e5,5e5,1e6,2e6,5e6,1e7,2e7):
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2
freq=2e5


#step through amplitudes
freq=2e5
for amp in (0.1,0.2,0.3,0.4,0.5):
    wave1.amplitude=amp
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#step through offsets
wave1.amplitude=0.4
for offs in (0.4,0.5,0.6,0.7,0.8):
    wave1.offset=offs
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#pulse: step through duty cycle
wave1.amplitude=1.0
wave1.offset=0.0
wave1.func=pulse
wave1.pars=[0.0,0.5,0.0]
for duty in (0.5,0.4,0.3,0.2,0.1,0.05,0.02,0.01):
    wave1.pars[1]=duty
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#pulse: step through rise time,fall time
for rise in (0.02,0.05,0.10,0.20):
    wave1.pars=[rise,0.1,0.0]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2
for fall in (0.02,0.05,0.10,0.20):
    wave1.pars=[0.2,0.1,fall]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#pulse: step from saw to triangle to saw
wave1.pars=[0.0,0.0,1.0]
for x in (0.00,0.17,0.33,0.50,0.67,0.83,1.00):
    wave1.pars=[x,0,1-x]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#gaussian: step through widths
wave1.func=gaussian
for width in (0.15,0.1,0.05,0.02,0.01):
    wave1.pars=[width]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#sinc: step through widths
wave1.func=sinc
wave1.amplitude=0.5
wave1.offset=0.2
for width in (0.020,0.015,0.010,0.005,0.004,0.003):
    wave1.pars=[width]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#exponential: step through widths
wave1.func=exponential
wave1.amplitude=1.0
wave1.offset=0.0
for width in (0.20,0.10,0.05,0.02):
    wave1.pars=[width]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

# reverse exponential: step through widths
wave1.replicate=-1
for width in (0.20,0.10,0.05,0.02):
    wave1.pars=[width]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

# noise: step through quality
wave1.replicate=1
wave1.func=noise
wave1.amplitude=0.2
wave1.offset=0.5
for n in (1,2,5,10,20):
    wave1.pars=[n]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

# summation: build up a square wave from sines
wi={}
for i in range(6):
    wi[i]=wave()
    wi[i].amplitude=0.5/(2*i+1)
    wi[i].offset=0.5*(i==0)
    wi[i].phase=0.0
    wi[i].replicate=2*i+1
    wi[i].func=sine
    wi[i].pars=[]
    if i>0: wi[i-1].sum=wi[i]
    setupwave(wavbuf[ibuf],freq,wi[0]); ibuf=(ibuf+1)%2

#summation: double pulse
wave1=wave()
wave1.amplitude=1.0
wave1.offset=0.0
wave1.phase=0.0
wave1.replicate=1
wave1.func=pulse
wave1.pars=[0.0,0.1,0.0]

wave2=wave()
wave2.amplitude=0.4
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=1
wave2.func=pulse
wave2.pars=[0.1,0.1,0.1]
wave1.sum=wave2
for delay in (0.15,0.20,0.25,0.30,0.35):
    wave2.phase=delay
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#summation: gauss + noise
wave1=wave()
wave1.amplitude=-0.5
wave1.offset=0.75
wave1.phase=0.0
wave1.replicate=1
wave1.func=gaussian
wave1.pars=[0.05]

wave2=wave()
wave2.amplitude=0.1
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=1
wave2.func=noise
wave2.pars=[4]
wave1.sum=wave2
for amp in (0.01,0.02,0.03,0.04,0.05):
    wave2.amplitude=amp
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

#summation: exponential + gaussians
wave1=wave()
wave1.amplitude=0.8
wave1.offset=0.0
wave1.phase=0.0
wave1.replicate=1
wave1.func=exponential
wave1.pars=[0.2]

wave2=wave()
wave2.amplitude=0.2
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=1
wave2.func=gaussian
wave2.pars=[0.05]
wave1.sum=wave2
for r in (1,2,3,4,5):
    wave2.replicate=r
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

# product: sine * exponential
wave1=wave()
wave1.amplitude=0.4
wave1.offset=0.3
wave1.phase=0.0
wave1.replicate=10
wave1.func=sine
wave1.pars=[]

wave2=wave()
wave2.amplitude=1.0
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=1
wave2.func=exponential
wave2.pars=[0.2]
wave1.mult=wave2
for width in (0.05,0.10,0.15,0.20,0.25):
    wave2.pars=[width]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2


# product: sine * pulse
wave1=wave()
wave1.amplitude=0.4
wave1.offset=0.4
wave1.phase=0.0
wave1.replicate=10
wave1.func=sine
wave1.pars=[]

wave2=wave()
wave2.amplitude=1.0
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=1
wave2.func=pulse
wave2.pars=[0.0,0.1,0.0]
wave1.mult=wave2
for width in (0.05,0.10,0.15,0.20,0.25,0.30):
    wave2.pars=[0.0,width,0.0]
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

# product: gauss * sine
wave1=wave()
wave1.amplitude=0.4
wave1.offset=0.4
wave1.phase=0.0
wave1.replicate=1
wave1.func=sine
wave1.pars=[]

wave2=wave()
wave2.amplitude=1.0
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=20
wave2.func=gaussian
wave2.pars=[0.1]
wave1.mult=wave2
for r in (2,5,10,20):
    wave2.replicate=r
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2


# product: pulse * pulse
wave1=wave()
wave1.amplitude=0.4
wave1.offset=0.4
wave1.phase=0.0
wave1.replicate=1
wave1.func=pulse
wave1.pars=[0.5,0.0,0.5]

wave2=wave()
wave2.amplitude=1.0
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=20
wave2.func=pulse
wave2.pars=[0.0,0.5,0.0]
wave1.mult=wave2
for r in (2,5,10,20):
    wave2.replicate=r
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2


# modulate: sine with sine - small beta
wave1=wave()
wave1.amplitude=0.5
wave1.offset=0.5
wave1.phase=0.0
wave1.replicate=10
wave1.func=sine
wave1.pars=[]

wave2=wave()
wave2.amplitude=1.0
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=1
wave2.func=sine
wave2.pars=[]
wave1.phasemod=wave2
for amp in (0.1,0.2,0.5,1.0):
    wave2.amplitude=amp
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

# modulate: sine with sine - large beta
wave1=wave()
wave1.amplitude=0.5
wave1.offset=0.5
wave1.phase=0.0
wave1.replicate=1
wave1.func=sine
wave1.pars=[]

wave2=wave()
wave2.amplitude=1.0
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=1
wave2.func=sine
wave2.pars=[]
wave1.phasemod=wave2
for amp in (0.2,0.5,1.0,2.0,5.0,10.0):
    wave2.amplitude=amp
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2



# modulate: pulse with pulse
wave1=wave()
wave1.amplitude=1.0
wave1.offset=0.0
wave1.phase=0.0
wave1.replicate=10
wave1.func=pulse
wave1.pars=[0.0,0.5,0.0]

wave2=wave()
wave2.amplitude=0.5
wave2.offset=0.0
wave2.phase=0.0
wave2.replicate=1
wave2.func=pulse
wave2.pars=[0.5,0.0,0.5]
wave1.phasemod=wave2
for amp in (0.1,0.2,0.5,1.0):
    wave2.amplitude=amp
    setupwave(wavbuf[ibuf],freq,wave1); ibuf=(ibuf+1)%2

