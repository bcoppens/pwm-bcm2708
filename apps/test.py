#!/usr/bin/python
import sys
import time

if len(sys.argv) >= 1 and sys.argv[1] == "pi":
  f_out = "/sys/bus/platform/drivers/bcm2708_pwm/pwm_led0_color_string"
else:
  f_out = None

def output_list(l):
  out = ""
  for (rgb) in reversed(l):
    out += "%i,%i,%i " % (rgb)
  
  if f_out is not None:
    f = open(f_out, "w")
    f.write(out)
    f.close()
  else:
    print out

def clean_list():
  return [(0,0,0)] * 16

l = clean_list()

for j in range(0,16):
  l = clean_list()
  l[j] = ( 50, 0, j*50/16 )
  output_list(l)

for i in range(1,4):
  for j in range(0,16):
    l[j] = ( i*50, 0, i*50 )
    output_list(l)

for i in range(1,4):
  for j in range(0,16):
    l[j] = ( 150, i*50, 150-i*50 )
    output_list(l)

for i in range(1,4):
  for j in range(0,16):
    l[j] = ( 150-i*50, 150-i*50, 0 )
    output_list(l)

# Smileys
colors = [(0,0,150), (0,150,0), (150,0,0), (0,0,150)]
for i in range(0,4):
  l = clean_list()
  color = colors[i]
  l[10] = l[15] = color
  l[2] = l[3] = l[4] = l[5] = l[6] = l[7] = color
  output_list(l)
  time.sleep(1)
time.sleep(1)
l = clean_list()
output_list(l)

