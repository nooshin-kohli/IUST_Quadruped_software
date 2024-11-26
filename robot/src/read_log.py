import sys
import lcm

#from exlcm import example_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)
print(sys.argv[1])
log = lcm.EventLog(sys.argv[1], "r")
i = 1
for event in log:
    print(event.channel)
    i = i + 1
    if event.channel == "RESP":
    	print("dd")
        
#print(i/12)
