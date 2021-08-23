import sys

filepath = r'2021-0820-1755-com.txt'
binfile = open(filepath, 'rb')
i=0
ch = binfile.read(1)

while ch:
	data = ord(ch)
	i = i + 1
	if i % 16 == 1:
		print ("\t0x%02X, " %(data), end='')
	elif i % 16 == 0:
		print ("0x%02X," %(data), end='')
		print ('')
	else:
		print ("0x%02X," %(data), end='')
	ch = binfile.read(1)

binfile.close()
