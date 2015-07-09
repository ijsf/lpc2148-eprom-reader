#!/usr/bin/python
#
#	lpc2148-eprom-reader
#
#  An 27-series EPROM reader/dumper based on a LPC2148 board.
#
#	Copyright (C) 2015 Cecill Etheredge / ijsf (c@ijsf.nl)
#
#	Redistribution and use in source and binary forms, with or without
#	modification, are permitted provided that the following conditions are met:
#
#	1. Redistributions of source code must retain the above copyright
#	   notice, this list of conditions and the following disclaimer.
#	2. Redistributions in binary form must reproduce the above copyright
#	   notice, this list of conditions and the following disclaimer in the
#	   documentation and/or other materials provided with the distribution.
#	3. The name of the author may not be used to endorse or promote products
#	   derived from this software without specific prior written permission.
#
#	THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
#	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#	OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
#	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
#	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
#	THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import serial, sys, getopt

# Dead simple dumper script
def usage():
	print 'tty_dump.py -t <tty name> -o <output file> -s <eprom size>'
	sys.exit(2)
	
def main(argv):
	eprom_size = 0
	tty_name = ''
	out_name = ''
	try:
		opts, args = getopt.getopt(argv,"ht:o:s:",["tty=","outfile=","size="])
	except getopt.GetoptError:
		usage()
	for opt, arg in opts:
		if opt == '-h':
			usage()
		elif opt in ("-t", "--tty"):
			tty_name = arg
		elif opt in ("-o", "--outfile"):
			out_name = arg
		elif opt in ("-s", "--size"):
			eprom_size = int(arg)
	if tty_name and out_name:
		with serial.Serial(tty_name, 115200) as tty_handle:
			with open(out_name,'w') as out_handle:
				# Write start command
				print "Writing start command"
				tty_handle.write('c');
		
				# Check leader sequence
				print "Dump starting, capturing %u bytes" % (eprom_size)
				leader = tty_handle.read(4)
				if leader == 'ijsf':
					c = 0
					while c < eprom_size:
						b = tty_handle.read(1)
						out_handle.write(b)
						c = c + 1
					print "Dump completed"
				else:
					print "ERROR: Leader sequence not found"
	else:
		usage()

if __name__ == "__main__":
	main(sys.argv[1:])
